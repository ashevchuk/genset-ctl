#include "bootloader_update.h"

#if BOOTLOADER_UPDATE_ENDPOINT_ENABLED

#include <esp_flash.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <string.h>
#include "esp_private/esp_flash_internal.h"
#include "rtos_config.h"
#include "debug.h"
#include "http_server.h"

extern SemaphoreHandle_t xFlashI2CGuard;

// Fixed by this project's own partition table (partitions.csv): the
// bootloader lives in the gap between the fixed ESP32 boot-ROM offset and
// this project's partition table (CONFIG_PARTITION_TABLE_OFFSET, idf's own
// default of 0x8000 — not overridden in sdkconfig.defaults). Matches
// compile.sh's own build output exactly ("Bootloader binary size 0x65f0
// bytes. 0xa10 bytes (9%) free" -> 0x65f0 + 0xa10 = 0x7000). Both values
// are sector-aligned (4KB) already, so no alignment math is needed before
// erasing.
#define BOOTLOADER_FLASH_OFFSET 0x1000UL
#define BOOTLOADER_FLASH_MAX_SIZE 0x7000UL  // 28672 bytes

// esp_flash's own driver refuses (aborts!) any erase/write whose address
// falls at-or-before the partition table (esp_flash_api.c's
// CHECK_WRITE_ADDRESS -> esp_partition_main_flash_region_safe(), which
// treats "<= ESP_PARTITION_TABLE_OFFSET + ESP_PARTITION_TABLE_MAX_LEN" as
// categorically unsafe, and esp_flash_api.c's UNSAFE_WRITE_ADDRESS macro
// literally calls abort() when CONFIG_SPI_FLASH_DANGEROUS_WRITE_ABORTS is
// set, this project's idf.py default) — exactly the bootloader region this
// whole route exists to touch. Found live 2026-09-25: the very first real
// test crashed immediately inside esp_flash_erase_region() itself, not
// anywhere in this file's own code.
//
// esp_flash_set_dangerous_write_protection(esp_flash_default_chip, false)
// (esp_private/esp_flash_internal.h — an ESP-IDF "private" header, but this
// specific function is documented and clearly meant for exactly this: "the
// runtime option to allow or disallow writing to dangerous areas such as
// the bootloader") flips just the no_protect flag this check reads, with
// no other side effect — the OTHER check in the same gate
// (esp_partition_is_flash_region_writable(), which only blocks partitions
// explicitly marked read-only) already passes regardless, since the
// bootloader region isn't a registered partition at all.
//
// Deliberately NOT esp_flash_app_disable_protect() (also in this header,
// and also used by espcoredump), despite the very similar name and despite
// briefly trying it here first. That function swaps the flash driver's
// ENTIRE os_func table to the "NOOS" (spi_flash_os_func_noos.c) variant —
// correct for espcoredump's own use case (writing a coredump from inside
// the panic handler, where the RTOS scheduler is no longer reliable
// anyway), catastrophic here: found live 2026-09-25, second real test —
// swapping to NOOS functions while the RTOS and every other task are still
// running normally starves the whole system for as long as it's active,
// long enough to trip the 15s Task Watchdog on some other task entirely
// (a real TWDT reset, "PRO CPU has been reset by WDT" — a different
// failure shape from the first test's abort(), both correctly caught by
// this project's own OTA-rollback mechanism, but neither is what this
// route should ever be causing). esp_flash_set_dangerous_write_protection()
// only flips a flag on the chip's existing, still fully RTOS-aware
// function table — the actually-surgical fix.
//
// Scoped as tightly as possible around just the actual flash operation:
// re-enabled immediately after, not left off for the rest of this boot —
// an unrelated bug elsewhere in the app writing flash at a bad address
// should still be caught while this route isn't actively using the
// exception, not silently corrupt something.

// A minimal sanity floor, not a real signature check: every valid ESP32
// image (bootloader or app) starts with this magic byte. Catches "wrong
// file entirely" before anything touches flash; does not catch "well-
// formed but wrong/corrupt bootloader" — this route is for a manual,
// one-time, self-supplied upload, not a scenario needing real signing.
#define ESP_IMAGE_MAGIC_BYTE 0xE9

static uint8_t *s_upload_buf = nullptr;
static size_t s_upload_len = 0;
static bool s_upload_ok = true;  // false = abort silently, response already decided

static void bootloader_update_reset_buffer(void) {
    if (s_upload_buf) {
        free(s_upload_buf);
        s_upload_buf = nullptr;
    }
    s_upload_len = 0;
    s_upload_ok = true;
}

#define BOOTLOADER_UPDATE_MAX_ATTEMPTS 3

// Erases + writes + read-back-verifies the bootloader region, retrying the
// whole erase/write/verify cycle up to BOOTLOADER_UPDATE_MAX_ATTEMPTS times
// before giving up. Retrying the erase too, not just the write, matters:
// esp_flash_erase_region() erases sector-by-sector under the hood, and a
// failure partway through can leave some sectors blank and others still
// holding the OLD bootloader's bytes — just as broken as a bad write, so a
// clean full erase is the right starting point for every attempt, not just
// the first. Verifying by reading the region back and comparing against
// the buffer we actually meant to write catches a write that returned
// ESP_OK but didn't actually land correctly — cheap for 28KB, and the same
// kind of check esptool's own USB flash already does after every write.
// Returns ESP_OK only once erase+write+verify all genuinely succeeded on
// some attempt; ESP_FAIL once every attempt has been exhausted.
static esp_err_t bootloader_flash_write_verified(const uint8_t *data, size_t len) {
    static uint8_t verify_chunk[512];

    for (int attempt = 1; attempt <= BOOTLOADER_UPDATE_MAX_ATTEMPTS; attempt++) {
        if (attempt > 1) {
            SLOG_PRINTF("[bootloader_update] Retrying (attempt %d/%d)...\n", attempt, BOOTLOADER_UPDATE_MAX_ATTEMPTS);
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_task_wdt_reset();
        }

        SLOG_PRINTF("[bootloader_update] Erasing 0x%lx bytes at offset 0x%lx (attempt %d/%d)...\n",
                    (unsigned long)BOOTLOADER_FLASH_MAX_SIZE, (unsigned long)BOOTLOADER_FLASH_OFFSET,
                    attempt, BOOTLOADER_UPDATE_MAX_ATTEMPTS);
        esp_err_t err = esp_flash_erase_region(NULL, BOOTLOADER_FLASH_OFFSET, BOOTLOADER_FLASH_MAX_SIZE);
        if (err != ESP_OK) {
            SLOG_PRINTF("[bootloader_update] Erase failed: %d\n", (int)err);
            continue;
        }

        SLOG_PRINTF("[bootloader_update] Writing %u bytes...\n", (unsigned)len);
        err = esp_flash_write(NULL, data, BOOTLOADER_FLASH_OFFSET, len);
        if (err != ESP_OK) {
            SLOG_PRINTF("[bootloader_update] Write failed: %d\n", (int)err);
            continue;
        }

        SLOG_PRINTLN("[bootloader_update] Verifying by reading back...");
        bool verify_ok = true;
        for (size_t off = 0; off < len; off += sizeof(verify_chunk)) {
            size_t chunk_len = len - off;
            if (chunk_len > sizeof(verify_chunk)) chunk_len = sizeof(verify_chunk);
            err = esp_flash_read(NULL, verify_chunk, BOOTLOADER_FLASH_OFFSET + off, chunk_len);
            if (err != ESP_OK || memcmp(verify_chunk, data + off, chunk_len) != 0) {
                verify_ok = false;
                break;
            }
            esp_task_wdt_reset();
        }
        if (!verify_ok) {
            SLOG_PRINTLN("[bootloader_update] Verify FAILED — written data doesn't match what was uploaded.");
            continue;
        }

        SLOG_PRINTF("[bootloader_update] Erase+write+verify succeeded on attempt %d/%d.\n", attempt, BOOTLOADER_UPDATE_MAX_ATTEMPTS);
        return ESP_OK;
    }
    return ESP_FAIL;
}

// Collects the file part into s_upload_buf (bounded by the bootloader
// region size).
class BootloaderUpload : public UploadHandler {
public:
    explicit BootloaderUpload(bool confirmed) : confirmed_(confirmed) {}
    bool on_file_start(const std::string &field, const std::string &filename) override {
        (void)field;
        if (!confirmed_) return true;  // refusal already recorded in s_upload_ok
        bootloader_update_reset_buffer();
        s_upload_buf = (uint8_t *)malloc(BOOTLOADER_FLASH_MAX_SIZE);
        if (!s_upload_buf) {
            SLOG_PRINTLN("[bootloader_update] REFUSED: malloc failed.");
            s_upload_ok = false;
            return true;
        }
        SLOG_PRINTF("[bootloader_update] Upload started: %s\n", filename.c_str());
        return true;
    }
    bool on_file_data(const uint8_t *data, size_t len) override {
        if (!s_upload_ok || !s_upload_buf) return true;
        if (s_upload_len + len > BOOTLOADER_FLASH_MAX_SIZE) {
            SLOG_PRINTF("[bootloader_update] REFUSED: upload exceeds max bootloader size (0x%lx bytes) — not a plausible bootloader image.\n",
                        (unsigned long)BOOTLOADER_FLASH_MAX_SIZE);
            s_upload_ok = false;
            return true;
        }
        memcpy(s_upload_buf + s_upload_len, data, len);
        s_upload_len += len;
        return true;
    }
    bool on_file_end() override { return true; }
    void on_file_end_aborted() override { bootloader_update_reset_buffer(); }

private:
    bool confirmed_;
};

static esp_err_t send_plain(httpd_req_t *req, const char *status, const char *body) {
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, body);
}

esp_err_t bootloader_update_handler(httpd_req_t *req) {
    char query[64] = "";
    char confirm[8] = "";
    bool confirmed = httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
                     httpd_query_key_value(query, "confirm", confirm, sizeof(confirm)) == ESP_OK &&
                     strcmp(confirm, "1") == 0;

    bootloader_update_reset_buffer();
    if (!confirmed) {
        SLOG_PRINTLN("[bootloader_update] REFUSED: missing confirm=1 query param.");
        s_upload_ok = false;
    }
    BootloaderUpload upload(confirmed);
    bool complete = false;
    if (!receive_multipart(req, upload, complete) || !complete) {
        bootloader_update_reset_buffer();
        return ESP_FAIL;
    }

    if (!s_upload_ok) {
        bootloader_update_reset_buffer();
        return send_plain(req, "400 Bad Request", "FAIL (see serial log for reason)");
    }
    if (!s_upload_buf || s_upload_len == 0) {
        return send_plain(req, "400 Bad Request", "FAIL (no data received)");
    }
    if (s_upload_buf[0] != ESP_IMAGE_MAGIC_BYTE) {
        SLOG_PRINTLN("[bootloader_update] REFUSED: uploaded file doesn't start with the ESP image magic byte (0xE9) — not touching flash.");
        bootloader_update_reset_buffer();
        return send_plain(req, "400 Bad Request", "FAIL (not a valid ESP image — wrong magic byte)");
    }

    bool held = (xSemaphoreTake(xFlashI2CGuard, pdMS_TO_TICKS(600)) == pdTRUE);
    esp_flash_set_dangerous_write_protection(esp_flash_default_chip, false);
    esp_err_t err = bootloader_flash_write_verified(s_upload_buf, s_upload_len);
    esp_flash_set_dangerous_write_protection(esp_flash_default_chip, true);
    if (held) xSemaphoreGive(xFlashI2CGuard);

    if (err != ESP_OK) {
        // Every attempt (erase+write+verify, retried
        // BOOTLOADER_UPDATE_MAX_ATTEMPTS times) failed. The bootloader region
        // is now in an unknown state; there is no further software fallback.
        SLOG_PRINTF("[bootloader_update] FAILED after %d attempts — bootloader flash region is now in an UNKNOWN/PARTIAL state. Do not power-cycle. USB recovery will be required.\n", BOOTLOADER_UPDATE_MAX_ATTEMPTS);
        bootloader_update_reset_buffer();
        return send_plain(req, "500 Internal Server Error",
                          "FAIL (all retries exhausted — bootloader region left in an unknown state, USB recovery needed)");
    }

    SLOG_PRINTLN("[bootloader_update] Write verified OK. Rebooting to confirm the new bootloader...");
    send_plain(req, "200 OK", "OK. Rebooting...");
    bootloader_update_reset_buffer();
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
    return ESP_OK;
}

#endif  // BOOTLOADER_UPDATE_ENDPOINT_ENABLED
