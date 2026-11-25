#include "ffat.h"
#include <esp_vfs_fat.h>
#include <wear_levelling.h>

#define FFAT_PARTITION_LABEL "ffat"
#define FFAT_MAX_OPEN_FILES  10

static wl_handle_t s_wl = WL_INVALID_HANDLE;

bool ffat_mount(void) {
    if (s_wl != WL_INVALID_HANDLE) return true;
    esp_vfs_fat_mount_config_t conf = {};
    conf.format_if_mount_failed = true;
    conf.max_files = FFAT_MAX_OPEN_FILES;
    conf.allocation_unit_size = CONFIG_WL_SECTOR_SIZE;
    if (esp_vfs_fat_spiflash_mount_rw_wl(FFAT_MOUNT_POINT, FFAT_PARTITION_LABEL, &conf, &s_wl) != ESP_OK) {
        s_wl = WL_INVALID_HANDLE;
        return false;
    }
    return true;
}

bool ffat_info(uint64_t *total_bytes, uint64_t *free_bytes) {
    return esp_vfs_fat_info(FFAT_MOUNT_POINT, total_bytes, free_bytes) == ESP_OK;
}
