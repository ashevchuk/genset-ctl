#include "rtos_config.h"  // pulls in Arduino.h; must precede lwip (INADDR_NONE clash)
#include "http_server.h"
#include "config.h"
#include "fram_log.h"
#include "uptime.h"
#include "statemachine.h"
#include "shiftreg.h"
#include "mcp39f511a_dev.h"
#include "powermeter.h"
#include "wifi_setup.h"
#include "esp_now_link.h"
#include "oled_dev.h"
#include "bootloader_update.h"
#include "multipart.h"
#include <ArduinoJson.h>
#include <esp_heap_caps.h>
#include <lwip/sockets.h>
#include "debug.h"
#include <esp_system.h>
#include <map>
#include <esp_http_server.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_task_wdt.h>
#include <esp_vfs_fat.h>
#include <dirent.h>
#include <sys/stat.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <time.h>

#include "ffat.h"
#define FS_MOUNT_POINT FFAT_MOUNT_POINT

#define HTTP_MAX_OPEN_SOCKETS 16
#define HTTP_MAX_URI_HANDLERS 48
#define HTTP_TASK_STACK       8192
#define HTTP_TASK_PRIORITY    TASK_PRIORITY_WEB
#define HTTP_TASK_CORE        0

#define STATIC_REQUEST_RATE_LIMIT     8
#define STATIC_REQUEST_RATE_WINDOW_MS 1000
#define STREAM_CHUNK_SIZE             512
#define QUERY_MAX_LEN                 512
#define FORM_BODY_MAX_LEN             8192
#define UPLOAD_RECV_CHUNK             1460
#define UPLOAD_RECV_TIMEOUT_RETRIES   5
#define WS_MAX_CLIENTS                16
#define WS_PING_INTERVAL_MS           15000
#define WS_PONG_TIMEOUT_MS            5000
#define WS_MISSED_PONGS_TO_DROP       2
#define WS_RX_MAX_LEN                 128
#define WS_BROADCAST_MAX_LEN          1536
#define HTTP_SEND_WAIT_TIMEOUT_S      2  // bounds each send to a stalled client

// Handlers run in the httpd task, which is not subscribed to the TWDT.
static void wdt_reset_if_subscribed(void) {
    if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset();
}

// ---- request helpers ------------------------------------------------------

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// In-place application/x-www-form-urlencoded decoding ('+' and %XX), as
// WebServer::arg() returned values.
static void url_decode(char *s) {
    char *o = s;
    for (char *p = s; *p; p++) {
        if (*p == '+') {
            *o++ = ' ';
        } else if (*p == '%' && hex_value(p[1]) >= 0 && hex_value(p[2]) >= 0) {
            *o++ = (char)(hex_value(p[1]) * 16 + hex_value(p[2]));
            p += 2;
        } else {
            *o++ = *p;
        }
    }
    *o = '\0';
}

// True if `key` is present in the query string; its decoded value is
// copied into `out` (empty string for a bare key).
static bool query_arg(httpd_req_t *req, const char *key, char *out, size_t out_len) {
    char query[QUERY_MAX_LEN];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;
    if (httpd_query_key_value(query, key, out, out_len) != ESP_OK) return false;
    url_decode(out);
    return true;
}

// Same parsing as Arduino String::toInt() (atol: leading digits, 0 on garbage).
static long query_long(httpd_req_t *req, const char *key, long fallback, bool *present = nullptr) {
    char buf[32];
    bool has = query_arg(req, key, buf, sizeof(buf));
    if (present) *present = has;
    return has ? atol(buf) : fallback;
}

static esp_err_t send_text(httpd_req_t *req, const char *status, const char *type, const char *body) {
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, type);
    return httpd_resp_sendstr(req, body);
}

// Query-string plus application/x-www-form-urlencoded body arguments, in the
// order WebServer::arg() saw them (query first; first occurrence of a key
// wins).
class FormArgs {
public:
    bool load(httpd_req_t *req) {
        size_t qlen = httpd_req_get_url_query_len(req);
        if (qlen > 0) {
            std::string q(qlen + 1, '\0');
            if (httpd_req_get_url_query_str(req, &q[0], q.size()) == ESP_OK) parse(q.c_str());
        }
        if (req->content_len == 0) return true;
        if (req->content_len > FORM_BODY_MAX_LEN) return false;
        std::string body(req->content_len, '\0');
        size_t got = 0;
        while (got < req->content_len) {
            int n = httpd_req_recv(req, &body[got], req->content_len - got);
            if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
            if (n <= 0) return false;
            got += (size_t)n;
        }
        parse(body.c_str());
        return true;
    }
    bool has(const char *key) const { return args_.count(key) > 0; }
    std::string get(const char *key) const {
        auto it = args_.find(key);
        return it == args_.end() ? std::string() : it->second;
    }

private:
    void parse(const char *s) {
        while (*s) {
            const char *end = strchr(s, '&');
            std::string pair(s, end ? (size_t)(end - s) : strlen(s));
            size_t eq = pair.find('=');
            std::string key = pair.substr(0, eq);
            std::string val = (eq == std::string::npos) ? std::string() : pair.substr(eq + 1);
            url_decode(&key[0]);
            key.resize(strlen(key.c_str()));
            url_decode(&val[0]);
            val.resize(strlen(val.c_str()));
            if (!key.empty() && !has(key.c_str())) args_[key] = val;
            if (!end) break;
            s = end + 1;
        }
    }
    std::map<std::string, std::string> args_;
};

// ---- static files ---------------------------------------------------------

static bool static_request_rate_exceeded(void) {
    static uint32_t window_start = 0;
    static uint8_t window_count = 0;
    uint32_t now = uptime_ms();
    if (now - window_start >= STATIC_REQUEST_RATE_WINDOW_MS) {
        window_start = now;
        window_count = 0;
    }
    window_count++;
    return window_count > STATIC_REQUEST_RATE_LIMIT;
}

static bool ends_with(const std::string &s, const char *suffix) {
    size_t n = strlen(suffix);
    return s.size() >= n && s.compare(s.size() - n, n, suffix) == 0;
}

static const char *mime_type_for_path(const char *path) {
    std::string p(path);
    for (char &c : p) c = (char)tolower((unsigned char)c);
    if (ends_with(p, ".gz")) p.resize(p.size() - 3);
    if (ends_with(p, ".html") || ends_with(p, ".htm")) return "text/html";
    if (ends_with(p, ".css")) return "text/css";
    if (ends_with(p, ".js")) return "application/javascript";
    if (ends_with(p, ".json")) return "application/json";
    if (ends_with(p, ".png")) return "image/png";
    if (ends_with(p, ".jpg") || ends_with(p, ".jpeg")) return "image/jpeg";
    if (ends_with(p, ".gif")) return "image/gif";
    if (ends_with(p, ".svg")) return "image/svg+xml";
    if (ends_with(p, ".ico")) return "image/x-icon";
    if (ends_with(p, ".txt")) return "text/plain";
    if (ends_with(p, ".xml")) return "application/xml";
    if (ends_with(p, ".pdf")) return "application/pdf";
    if (ends_with(p, ".zip")) return "application/zip";
    return "application/octet-stream";
}

// Update mode serves requests before setup_rtos() has created the guard.
static bool take_flash_guard(void) {
    return xFlashI2CGuard != NULL && xSemaphoreTake(xFlashI2CGuard, pdMS_TO_TICKS(600)) == pdTRUE;
}

static void give_flash_guard(bool held) {
    if (held) xSemaphoreGive(xFlashI2CGuard);
}

static bool is_regular_file(const char *vfs_path) {
    struct stat st;
    return stat(vfs_path, &st) == 0 && S_ISREG(st.st_mode);
}

// Serves `filePath` (FS-relative, leading '/'), preferring a ".gz" sibling.
// The flash guard is held only around each file operation, not the whole
// transfer, so a slow client can't starve I2CTask.
static esp_err_t serve_static(httpd_req_t *req, const char *filePath, const char *contentTypeOverride = nullptr,
                              bool cacheable = false) {
    if (static_request_rate_exceeded()) {
        return send_text(req, "503 Service Unavailable", "text/plain", "Server busy, try again shortly");
    }

    std::string plain = std::string(FS_MOUNT_POINT) + filePath;
    std::string gz = plain + ".gz";

    bool held = take_flash_guard();
    bool useGzip = is_regular_file(gz.c_str());
    FILE *f = nullptr;
    if (useGzip || is_regular_file(plain.c_str())) {
        f = fopen((useGzip ? gz : plain).c_str(), "rb");
    }
    give_flash_guard(held);
    if (f == nullptr) {
        std::string body = std::string("File not found: ") + filePath;
        return send_text(req, "404 Not Found", "text/plain", body.c_str());
    }

    httpd_resp_set_type(req, contentTypeOverride ? contentTypeOverride : mime_type_for_path(filePath));
    if (useGzip) httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    if (cacheable) httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=3600");

    char buf[STREAM_CHUNK_SIZE];
    unsigned chunk = 0;
    esp_err_t err = ESP_OK;
    for (;;) {
        bool chunkHeld = take_flash_guard();
        size_t n = fread(buf, 1, sizeof(buf), f);
        give_flash_guard(chunkHeld);
        if (n == 0) break;
        err = httpd_resp_send_chunk(req, buf, n);
        if (err != ESP_OK) break;
        if ((++chunk & 7) == 0) wdt_reset_if_subscribed();
    }
    fclose(f);
    if (err != ESP_OK) return err;
    return httpd_resp_send_chunk(req, nullptr, 0);
}

struct StaticPage {
    const char *uri;
    const char *file;
};

static const StaticPage kPages[] = {
    {"/", "/index.html"},
    {"/fw", "/fw.html"},
    {"/journal", "/journal.html"},
    {"/wifi-config", "/wifi-config.html"},
    {"/files", "/fm.html"},
};

static esp_err_t page_handler(httpd_req_t *req) {
    const StaticPage *page = (const StaticPage *)req->user_ctx;
    return serve_static(req, page->file);
}

// Browsers request this on every page load regardless of any <link rel="icon">.
static esp_err_t favicon_handler(httpd_req_t *req) {
    return serve_static(req, "/favicon.ico", "image/x-icon", true);
}

static esp_err_t static_handler(httpd_req_t *req) {
    const char *uri = req->uri;
    const char *query = strchr(uri, '?');
    std::string path(uri + strlen("/static/"), query ? (size_t)(query - uri) - strlen("/static/") : strlen(uri) - strlen("/static/"));
    if (path.empty() || path.find("..") != std::string::npos) {
        return send_text(req, "403 Forbidden", "text/plain", "Forbidden");
    }
    std::string filePath = "/" + path;
    return serve_static(req, filePath.c_str(), mime_type_for_path(path.c_str()), true);
}

// ---- journal / metrics ----------------------------------------------------

static esp_err_t metrics_handler(httpd_req_t *req) {
    char buf[768];
    fram_metrics_json(buf, sizeof(buf), g_limits_config.maint_interval_hours);
    return send_text(req, "200 OK", "application/json", buf);
}

static esp_err_t log_handler(httpd_req_t *req) {
    uint32_t newest = fram_log_newest_seq();
    uint32_t oldest = fram_log_oldest_seq();

    bool has_count = false;
    long count = query_long(req, "count", 20, &has_count);
    if (has_count) {
        if (count < 1) count = 1;
        if (count > 50) count = 50;
    }
    // Optional filter group — same names as journal.html FILTERS / EV_META
    // ("engine"/"charge"/"alarm"/"maint"/"sys"). Empty or "all" = no filter.
    char group[64] = "";
    if (!query_arg(req, "group", group, sizeof(group))) group[0] = '\0';
    if (strcmp(group, "all") == 0) group[0] = '\0';
    bool filter = group[0] != '\0';

    uint32_t from = newest;
    bool has_before = false;
    uint32_t before = (uint32_t)query_long(req, "before", 0, &has_before);
    if (has_before) {
        from = (before > 0) ? before - 1 : 0;
        if (from > newest) from = newest;
    }
    // Exclusive lower bound (live poll: only events newer than this seq).
    uint32_t after = (uint32_t)query_long(req, "after", 0);

    // Cap I2C reads per request so a sparse filter can't monopolize the
    // server/I2C for a full 4080-record walk in one request.
    const int max_scan = filter ? 400 : (int)count;
    int want = (int)count;
    int scanned = 0;
    int matched = 0;
    uint32_t scanned_to = from;

    std::string out;
    out.reserve(count * 220 + 128);
    char num[16];
    out += "{\"newest\":";
    snprintf(num, sizeof(num), "%lu", (unsigned long)newest);
    out += num;
    out += ",\"oldest\":";
    snprintf(num, sizeof(num), "%lu", (unsigned long)oldest);
    out += num;
    out += ",\"ts_now\":";
    snprintf(num, sizeof(num), "%lu", (unsigned long)(uint32_t)time(NULL));
    out += num;
    out += ",\"group\":\"";
    out += filter ? group : "all";
    out += "\",\"events\":[";

    char jbuf[256];
    FramLogRecord batch[FLOG_READ_BATCH_MAX];
    bool batch_ok[FLOG_READ_BATCH_MAX];
    int batch_n = 0, batch_i = 0;
    bool first = true;
    for (uint32_t seq = from; seq >= oldest && seq > after && want > 0 && scanned < max_scan; seq--) {
        if (batch_i >= batch_n) {
            batch_n = fram_log_read_batch(seq, FLOG_READ_BATCH_MAX, batch, batch_ok);
            batch_i = 0;
            if (batch_n == 0) break;  // the I2C job itself failed; retrying per seq would only stall
        }
        scanned++;
        scanned_to = seq;
        if ((scanned & 7) == 0) wdt_reset_if_subscribed();
        const FramLogRecord &rec = batch[batch_i];
        bool rec_ok = batch_ok[batch_i];
        batch_i++;
        if (!rec_ok) continue;
        if (filter && strcmp(fram_log_group_name(rec.type), group) != 0) continue;
        if (fram_log_record_json(&rec, jbuf, sizeof(jbuf)) <= 0) continue;
        if (!first) out += ",";
        out += jbuf;
        first = false;
        matched++;
        want--;
    }
    wdt_reset_if_subscribed();
    // more=true when older matching rows may still exist below scanned_to.
    bool hit_count = (matched >= count);
    bool hit_scan_cap = (scanned >= max_scan) && (scanned_to > oldest) && (scanned_to > after + 1);
    bool more = (scanned_to > oldest) && (scanned_to > after + 1) && (hit_count || hit_scan_cap);
    out += "],\"returned\":";
    snprintf(num, sizeof(num), "%d", matched);
    out += num;
    out += ",\"scanned\":";
    snprintf(num, sizeof(num), "%d", scanned);
    out += num;
    out += ",\"scanned_to\":";
    snprintf(num, sizeof(num), "%lu", (unsigned long)(scanned > 0 ? scanned_to : 0));
    out += num;
    out += ",\"more\":";
    out += more ? "true" : "false";
    out += "}";
    return send_text(req, "200 OK", "application/json", out.c_str());
}

// Raw dump of the coredump partition, decoded off-device by decode_crash.sh.
static esp_err_t coredump_handler(httpd_req_t *req) {
    const esp_partition_t *part =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, NULL);
    if (!part) {
        return send_text(req, "404 Not Found", "text/plain", "No coredump partition in partitions.csv");
    }
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"coredump.bin\"");
    uint8_t buf[STREAM_CHUNK_SIZE];
    size_t sent = 0;
    unsigned chunk = 0;
    while (sent < part->size) {
        size_t n = (part->size - sent < sizeof(buf)) ? (part->size - sent) : sizeof(buf);
        bool held = take_flash_guard();
        esp_err_t err = esp_partition_read(part, sent, buf, n);
        give_flash_guard(held);
        if (err != ESP_OK) break;
        if (httpd_resp_send_chunk(req, (const char *)buf, n) != ESP_OK) return ESP_FAIL;
        sent += n;
        if ((++chunk & 7) == 0) wdt_reset_if_subscribed();
    }
    return httpd_resp_send_chunk(req, nullptr, 0);
}

// ---- firmware partitions --------------------------------------------------

static const char *fw_ota_state_name(esp_ota_img_states_t state) {
    switch (state) {
        case ESP_OTA_IMG_NEW: return "new";
        case ESP_OTA_IMG_PENDING_VERIFY: return "pending_verify";
        case ESP_OTA_IMG_VALID: return "valid";
        case ESP_OTA_IMG_INVALID: return "invalid";
        case ESP_OTA_IMG_ABORTED: return "aborted";
        default: return "undefined";
    }
}

static void fw_append_partition_json(char *buf, size_t bufSize, size_t &off, const esp_partition_t *part,
                                     bool running) {
    esp_app_desc_t desc = {};
    bool haveDesc = (part != nullptr) && (esp_ota_get_partition_description(part, &desc) == ESP_OK);
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    if (part != nullptr) esp_ota_get_state_partition(part, &state);

    off += snprintf(buf + off, bufSize - off,
                    "{\"label\":\"%s\",\"running\":%s,\"state\":\"%s\","
                    "\"has_image\":%s,\"version\":\"%s\",\"date\":\"%s\",\"time\":\"%s\"}",
                    part ? part->label : "", running ? "true" : "false", fw_ota_state_name(state),
                    haveDesc ? "true" : "false", haveDesc ? desc.version : "", haveDesc ? desc.date : "",
                    haveDesc ? desc.time : "");
}

static esp_err_t fw_partitions_handler(httpd_req_t *req) {
    char buf[512];
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *other = esp_ota_get_next_update_partition(NULL);
    size_t off = 0;
    off += snprintf(buf + off, sizeof(buf) - off, "{\"running\":");
    fw_append_partition_json(buf, sizeof(buf), off, running, true);
    off += snprintf(buf + off, sizeof(buf) - off, ",\"other\":");
    fw_append_partition_json(buf, sizeof(buf), off, other, false);
    snprintf(buf + off, sizeof(buf) - off, "}");
    return send_text(req, "200 OK", "application/json", buf);
}

// ---- config / file manager reads ------------------------------------------

static esp_err_t wifi_config_handler(httpd_req_t *req) {
    std::string path = std::string(FS_MOUNT_POINT) + WIFI_JSON_PATH;
    bool held = take_flash_guard();
    FILE *f = is_regular_file(path.c_str()) ? fopen(path.c_str(), "rb") : nullptr;
    if (f == nullptr) {
        give_flash_guard(held);
        return send_text(req, "404 Not Found", "application/json", "{\"error\":\"not found\"}");
    }
    std::string buf(WIFI_JSON_SIZE, '\0');
    size_t n = fread(&buf[0], 1, WIFI_JSON_SIZE, f);
    fclose(f);
    give_flash_guard(held);
    buf.resize(n);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf.data(), buf.size());
}

static void json_append_escaped(std::string &out, const char *s) {
    for (; *s; s++) {
        if (*s == '"' || *s == '\\') out += '\\';
        out += *s;
    }
}

static esp_err_t files_handler(httpd_req_t *req) {
    char arg[256];
    std::string path = query_arg(req, "path", arg, sizeof(arg)) ? arg : "/";
    if (path.find("..") != std::string::npos) {
        return send_text(req, "403 Forbidden", "application/json", "{\"error\":\"Forbidden\"}");
    }
    if (path.empty() || path[0] != '/') path = "/" + path;
    // Canonical form: no trailing slash except root.
    while (path.size() > 1 && path.back() == '/') path.pop_back();

    std::string vfs_dir = std::string(FS_MOUNT_POINT) + (path == "/" ? "" : path);
    bool held = take_flash_guard();
    DIR *dir = opendir(vfs_dir.c_str());
    if (dir == nullptr && path != "/") {
        // A concurrent static-file read can make a momentarily-busy FAT open
        // fail even for a real directory.
        vTaskDelay(pdMS_TO_TICKS(20));
        wdt_reset_if_subscribed();
        dir = opendir(vfs_dir.c_str());
    }
    if (dir == nullptr) {
        give_flash_guard(held);
        return send_text(req, "404 Not Found", "application/json", "{\"error\":\"Directory not found\"}");
    }

    std::string json = "{\"path\":\"";
    json += path;
    json += "\",\"files\":[";
    bool first = true;
    unsigned n_entries = 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string entryPath = (path == "/") ? (std::string("/") + entry->d_name) : (path + "/" + entry->d_name);
        std::string vfs_entry = std::string(FS_MOUNT_POINT) + entryPath;
        struct stat st = {};
        stat(vfs_entry.c_str(), &st);
        bool isDir = S_ISDIR(st.st_mode);

        if (!first) json += ",";
        first = false;
        char size[16];
        snprintf(size, sizeof(size), "%lu", isDir ? 0UL : (unsigned long)st.st_size);
        json += "{\"name\":\"";
        json_append_escaped(json, entry->d_name);
        json += "\",\"path\":\"";
        json_append_escaped(json, entryPath.c_str());
        json += "\",\"isDir\":";
        json += isDir ? "true" : "false";
        json += ",\"size\":";
        json += size;
        json += "}";
        if ((++n_entries & 15) == 0) wdt_reset_if_subscribed();
    }
    closedir(dir);
    give_flash_guard(held);

    uint64_t total = 0, free_bytes = 0;
    ffat_info(&total, &free_bytes);
    char fsinfo[96];
    snprintf(fsinfo, sizeof(fsinfo), "],\"fsinfo\":{\"total\":%llu,\"used\":%llu,\"free\":%llu}}",
             (unsigned long long)total, (unsigned long long)(total - free_bytes), (unsigned long long)free_bytes);
    json += fsinfo;
    return send_text(req, "200 OK", "application/json", json.c_str());
}

// ---- commands ---------------------------------------------------------------

static esp_err_t send_json(httpd_req_t *req, const char *status, const char *body) {
    return send_text(req, status, "application/json", body);
}

static bool confirmed(const FormArgs &a) {
    return a.has("confirm") && a.get("confirm") == "1";
}

static void queue_command(WebCommandType_t type) {
    if (xWebCommandQueue == NULL) return;
    WebCommand_t cmd = {};
    cmd.type = type;
    cmd.source = FLOG_SRC_HTTP;
    xQueueSend(xWebCommandQueue, &cmd, 0);
}

static esp_err_t start_handler(httpd_req_t *req) {
    esp_err_t err = send_text(req, "200 OK", "text/plain", "OK");
    queue_command(WEB_CMD_START);
    return err;
}

static esp_err_t start_retry_handler(httpd_req_t *req) {
    esp_err_t err = send_text(req, "200 OK", "text/plain", "OK");
    queue_command(WEB_CMD_START_RETRY);
    return err;
}

static esp_err_t stop_handler(httpd_req_t *req) {
    esp_err_t err = send_text(req, "200 OK", "text/plain", "OK");
    queue_command(WEB_CMD_STOP);
    return err;
}

static esp_err_t reg_handler(httpd_req_t *req) {
    FormArgs a;
    a.load(req);
    bool set = true;
    if (a.has("set")) set = (bool)atol(a.get("set").c_str());
    if (a.has("reg")) {
        int8_t reg = (int8_t)atol(a.get("reg").c_str());
        if (reg > -1 && reg < 16 && xWebCommandQueue != NULL) {
            WebCommand_t cmd = {};
            cmd.type = WEB_CMD_REG_SET;
            cmd.reg = (uint8_t)reg;
            cmd.state = set;
            cmd.source = FLOG_SRC_HTTP;
            xQueueSend(xWebCommandQueue, &cmd, 0);
        }
    }
    return send_text(req, "200 OK", "text/plain", "OK");
}

static esp_err_t log_wipe_handler(httpd_req_t *req) {
    FormArgs a;
    a.load(req);
    if (!confirmed(a)) return send_json(req, "400 Bad Request", "{\"error\":\"add confirm=1\"}");
    bool metrics_too = a.has("metrics") && a.get("metrics") == "1";
    fram_log_wipe(metrics_too);
    return send_json(req, "200 OK", "{\"ok\":1}");
}

static esp_err_t energy_reset_handler(httpd_req_t *req) {
    FormArgs a;
    a.load(req);
    if (!confirmed(a)) return send_json(req, "400 Bad Request", "{\"error\":\"add confirm=1\"}");
    // PowerMeterTask holds xSerial2Mutex across a full measurement burst,
    // routinely longer than MUTEX_TIMEOUT_MS; wait out one PowerMeter tick.
    bool got_lock = false;
    bool ok = false;
    if (xSerial2Mutex != NULL && xSemaphoreTake(xSerial2Mutex, pdMS_TO_TICKS(3000)) == pdTRUE) {
        got_lock = true;
        ok = mcp39f511a.resetEnergyCounters();
        GIVE_MUTEX(xSerial2Mutex);
    }
    if (!got_lock) return send_json(req, "503 Service Unavailable", "{\"error\":\"uart busy\"}");
    if (!ok) return send_json(req, "500 Internal Server Error", "{\"error\":\"reset failed\"}");
    powermeter_note_energy_reset();
    return send_json(req, "200 OK", "{\"ok\":1}");
}

static esp_err_t metrics_set_handler(httpd_req_t *req) {
    FormArgs a;
    a.load(req);
    if (!confirmed(a)) return send_json(req, "400 Bad Request", "{\"error\":\"add confirm=1\"}");

    FramMetricsPatch_t p = {};
    char err[80];
    auto take_f = [&](const char *name, uint32_t flag, float *dst) -> bool {
        if (!a.has(name)) return true;
        float v = (float)atof(a.get(name).c_str());
        if (v < 0.0f) {
            snprintf(err, sizeof(err), "{\"error\":\"%s must be >= 0\"}", name);
            return false;
        }
        p.fields |= flag;
        *dst = v;
        return true;
    };
    auto take_u = [&](const char *name, uint32_t flag, uint32_t *dst) -> bool {
        if (!a.has(name)) return true;
        long v = atol(a.get(name).c_str());
        if (v < 0) {
            snprintf(err, sizeof(err), "{\"error\":\"%s must be >= 0\"}", name);
            return false;
        }
        p.fields |= flag;
        *dst = (uint32_t)v;
        return true;
    };

    bool ok = true;
    // hours is the original name; run_hours matches GET /metrics.
    if (a.has("run_hours") || a.has("hours")) {
        ok = take_f(a.has("run_hours") ? "run_hours" : "hours", FRAM_METRICS_SET_RUN_HOURS, &p.run_hours);
    }
    ok = ok && take_f("charge_hours", FRAM_METRICS_SET_CHARGE_HOURS, &p.charge_hours);
    ok = ok && take_u("starts", FRAM_METRICS_SET_STARTS, &p.starts);
    ok = ok && take_u("start_fails", FRAM_METRICS_SET_START_FAILS, &p.start_fails);
    ok = ok && take_u("emergencies", FRAM_METRICS_SET_EMERGENCIES, &p.emergencies);
    ok = ok && take_u("charge_cycles", FRAM_METRICS_SET_CHARGE_CYCLES, &p.charge_cycles);
    ok = ok && take_f("max_head_temp", FRAM_METRICS_SET_MAX_HEAD_TEMP, &p.max_head_temp);
    ok = ok && take_f("max_rpm", FRAM_METRICS_SET_MAX_RPM, &p.max_rpm);
    ok = ok && take_f("max_power", FRAM_METRICS_SET_MAX_POWER, &p.max_power);
    if (!ok) return send_json(req, "400 Bad Request", err);

    if (p.fields == 0) return send_json(req, "400 Bad Request", "{\"error\":\"no fields\"}");
    if (!fram_metrics_set(&p)) return send_json(req, "400 Bad Request", "{\"error\":\"set failed\"}");
    char metrics[768];
    fram_metrics_json(metrics, sizeof(metrics), g_limits_config.maint_interval_hours);
    char buf[800];
    if (metrics[0] == '{') {
        snprintf(buf, sizeof(buf), "{\"ok\":1,%s", metrics + 1);
    } else {
        snprintf(buf, sizeof(buf), "{\"ok\":1}");
    }
    return send_json(req, "200 OK", buf);
}

static esp_err_t maintenance_handler(httpd_req_t *req) {
    FormArgs a;
    a.load(req);
    if (!a.has("type")) return send_json(req, "400 Bad Request", "{\"error\":\"type required\"}");
    int sub = fram_log_maint_parse(a.get("type").c_str());
    if (sub < 0) return send_json(req, "400 Bad Request", "{\"error\":\"unknown type\"}");
    std::string note = a.get("note");
    bool ok = fram_log_maintenance((uint8_t)sub, note.c_str());
    char buf[96];
    snprintf(buf, sizeof(buf), "{\"ok\":%d,\"type\":\"%s\"}", ok ? 1 : 0, fram_log_maint_name((uint8_t)sub));
    return send_json(req, ok ? "200 OK" : "503 Service Unavailable", buf);
}

static esp_err_t reset_handler(httpd_req_t *req) {
    send_text(req, "200 OK", "text/plain", "OK");
    // Give lwip time to put the response on the wire before the reboot.
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
    return ESP_OK;
}

static esp_err_t fw_switch_slot_handler(httpd_req_t *req) {
    FormArgs a;
    a.load(req);
    if (!confirmed(a)) return send_json(req, "400 Bad Request", "{\"error\":\"add confirm=1\"}");
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target) return send_json(req, "500 Internal Server Error", "{\"error\":\"no alternate OTA partition\"}");
    esp_app_desc_t desc = {};
    if (esp_ota_get_partition_description(target, &desc) != ESP_OK) {
        return send_json(req, "400 Bad Request", "{\"error\":\"alternate slot has no valid image\"}");
    }
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    esp_ota_get_state_partition(target, &state);
    if (state == ESP_OTA_IMG_INVALID || state == ESP_OTA_IMG_ABORTED) {
        return send_json(req, "400 Bad Request", "{\"error\":\"alternate slot marked invalid, refusing\"}");
    }
    if (esp_ota_set_boot_partition(target) != ESP_OK) {
        return send_json(req, "500 Internal Server Error", "{\"error\":\"failed to set boot partition\"}");
    }
    send_json(req, "200 OK", "{\"ok\":1,\"rebooting\":1}");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
    return ESP_OK;
}

// ---- WiFi / settings save ---------------------------------------------------

// Same fields as web.cpp's parse_wifi_core_fields(), read from `a`.
static void parse_wifi_core_fields(const FormArgs &a, JsonDocument &doc) {
    auto p = [&](const char *k) { return a.get(k); };
    doc["mode"] = p("mode");
    doc["hostname"] = p("hostname");
    JsonObject ap = doc["ap"].to<JsonObject>();
    ap["ssid"] = p("ap_ssid");
    ap["password"] = p("ap_pass");
    int ch = atoi(p("ap_ch").c_str());
    ap["channel"] = (ch >= 1 && ch <= 13) ? ch : 1;
    ap["hidden"] = (atoi(p("ap_hidden").c_str()) != 0);
    ap["max_conn"] = 4;
    ap["ip"] = "192.168.4.1";
    ap["gw"] = "192.168.4.1";
    ap["sn"] = "255.255.255.0";
    JsonObject sta = doc["sta"].to<JsonObject>();
    sta["dhcp"] = (atoi(p("sta_dhcp").c_str()) != 0);
    sta["ip"] = p("sta_ip");
    sta["gw"] = p("sta_gw");
    sta["sn"] = p("sta_sn");
    sta["dns1"] = p("sta_dns1");
    sta["dns2"] = p("sta_dns2");
    JsonArray nets = sta["networks"].to<JsonArray>();
    for (int i = 1; i <= WIFI_NETWORKS_MAX; i++) {
        char sk[16], pk[16];
        snprintf(sk, sizeof(sk), "sta_ssid_%d", i);
        snprintf(pk, sizeof(pk), "sta_pass_%d", i);
        std::string ssid = p(sk);
        std::string pw = p(pk);
        // pw >= 8 matches WPA2's own minimum.
        if (ssid.size() > 0 && pw.size() >= 8) {
            JsonObject n = nets.add<JsonObject>();
            n["ssid"] = ssid;
            n["password"] = pw;
        }
    }
}

static std::string trim_lower(std::string s) {
    size_t b = 0, e = s.size();
    while (b < e && isspace((unsigned char)s[b])) b++;
    while (e > b && isspace((unsigned char)s[e - 1])) e--;
    s = s.substr(b, e - b);
    for (char &c : s) c = (char)tolower((unsigned char)c);
    return s;
}

static esp_err_t wifi_save_handler(httpd_req_t *req) {
    FormArgs a;
    if (!a.load(req)) return send_text(req, "400 Bad Request", "text/plain", "Bad request body");
    auto p = [&](const char *k) { return a.get(k); };

    JsonDocument doc;
    parse_wifi_core_fields(a, doc);

    JsonObject mqtt_obj = doc["mqtt"].to<JsonObject>();
    mqtt_obj["enabled"] = (atoi(p("mqtt_enabled").c_str()) != 0);
    mqtt_obj["host"] = p("mqtt_host");
    int mqtt_port = atoi(p("mqtt_port").c_str());
    mqtt_obj["port"] = (mqtt_port > 0 && mqtt_port <= 65535) ? mqtt_port : 1883;
    mqtt_obj["user"] = p("mqtt_user");
    mqtt_obj["password"] = p("mqtt_pass");
    std::string mqtt_cid = p("mqtt_client_id");
    mqtt_obj["client_id"] = mqtt_cid.size() > 0 ? mqtt_cid : std::string("gen-ctl");
    std::string mqtt_pfx = p("mqtt_prefix");
    mqtt_obj["topic_prefix"] = mqtt_pfx.size() > 0 ? mqtt_pfx : std::string("gen-ctl");

    JsonObject limits_obj = doc["limits"].to<JsonObject>();
    limits_obj["temp_air_max"]      = atof(p("limit_temp_air").c_str());
    limits_obj["temp_oil_max"]      = atof(p("limit_temp_oil").c_str());
    limits_obj["temp_engine_max"]   = atof(p("limit_temp_engine").c_str());
    limits_obj["temp_inverter_max"] = atof(p("limit_temp_inverter").c_str());
    limits_obj["power_max"]         = atof(p("limit_power").c_str());
    limits_obj["auto_load_relay_on_run"] = (atoi(p("limit_auto_load_relay").c_str()) != 0);
    limits_obj["choke_cold_start_temp"]  = atof(p("limit_choke_cold_start").c_str());
    JsonObject maint_int_obj = limits_obj["maint_interval"].to<JsonObject>();
    for (uint8_t i = 0; i < FLOG_MAINT_TYPES; i++) {
        // "limit_maint_interval_fuel_filter" is exactly 32 chars; keep the
        // buffer well above that.
        char key[48];
        snprintf(key, sizeof(key), "limit_maint_interval_%s", fram_log_maint_name(i));
        maint_int_obj[fram_log_maint_name(i)] = atof(p(key).c_str());
    }

    JsonObject timing_obj = doc["timing"].to<JsonObject>();
    int max_try = atoi(p("timing_max_start_try").c_str());
    timing_obj["max_start_try"]            = (max_try >= 1) ? max_try : 5;
    timing_obj["start_time_check_ms"]      = atol(p("timing_start_time_check").c_str());
    timing_obj["start_time_ms"]            = atol(p("timing_start_time").c_str());
    timing_obj["start_retry_time_ms"]      = atol(p("timing_start_retry_time").c_str());
    timing_obj["stopping_time_ms"]         = atol(p("timing_stopping_time").c_str());
    timing_obj["running_time_ms"]          = atol(p("timing_running_time").c_str());
    timing_obj["load_relay_stop_delay_ms"] = atol(p("timing_load_relay_stop_delay").c_str());
    timing_obj["after_run_period_ms"]      = atol(p("timing_after_run_period").c_str());
    timing_obj["cooldown_enabled"]         = (atoi(p("timing_cooldown_enabled").c_str()) != 0);
    timing_obj["cooldown_temp_c"]          = atof(p("timing_cooldown_temp").c_str());
    timing_obj["cooldown_min_s"]           = atol(p("timing_cooldown_min").c_str());
    timing_obj["cooldown_max_s"]           = atol(p("timing_cooldown_max").c_str());

    JsonObject battery_obj = doc["battery"].to<JsonObject>();
    battery_obj["threshold_start_charge_v"] = atof(p("bat_threshold_start_charge").c_str());
    battery_obj["threshold_stop_charge_v"]  = atof(p("bat_threshold_stop_charge").c_str());
    battery_obj["threshold_starting_low_v"] = atof(p("bat_threshold_starting_low").c_str());
    battery_obj["threshold_shutdown_v"]     = atof(p("bat_threshold_shutdown").c_str());
    battery_obj["shutdown_persist_time_s"]  = atol(p("bat_shutdown_persist_time").c_str());
    battery_obj["start_charge_persist_time_s"] = atol(p("bat_start_charge_persist_time").c_str());
    battery_obj["read_period_ms"]           = atol(p("bat_read_period").c_str());
    battery_obj["read_starting_period_ms"]  = atol(p("bat_read_starting_period").c_str());
    battery_obj["read_running_period_ms"]   = atol(p("bat_read_running_period").c_str());
    battery_obj["read_after_run_period_ms"] = atol(p("bat_read_after_run_period").c_str());
    battery_obj["read_charging_period_ms"]  = atol(p("bat_read_charging_period").c_str());
    battery_obj["psu_loss_backup_enabled"]  = (atoi(p("bat_psu_loss_backup").c_str()) != 0);
    battery_obj["psu_autostart_enabled"]    = (atoi(p("bat_psu_autostart").c_str()) != 0);
    battery_obj["psu_autostart_delay_s"]    = atol(p("bat_psu_autostart_delay").c_str());
    battery_obj["psu_autostop_delay_s"]     = atol(p("bat_psu_autostop_delay").c_str());

    JsonObject engine_obj = doc["engine"].to<JsonObject>();
    engine_obj["rpm_max"]                = atol(p("engine_rpm_max").c_str());
    engine_obj["rpm_starting"]           = atol(p("engine_rpm_starting").c_str());
    engine_obj["rpm_running"]            = atol(p("engine_rpm_running").c_str());
    engine_obj["rpm_starting_threshold"] = atol(p("engine_rpm_starting_threshold").c_str());
    engine_obj["rpm_running_threshold"]  = atol(p("engine_rpm_running_threshold").c_str());
    engine_obj["fuel_pump_stop_level"]   = atof(p("engine_fuel_pump_stop_level").c_str());
    engine_obj["fuel_pump_no_rise_timeout_s"] = atol(p("engine_fuel_pump_no_rise_timeout").c_str());

    JsonObject time_obj = doc["time"].to<JsonObject>();
    time_obj["tz_offset_hours"]   = atoi(p("tz_offset_hours").c_str());
    time_obj["tz_offset_minutes"] = atoi(p("tz_offset_minutes").c_str());

    // A bad key would otherwise save fine and silently disable the link at
    // the next boot (espnow_config_load() fails closed) — refuse it here.
    bool espnow_enabled = atoi(p("espnow_enabled").c_str()) != 0;
    std::string espnow_key = trim_lower(p("espnow_key"));
    bool espnow_key_ok = espnow_key.size() == ESPNOW_KEY_LEN * 2;
    for (size_t i = 0; espnow_key_ok && i < espnow_key.size(); i++)
        espnow_key_ok = isxdigit((unsigned char)espnow_key[i]);
    if (espnow_enabled && !espnow_key_ok) {
        return send_text(req, "400 Bad Request", "text/plain",
                         "ESP-NOW key must be exactly 32 hex characters. Nothing was saved.");
    }
    JsonObject espnow_obj = doc["espnow"].to<JsonObject>();
    espnow_obj["enabled"] = espnow_enabled;
    espnow_obj["key"]     = espnow_key;

    if (!wifi_config_save(doc)) return send_text(req, "500 Internal Server Error", "text/plain", "Save failed");
    send_text(req, "200 OK", "text/plain", "OK. Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

// ---- file manager writes ----------------------------------------------------

static esp_err_t mkdir_handler(httpd_req_t *req) {
    FormArgs a;
    a.load(req);
    std::string path = a.has("path") ? a.get("path") : "/";
    if (path.empty() || path.back() != '/') path += "/";
    std::string dirName = a.get("name");
    if (path.find("..") != std::string::npos || dirName.find("..") != std::string::npos ||
        dirName.find('/') != std::string::npos) {
        return send_text(req, "403 Forbidden", "text/plain", "Forbidden");
    }
    std::string dirPath = path + dirName;
    std::string vfs = std::string(FS_MOUNT_POINT) + dirPath;
    bool held = take_flash_guard();
    bool ok = mkdir(vfs.c_str(), 0775) == 0;
    give_flash_guard(held);
    if (!ok) return send_text(req, "500 Internal Server Error", "text/plain", "Failed to create directory");
    return send_text(req, "200 OK", "text/plain", "OK");
}

static esp_err_t delete_handler(httpd_req_t *req) {
    FormArgs a;
    a.load(req);
    std::string path = a.get("path");
    if (path.find("..") != std::string::npos || path.empty()) {
        return send_text(req, "403 Forbidden", "text/plain", "Forbidden");
    }
    if (path[0] != '/') path = "/" + path;
    while (path.size() > 1 && path.back() == '/') path.pop_back();

    std::string vfs = std::string(FS_MOUNT_POINT) + path;
    bool held = take_flash_guard();
    struct stat st;
    if (stat(vfs.c_str(), &st) != 0) {
        give_flash_guard(held);
        return send_text(req, "404 Not Found", "text/plain", "File not found");
    }
    bool ok = S_ISDIR(st.st_mode) ? (rmdir(vfs.c_str()) == 0) : (unlink(vfs.c_str()) == 0);
    give_flash_guard(held);
    if (!ok) return send_text(req, "500 Internal Server Error", "text/plain", "Failed to delete");
    return send_text(req, "200 OK", "text/plain", "OK");
}

// ---- uploads ----------------------------------------------------------------


bool receive_multipart(httpd_req_t *req, UploadHandler &handler, bool &complete) {
    complete = false;
    char ctype[160];
    if (httpd_req_get_hdr_value_str(req, "Content-Type", ctype, sizeof(ctype)) != ESP_OK) ctype[0] = '\0';
    MultipartParser parser;
    bool parsing = parser.begin(ctype, &handler);
    static uint8_t buf[UPLOAD_RECV_CHUNK];  // one request at a time: httpd runs handlers sequentially
    size_t remaining = req->content_len;
    int timeouts = 0;
    while (remaining > 0) {
        int n = httpd_req_recv(req, (char *)buf, remaining < sizeof(buf) ? remaining : sizeof(buf));
        if (n == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts <= UPLOAD_RECV_TIMEOUT_RETRIES) continue;
        if (n <= 0) {
            if (parser.in_file()) handler.on_file_end_aborted();
            return false;
        }
        timeouts = 0;
        remaining -= (size_t)n;
        if (parsing && !parser.feed(buf, (size_t)n)) {
            parsing = false;
            if (parser.in_file()) handler.on_file_end_aborted();
        }
        wdt_reset_if_subscribed();
        vTaskDelay(1);
    }
    complete = parsing && parser.complete();
    if (parsing && !complete && parser.in_file()) handler.on_file_end_aborted();
    return true;
}

// /api/upload: writes the file part into ?path= (default "/").
class FileUpload : public UploadHandler {
public:
    explicit FileUpload(const std::string &dir) : dir_(dir) {}
    bool on_file_start(const std::string &field, const std::string &filename) override {
        (void)field;
        vfs_path_ = std::string(FS_MOUNT_POINT) + dir_ + filename;
        bool held = take_flash_guard();
        f_ = fopen(vfs_path_.c_str(), "wb");
        give_flash_guard(held);
        return true;  // like WebServer: a failed open just drops the data
    }
    bool on_file_data(const uint8_t *data, size_t len) override {
        if (f_ == nullptr) return true;
        bool held = take_flash_guard();
        fwrite(data, 1, len, f_);
        give_flash_guard(held);
        return true;
    }
    bool on_file_end() override {
        close();
        return true;
    }
    void on_file_end_aborted() override {
        if (f_ == nullptr) return;
        close();
        bool held = take_flash_guard();
        unlink(vfs_path_.c_str());
        give_flash_guard(held);
    }
    ~FileUpload() override { close(); }

private:
    void close() {
        if (f_ == nullptr) return;
        bool held = take_flash_guard();
        fclose(f_);
        give_flash_guard(held);
        f_ = nullptr;
    }
    std::string dir_;
    std::string vfs_path_;
    FILE *f_ = nullptr;
};

static esp_err_t upload_handler(httpd_req_t *req) {
    char arg[256];
    std::string dir = query_arg(req, "path", arg, sizeof(arg)) ? arg : "/";
    if (dir.empty() || (dir.back() != '/' && dir != "/")) dir += "/";
    if (dir.find("..") != std::string::npos) dir = "/";
    if (dir[0] != '/') dir = "/" + dir;

    FileUpload upload(dir);
    bool complete = false;
    if (!receive_multipart(req, upload, complete) || !complete) return ESP_FAIL;
    return send_text(req, "200 OK", "text/plain", "OK");
}

// /update: OTA image into the next app slot.
class OtaUpload : public UploadHandler {
public:
    bool on_file_start(const std::string &field, const std::string &filename) override {
        (void)field;
        (void)filename;
        part_ = esp_ota_get_next_update_partition(NULL);
        bool held = take_flash_guard();
        // esp_ota_begin() refuses while the running image is still in its
        // post-OTA soak (PENDING_VERIFY), and the reboot after a failed OTA
        // would then roll that image back. An image that is serving an OTA
        // request is healthy enough to become the rollback target, so
        // confirm it first, as a second OTA in a row always worked before.
        esp_ota_img_states_t running_state = ESP_OTA_IMG_UNDEFINED;
        if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &running_state) == ESP_OK &&
            running_state == ESP_OTA_IMG_PENDING_VERIFY) {
            SLOG_PRINTLN("OTA: confirming the running image before writing the next one");
            esp_ota_mark_app_valid_cancel_rollback();
        }
        // Sequential writes erase as they go instead of wiping the whole slot
        // up front, which would stall the request for tens of seconds.
        started_ = part_ != nullptr && esp_ota_begin(part_, OTA_WITH_SEQUENTIAL_WRITES, &handle_) == ESP_OK;
        give_flash_guard(held);
        if (!started_) error_ = true;
        return true;
    }
    bool on_file_data(const uint8_t *data, size_t len) override {
        if (!started_ || error_) return true;
        bool held = take_flash_guard();
        if (esp_ota_write(handle_, data, len) != ESP_OK) error_ = true;
        give_flash_guard(held);
        size_ += len;
        return true;
    }
    bool on_file_end() override {
        if (!started_) return true;
        started_ = false;
        bool held = take_flash_guard();
        bool ok = !error_ && esp_ota_end(handle_) == ESP_OK && esp_ota_set_boot_partition(part_) == ESP_OK;
        if (error_) esp_ota_abort(handle_);
        give_flash_guard(held);
        if (!ok) {
            error_ = true;
            return true;
        }
        FramLogRecord r = {};
        r.type = FLOG_EV_FW_UPDATE;
        r.d.fw_update.size = size_;
        esp_app_desc_t desc = {};
        if (esp_ota_get_partition_description(part_, &desc) == ESP_OK) {
            strlcpy(r.d.fw_update.version, desc.version, sizeof(r.d.fw_update.version));
        }
        fram_log_event(&r);
        return true;
    }
    void on_file_end_aborted() override {
        if (!started_) return;
        started_ = false;
        esp_ota_abort(handle_);
        error_ = true;
    }
    bool has_error() const { return error_; }

private:
    const esp_partition_t *part_ = nullptr;
    esp_ota_handle_t handle_ = 0;
    bool started_ = false;
    bool error_ = false;
    uint32_t size_ = 0;
};

static esp_err_t ota_handler(httpd_req_t *req) {
    OtaUpload ota;
    bool complete = false;
    if (!receive_multipart(req, ota, complete) || !complete) return ESP_FAIL;
    send_text(req, "200 OK", "text/plain", ota.has_error() ? "FAIL" : "OK");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
    return ESP_OK;
}

// ---- WebSocket ---------------------------------------------------------------
//
// Client table, heartbeat and broadcast all run in the httpd task (the /ws
// handler, queued work and the session close callback), so the table needs
// no lock. Other tasks only read the client count.

struct WsClient {
    int fd;
    uint32_t last_ping_ms;
    bool awaiting_pong;
    uint8_t missed_pongs;
};

static httpd_handle_t s_server = nullptr;
static WsClient s_ws[WS_MAX_CLIENTS];
static volatile int s_ws_count = 0;
static char s_broadcast_buf[WS_BROADCAST_MAX_LEN];
static size_t s_broadcast_len = 0;
static volatile bool s_broadcast_busy = false;

static void ws_table_init(void) {
    for (WsClient &c : s_ws) c.fd = -1;
}

// False if all slots are taken.
static bool ws_add(int fd) {
    for (WsClient &c : s_ws) {
        if (c.fd == fd) return true;
    }
    for (WsClient &c : s_ws) {
        if (c.fd < 0) {
            c = {fd, 0, false, 0};  // last_ping 0: first ping goes out on the next tick
            s_ws_count = s_ws_count + 1;
            return true;
        }
    }
    return false;
}

static void ws_remove(int fd) {
    for (WsClient &c : s_ws) {
        if (c.fd == fd) {
            c.fd = -1;
            s_ws_count = s_ws_count - 1;
            SLOG_PRINTF("WebSocket client fd=%d disconnected - free=%u largest8bit=%u\n", fd,
                        heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
            return;
        }
    }
}

static WsClient *ws_find(int fd) {
    for (WsClient &c : s_ws) {
        if (c.fd == fd) return &c;
    }
    return nullptr;
}

// Session close hook: keeps the client table in step with the sockets.
static void on_session_close(httpd_handle_t hd, int fd) {
    (void)hd;
    ws_remove(fd);
    close(fd);
}

static bool ws_parse_reg(const char *s, uint8_t *reg_out) {
    if (!s || s[0] < '0' || s[0] > '9') return false;
    char *end = nullptr;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0') return false;
    if (v < 0 || v > 15) return false;
    *reg_out = (uint8_t)v;
    return true;
}

// Same text command set as the HTTP /start, /stop and /reg routes.
static void ws_handle_text(const char *buf) {
    if (xWebCommandQueue == NULL) return;
    WebCommand_t cmd = {};
    cmd.source = FLOG_SRC_WEBSOCKET;
    uint8_t reg = 0;
    if (strcmp(buf, "start") == 0) {
        cmd.type = WEB_CMD_START;
    } else if (strcmp(buf, "start_retry") == 0) {
        cmd.type = WEB_CMD_START_RETRY;
    } else if (strcmp(buf, "stop") == 0) {
        cmd.type = WEB_CMD_STOP;
    } else if (ws_parse_reg(buf, &reg)) {
        cmd.type = WEB_CMD_REG_TOGGLE;
        cmd.reg = reg;
    } else {
        return;
    }
    xQueueSend(xWebCommandQueue, &cmd, 0);
}

static esp_err_t ws_post_handshake(httpd_req_t *req) {
    int fd = httpd_req_to_sockfd(req);
    if (!ws_add(fd)) {
        SLOG_PRINTF("WebSocket client fd=%d refused: all %d slots in use\n", fd, WS_MAX_CLIENTS);
        httpd_sess_trigger_close(req->handle, fd);
        return ESP_OK;
    }
    // httpd's sockets are IPv6, so an IPv4 client shows up as ::ffff:a.b.c.d.
    struct sockaddr_storage peer = {};
    socklen_t peer_len = sizeof(peer);
    char ip[16] = "?";
    if (getpeername(fd, (struct sockaddr *)&peer, &peer_len) == 0) {
        const uint8_t *a = nullptr;
        if (peer.ss_family == AF_INET6) {
            a = ((struct sockaddr_in6 *)&peer)->sin6_addr.s6_addr + 12;
        } else if (peer.ss_family == AF_INET) {
            a = (const uint8_t *)&((struct sockaddr_in *)&peer)->sin_addr.s_addr;
        }
        if (a) snprintf(ip, sizeof(ip), "%u.%u.%u.%u", a[0], a[1], a[2], a[3]);
    }
    SLOG_PRINTF("WebSocket client fd=%d connected from %s - free=%u largest8bit=%u\n", fd, ip,
                heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    return ESP_OK;
}

static esp_err_t ws_handler(httpd_req_t *req) {
    int fd = httpd_req_to_sockfd(req);
    httpd_ws_frame_t frame = {};
    if (httpd_ws_recv_frame(req, &frame, 0) != ESP_OK) return ESP_FAIL;
    uint8_t buf[WS_RX_MAX_LEN];
    if (frame.len > sizeof(buf)) {
        // Larger than any command or control frame we expect; drain and drop.
        return ESP_FAIL;
    }
    frame.payload = buf;
    if (frame.len > 0 && httpd_ws_recv_frame(req, &frame, frame.len) != ESP_OK) return ESP_FAIL;

    switch (frame.type) {
        case HTTPD_WS_TYPE_TEXT: {
            char text[32];
            size_t n = frame.len < sizeof(text) - 1 ? frame.len : sizeof(text) - 1;
            memcpy(text, buf, n);
            text[n] = '\0';
            ws_handle_text(text);
            break;
        }
        case HTTPD_WS_TYPE_PONG: {
            WsClient *c = ws_find(fd);
            if (c) {
                c->awaiting_pong = false;
                c->missed_pongs = 0;
            }
            break;
        }
        case HTTPD_WS_TYPE_PING: {
            httpd_ws_frame_t pong = {};
            pong.type = HTTPD_WS_TYPE_PONG;
            pong.final = true;
            pong.payload = buf;
            pong.len = frame.len;
            httpd_ws_send_frame(req, &pong);
            break;
        }
        case HTTPD_WS_TYPE_CLOSE: {
            httpd_ws_frame_t reply = {};
            reply.type = HTTPD_WS_TYPE_CLOSE;
            reply.final = true;
            httpd_ws_send_frame(req, &reply);
            httpd_sess_trigger_close(req->handle, fd);
            break;
        }
        default:
            break;
    }
    return ESP_OK;
}

static void ws_send_or_drop(int fd, httpd_ws_frame_t *frame) {
    if (httpd_ws_send_frame_async(s_server, fd, frame) != ESP_OK) {
        httpd_sess_trigger_close(s_server, fd);
    }
}

static void ws_broadcast_work(void *arg) {
    (void)arg;
    httpd_ws_frame_t frame = {};
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.final = true;
    frame.payload = (uint8_t *)s_broadcast_buf;
    frame.len = s_broadcast_len;
    for (WsClient &c : s_ws) {
        if (c.fd >= 0) ws_send_or_drop(c.fd, &frame);
    }
    s_broadcast_busy = false;
}

// Ping every WS_PING_INTERVAL_MS; a missed pong is retried after
// WS_PONG_TIMEOUT_MS, and the client is dropped after
// WS_MISSED_PONGS_TO_DROP misses in a row (10 s of silence).
static void ws_heartbeat_work(void *arg) {
    (void)arg;
    uint32_t now = uptime_ms();
    for (WsClient &c : s_ws) {
        if (c.fd < 0) continue;
        if (c.awaiting_pong) {
            if (now - c.last_ping_ms < WS_PONG_TIMEOUT_MS) continue;
            if (++c.missed_pongs >= WS_MISSED_PONGS_TO_DROP) {
                httpd_sess_trigger_close(s_server, c.fd);
                continue;
            }
        } else if (c.last_ping_ms != 0 && now - c.last_ping_ms < WS_PING_INTERVAL_MS) {
            continue;
        }
        httpd_ws_frame_t ping = {};
        ping.type = HTTPD_WS_TYPE_PING;
        ping.final = true;
        c.last_ping_ms = now ? now : 1;
        c.awaiting_pong = true;
        ws_send_or_drop(c.fd, &ping);
    }
}

int http_ws_client_count(void) {
    return s_ws_count;
}

bool http_ws_broadcast(const char *data, size_t len) {
    if (s_server == nullptr || s_ws_count == 0) return true;
    if (len > sizeof(s_broadcast_buf) || s_broadcast_busy) return false;
    memcpy(s_broadcast_buf, data, len);
    s_broadcast_len = len;
    s_broadcast_busy = true;
    if (httpd_queue_work(s_server, ws_broadcast_work, nullptr) != ESP_OK) {
        s_broadcast_busy = false;
        return false;
    }
    return true;
}

void http_ws_tick(void) {
    if (s_server == nullptr || s_ws_count == 0) return;
    httpd_queue_work(s_server, ws_heartbeat_work, nullptr);
}

// ---- server -----------------------------------------------------------------

static esp_err_t not_found_handler(httpd_req_t *req, httpd_err_code_t err) {
    (void)err;
    send_text(req, "404 Not Found", "text/plain", "Not found");
    return ESP_OK;
}

static void register_route(httpd_handle_t server, httpd_method_t method, const char *uri,
                           esp_err_t (*handler)(httpd_req_t *), void *ctx = nullptr) {
    httpd_uri_t route = {};
    route.uri = uri;
    route.method = method;
    route.handler = handler;
    route.user_ctx = ctx;
    httpd_register_uri_handler(server, &route);
}

static void register_get(httpd_handle_t server, const char *uri, esp_err_t (*handler)(httpd_req_t *),
                         void *ctx = nullptr) {
    register_route(server, HTTP_GET, uri, handler, ctx);
}

static httpd_handle_t start_httpd(uint16_t port) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = port;
    cfg.ctrl_port = (uint16_t)(ESP_HTTPD_DEF_CTRL_PORT + port);
    cfg.max_open_sockets = HTTP_MAX_OPEN_SOCKETS;
    cfg.max_uri_handlers = HTTP_MAX_URI_HANDLERS;
    cfg.stack_size = HTTP_TASK_STACK;
    cfg.task_priority = HTTP_TASK_PRIORITY;
    cfg.core_id = HTTP_TASK_CORE;
    cfg.lru_purge_enable = true;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.send_wait_timeout = HTTP_SEND_WAIT_TIMEOUT_S;
    cfg.close_fn = on_session_close;
    ws_table_init();

    httpd_handle_t server = nullptr;
    if (httpd_start(&server, &cfg) != ESP_OK) return nullptr;
    httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, not_found_handler);
    return server;
}

bool http_server_start(void) {
    httpd_handle_t server = start_httpd(HTTP_SERVER_PORT);
    if (server == nullptr) return false;
    s_server = server;

    httpd_uri_t ws = {};
    ws.uri = "/ws";
    ws.method = HTTP_GET;
    ws.handler = ws_handler;
    ws.is_websocket = true;
    ws.handle_ws_control_frames = true;
    // The dashboard opens the socket with subprotocol "arduino" (what the old
    // WebSocketsServer answered); browsers fail the handshake unless it is
    // echoed back.
    ws.supported_subprotocol = "arduino";
    ws.ws_post_handshake_cb = ws_post_handshake;
    httpd_register_uri_handler(server, &ws);

    for (const StaticPage &page : kPages) {
        register_get(server, page.uri, page_handler, (void *)&page);
    }
    register_get(server, "/favicon.ico", favicon_handler);
    register_get(server, "/static/*", static_handler);
    register_get(server, "/metrics", metrics_handler);
    register_get(server, "/log", log_handler);
    register_get(server, "/coredump", coredump_handler);
    register_get(server, "/api/fw/partitions", fw_partitions_handler);
    register_get(server, "/api/wifi/config", wifi_config_handler);
    register_get(server, "/api/files", files_handler);
    register_get(server, "/start", start_handler);
    register_get(server, "/start_retry", start_retry_handler);
    register_get(server, "/stop", stop_handler);
    register_get(server, "/reg", reg_handler);
    register_get(server, "/log/wipe", log_wipe_handler);
    register_get(server, "/energy/reset", energy_reset_handler);
    register_get(server, "/metrics/set", metrics_set_handler);
    register_get(server, "/maintenance", maintenance_handler);
    register_get(server, "/reset", reset_handler);
    register_get(server, "/api/fw/switch-slot", fw_switch_slot_handler);
    register_route(server, HTTP_POST, "/api/wifi/save", wifi_save_handler);
    register_route(server, HTTP_POST, "/api/mkdir", mkdir_handler);
    register_route(server, HTTP_DELETE, "/api/delete", delete_handler);
    register_route(server, HTTP_POST, "/api/upload", upload_handler);
    register_route(server, HTTP_POST, "/update", ota_handler);
#if BOOTLOADER_UPDATE_ENDPOINT_ENABLED
    // Temporary, single-use capability — see bootloader_update.h. Off (0)
    // in every normal build.
    register_route(server, HTTP_POST, "/api/bootloader/update", bootloader_update_handler);
#endif
    return true;
}

// ---- update mode (JP1 jumper, before setup_rtos()) ----------------------------

static esp_err_t update_mode_wifi_config_handler(httpd_req_t *req) {
    return serve_static(req, "/wifi-config.html");
}

// Recovery mode only rewrites the WiFi part of wifi.json.
static esp_err_t update_mode_wifi_save_handler(httpd_req_t *req) {
    FormArgs a;
    if (!a.load(req)) return send_text(req, "400 Bad Request", "text/plain", "Bad request body");
    JsonDocument doc;
    parse_wifi_core_fields(a, doc);
    if (!wifi_config_save(doc)) return send_text(req, "500 Internal Server Error", "text/plain", "Save failed");
    send_text(req, "200 OK", "text/plain", "OK. Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

static esp_err_t update_mode_ota_handler(httpd_req_t *req) {
    OtaUpload ota;
    bool complete = false;
    if (!receive_multipart(req, ota, complete) || !complete) return ESP_FAIL;
    send_text(req, "200 OK", "text/plain", ota.has_error() ? "FAIL" : "OK");
    REG_ON(REG_BUZZER);
    vTaskDelay(pdMS_TO_TICKS(100));
    REG_OFF(REG_BUZZER);
    vTaskDelay(pdMS_TO_TICKS(50));
    REG_ON(REG_BUZZER);
    vTaskDelay(pdMS_TO_TICKS(100));
    REG_OFF(REG_BUZZER);
    oled.println("Updated");
    esp_restart();
    return ESP_OK;
}

bool http_server_start_update_mode(void) {
    httpd_handle_t server = start_httpd(HTTP_SERVER_PORT);
    if (server == nullptr) return false;
    register_get(server, "/reset", reset_handler);
    register_get(server, "/wifi-config", update_mode_wifi_config_handler);
    register_route(server, HTTP_POST, "/api/wifi/save", update_mode_wifi_save_handler);
    register_route(server, HTTP_POST, "/update", update_mode_ota_handler);
    return true;
}
