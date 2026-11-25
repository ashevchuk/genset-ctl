/*
 * i2c.cpp — I2CTask, sole owner of the I2C bus, plus the sensor-read/RTC-service
 * work and the on-demand job-queue primitive that ride its tick.
 *
 * Extracted 2026-09-24 from esp32-gen-ctl-ai-rtos.cpp (see CLAUDE.md's "Code
 * organization" section and [[main-cpp-modularization]] project memory for
 * the pattern this follows). Moved verbatim.
 */

#include "i2c.h"
#include "rtos_config.h"
#include "fram_log.h"
#include "shiftreg.h"
#include "oled_menu.h"
#include "oled_dev.h"
#include "smoothed_safe.h"
#include "debug.h"
#include "rtc_dev.h"
#include "ina233_dev.h"
#include "mcp9600_dev.h"
#include "tacho.h"
#include "ads1115_dev.h"
#include "config.h"
#include <esp_task_wdt.h>
#include <sys/time.h>
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "uptime.h"
#include <math.h>

extern OledDev oled;
extern DateTime now;
extern DateTime start;

// Written here (sync_time_cb(), SNTP task context), read by the main
// .cpp's own loop() — a plain non-static handoff pair rather than a
// promoted shared header, since loop() is the only outside reader.
volatile bool ntp_sync_flag = false;
volatile time_t ntp_sync_time = 0;

#define CONFIG_SNTP_TIME_SERVER "time.windows.com"

static const char *ntpServer1 = "pool.ntp.org";
static const char *ntpServer2 = "time.nist.gov";

static void i2c_read_sensors(void);
static void i2c_service_rtc(void);

// ---- externs into esp32-gen-ctl-ai-rtos.cpp's own globals ------------------
extern bool ina233_present;   // set in setup(), avoids repeated NACK if sensor absent
extern bool mcp9600_present;  // set in setup(), avoids repeated NACK if sensor absent
extern SmoothedSafe<double> adc_1_0;
extern SmoothedSafe<double> adc_1_1;
extern SmoothedSafe<double> adc_1_2;
extern SmoothedSafe<double> adc_1_3;
extern SmoothedSafe<double> adc_2_0;
extern SmoothedSafe<double> adc_2_1;
extern SmoothedSafe<double> adc_2_2;
double readChannel1(ADS1115_MUX channel);
double readChannel2(ADS1115_MUX channel);
double adc_temp(const double adc_voltage);
uint32_t tachoRPM(void);
float tachoFREQ(void);

// Edge-triggered FRAM logging for a sensor read failure — logs once when a
// sensor transitions from OK to failing (same latch idea as loop()'s
// i2cStuckLogged), not on every single failed read, so a device that stays
// down for a while doesn't flood the 4080-record ring. *failing_flag is a
// caller-owned static bool that persists across SensorTask iterations.
static void sensor_log_fail_once(uint8_t sensor_id, bool *failing_flag) {
    if (*failing_flag) return;
    *failing_flag = true;
    FramLogRecord r = {};
    r.type = FLOG_EV_SENSOR_FAIL;
    r.d.sensor_fail.sensor = sensor_id;
    fram_log_event(&r);
}

// Blocks the calling task until I2CTask has run `fn(arg)` on its own
// context. Returns true iff the job actually ran to completion before
// `timeout` — NOT whether `fn`'s own operation succeeded; `fn` must report
// that itself (into `arg`'s own static storage, or another static owned by
// the call site), checked by the caller after this returns true. `I2CJob_t`
// crosses the queue by value, so there is no way to carry a result back
// through the job struct itself — see the comment on `I2CJob_t` in
// rtos_config.h. Each calling task should reuse its own static semaphore
// across calls (calls from a given task are already sequential, so one
// instance is enough). Not static: called from fram_log.cpp too (see
// rtos_config.h's prototype).
bool i2c_run_on_owner(bool (*fn)(void *), void *arg, SemaphoreHandle_t done_sem, TickType_t timeout) {
    I2CJob_t job = { fn, arg, done_sem };
    xSemaphoreTake(done_sem, 0);  // drain any stale signal before submitting
    if (xI2CJobQueue == NULL) return false;

    // Wait in short slices (not one long xQueueSend / xSemaphoreTake) so the
    // calling task — WebTask during /log/wipe, StateMachineTask during an
    // on-demand INA233 read, etc. — can keep petting its own TWDT. The job
    // body may reset I2CTask's WDT (e.g. wipe_log_area), which does nothing
    // for the waiter. Same 2×timeout budget as before: up to `timeout` to
    // enqueue, then up to `timeout` again for completion.
    const TickType_t slice = pdMS_TO_TICKS(200);
    TickType_t start = xTaskGetTickCount();
    for (;;) {
        TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= timeout) return false;
        TickType_t remaining = timeout - elapsed;
        TickType_t wait = (remaining < slice) ? remaining : slice;
        if (xQueueSend(xI2CJobQueue, &job, wait) == pdTRUE) break;
        esp_task_wdt_reset();
    }

    start = xTaskGetTickCount();
    for (;;) {
        TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= timeout) return xSemaphoreTake(done_sem, 0) == pdTRUE;
        TickType_t remaining = timeout - elapsed;
        TickType_t wait = (remaining < slice) ? remaining : slice;
        if (xSemaphoreTake(done_sem, wait) == pdTRUE) return true;
        esp_task_wdt_reset();
    }
}

void I2CTask(void *pvParameters) {
    (void)pvParameters;
    esp_task_wdt_add(NULL);  // self-register before first wdt_reset
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t tick = 0;

    SLOG_PRINTLN("I2CTask started");

    for (;;) {
        // xFlashI2CGuard: not logical I2C arbitration (this task is the sole
        // owner of the I2C bus) — guards the spi_flash_guard_start()/I2C collision
        // hazard against WebTask's concurrent FFat reads. Best-effort, same
        // 600ms-timeout pattern as WebTask's side: proceed even if not held,
        // matching how the rest of this file already treats this guard.
        bool i2c_held = (xSemaphoreTake(xFlashI2CGuard, pdMS_TO_TICKS(600)) == pdTRUE);

        I2CJob_t job;
        unsigned jobs = 0;
        while (xI2CJobQueue != NULL && xQueueReceive(xI2CJobQueue, &job, 0) == pdTRUE) {
            job.fn(job.arg);  // job itself records success/failure — see i2c_run_on_owner()
            xSemaphoreGive(job.done);
            // A burst of short jobs (many fram_log_read from /log) still runs
            // entirely before this task's end-of-tick reset — pet periodically.
            if ((++jobs & 7) == 0) esp_task_wdt_reset();
        }

        fram_log_service_tick();  // drain xLogQueue, flush metrics — cheap when idle

        tick++;
        if (tick % 4  == 0) i2c_render_display();  // ~200ms, matches old DisplayTask render_tick
        // ~1000ms — slowed from the old 500ms (PERIOD_SENSOR_MS): nothing
        // reading this cache needs sub-second freshness (dashboard/WS report
        // is itself ~1Hz; oil/air/inverter/head temps don't move meaningfully
        // in half a second). Also shrinks the worst-case duty cycle a truly
        // wedged bus could spend retrying here every tick (see
        // i2c_read_sensors() — it doesn't bail early on a failed channel).
        if (tick % 20 == 0) i2c_read_sensors();
        if (tick % 20 == 0) i2c_service_rtc();     // ~1000ms, matches old loop() cadence

        if (i2c_held) xSemaphoreGive(xFlashI2CGuard);

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PERIOD_I2C_MS));
    }
}

// ----------------------------------------------------------------------------
// i2c_read_sensors(): read all I2C sensors
// ----------------------------------------------------------------------------
// Called once per ~500ms from I2CTask (its sole owner). Body is what used
// to be SensorTask's own — no mutex needed any more (I2CTask is the only I2C
// caller left), and the dead xSensorDataQueue push is dropped (it was
// written every cycle but never read anywhere).
static void i2c_read_sensors(void) {
    SensorData_t sensor_data;
    memset(&sensor_data, 0, sizeof(sensor_data));
    sensor_data.timestamp = uptime_ms();

    double voltage;

    // Read ADS1115 #1 (temperature sensors)
    voltage = readChannel1(ADS1115_COMP_0_GND);
    if (voltage > 0.0) sensor_data.adc1_ch0 = adc_temp(voltage);

    voltage = readChannel1(ADS1115_COMP_1_GND);
    if (voltage > 0.0) sensor_data.adc1_ch1 = adc_temp(voltage);

    voltage = readChannel1(ADS1115_COMP_2_GND);
    if (voltage > 0.0) sensor_data.adc1_ch2 = adc_temp(voltage);

    voltage = readChannel1(ADS1115_COMP_3_GND);
    if (voltage > 0.0) sensor_data.adc1_ch3 = adc_temp(voltage);

    // Read ADS1115 #2 (fuel/voltage sensors)
    voltage = readChannel2(ADS1115_COMP_0_GND);
    if (voltage > 0.0) sensor_data.adc2_ch0 = voltage;

    voltage = readChannel2(ADS1115_COMP_3_GND);
    if (voltage > 0.0) sensor_data.adc2_ch2 = voltage;

    // Read INA233 battery voltage only when relay is closed (BAT_SENSE=1).
    // When relay is open the bus side is floating — INA233 always returns 0,
    // which we would discard anyway and it floods the log with warnings.
    if (ina233_present && IS_REG_IN_ON(REG_IN_BAT_SENSE)) {
        static bool ina233_failing = false;
        if (IC1.getBusVoltage_V(sensor_data.bat_voltage)) {
            ina233_failing = false;
        } else {
            sensor_log_fail_once(FLOG_SENSOR_INA233, &ina233_failing);
        }
    }

    // Read MCP9600 thermocouple temperature (skip entirely if absent).
    // No retry loop here any more (see mcp9600_dev.cpp) — a failed
    // read just logs once per episode and leaves head_temp_valid
    // false, so callers (e.g. flog_head_temp_or_warm()) fall back
    // instead of trusting a stale/zero cached value.
    if (mcp9600_present && tempSensor.available()) {
        static bool mcp9600_failing = false;
        if (tempSensor.getThermocoupleTemp(sensor_data.head_temp)) {
            sensor_data.head_temp_valid = true;
            mcp9600_failing = false;
        } else {
            sensor_log_fail_once(FLOG_SENSOR_MCP9600, &mcp9600_failing);
        }
    }

    sensor_data.valid = true;

    // Update PCNT-based tachometer (compute RPM/Hz from pulse count delta)
    tacho_update();
    sensor_data.rpm = tachoRPM();
    sensor_data.frequency = tachoFREQ();

    // Track lifetime peaks (RAM only, flushed to FRAM by fram_log_service_tick())
    fram_metrics_note_sensors(sensor_data.head_temp, (float)sensor_data.rpm);

    // Update smoothed values (thread-safe since we're the only writer)
    adc_1_0.add(sensor_data.adc1_ch0);
    adc_1_1.add(sensor_data.adc1_ch1);
    adc_1_2.add(sensor_data.adc1_ch2);
    adc_1_3.add(sensor_data.adc1_ch3);
    adc_2_0.add(sensor_data.adc2_ch0);
    adc_2_2.add(sensor_data.adc2_ch2);

    if (sensor_data.bat_voltage > 0.0) {
        adc_2_1.add(sensor_data.bat_voltage);
        // Log bat voltage every 60 s to avoid flooding the debug output
        static uint32_t last_bat_log_ms = 0;
        static double last_bat_logged = 0.0;
        double smoothed = adc_2_1.get();
        uint32_t now_ms = uptime_ms();
        if (now_ms - last_bat_log_ms >= 60000 ||
            fabs(smoothed - last_bat_logged) >= 0.1) {
            SDEBUG_CHARGING("[BAT] i2c_read_sensors: INA233 raw=", dbg_fixed(sensor_data.bat_voltage, 3),
                   "V smoothed=", dbg_fixed(smoothed, 3),
                   "V charger=", IS_REG_ON(REG_CHARGER) ? "ON" : "OFF");
            last_bat_log_ms = now_ms;
            last_bat_logged = smoothed;
        }
    } else if (ina233_present && IS_REG_IN_ON(REG_IN_BAT_SENSE)) {
        // BAT_SENSE=ON but INA233 returned 0 — unexpected, relay is closed but no voltage.
        static uint32_t last_zero_log_ms = 0;
        uint32_t now_ms = uptime_ms();
        if (now_ms - last_zero_log_ms >= 10000) {  // log every 10 s
            SDEBUG_CHARGING("[BAT] i2c_read_sensors: INA233=0 but BAT_SENSE=ON — unexpected! "
                   " BATT_FORCE=", IS_REG_ON(REG_BATT_FORCE),
                   " charger=", IS_REG_ON(REG_CHARGER) ? "ON" : "OFF",
                   " — adc_2_1 stale=", dbg_fixed(adc_2_1.get(), 3), "V");
            last_zero_log_ms = now_ms;
        }
    }

    // Copy to shared state
    portENTER_CRITICAL(&g_state_spinlock);
    g_state.sensors = sensor_data;
    portEXIT_CRITICAL(&g_state_spinlock);

    // Notify state machine
    if (xStateEvents != NULL) {
        xEventGroupSetBits(xStateEvents, EVENT_SENSOR_UPDATE);
    }
}

// `time(NULL)` — ESP32's own system clock — is already continuously
// accurate on its own: seeded from the RTC once at boot (setup(), before
// this even exists), then kept fresh by SNTP's own periodic resync
// whenever a network is available. Nothing in this codebase reads any
// calendar field off the RTC-derived `now` DateTime (only ever
// `.unixtime()`), so there's no reason to keep re-reading the physical
// chip every second just to serve this one field — that was pure
// redundant I2C traffic. Kept as a wrapper (rather than switching call
// sites to `time(NULL)` directly) purely so a future accuracy need has one
// obvious place to look.
uint32_t rtc_now_unixtime(void) {
    return (uint32_t)time(NULL);
}

// Re-anchors the ESP32 system clock (time(NULL)) from the DS3231 every
// RTC_RESYNC_PERIOD_MS as a fallback against long-term drift on ESP32's
// own internal oscillator if NTP/internet is unavailable for an extended
// stretch — the DS3231 is temperature-compensated (~2ppm, roughly ±1
// minute/year), far more accurate long-term on its own than ESP32's
// internal clock left uncorrected. Harmless when NTP *is* available too:
// job_rtc_adjust() below already writes every NTP correction back to the
// DS3231, so in that case this just re-confirms a value SNTP already set
// (at most a couple of seconds of DS3231 drift since the last correction —
// same order of magnitude already tolerated by the ±2s threshold on
// FLOG_EV_TIME_SYNC, not a new class of jump).
#define RTC_RESYNC_PERIOD_MS (6UL * 3600UL * 1000UL)  // 6 hours
static void i2c_service_rtc(void) {
    static uint32_t last_resync_ms = 0;
    if (uptime_ms() - last_resync_ms < RTC_RESYNC_PERIOD_MS) return;
    last_resync_ms = uptime_ms();

    DateTime fresh;
    if (!rtc.now(fresh)) return;
    struct timeval tv = { .tv_sec = (time_t)fresh.unixtime(), .tv_usec = 0 };
    settimeofday(&tv, NULL);
}

// On-demand RTC write for the NTP-triggered adjustment in the main .cpp's
// loop() — deciding whether a sync is worth journaling is ordinary
// housekeeping logic, not I2C-owner code, so it stays there; only the
// actual hardware write goes through I2CTask, the same i2c_run_on_owner()
// pattern already used for FRAM admin ops and the on-demand INA233 read.
static bool job_rtc_adjust(void *arg) {
    return rtc.adjust(*(DateTime *)arg);
}
bool rtc_adjust_now(time_t unix_time) {
    static SemaphoreHandle_t s_done = NULL;
    if (s_done == NULL) s_done = xSemaphoreCreateBinary();
    // Static, not a local: if i2c_run_on_owner() times out (e.g. this job
    // queued behind a long one like a FRAM wipe), the caller may already
    // have moved on by the time I2CTask actually runs the job — a stack
    // local would then be a dangling pointer by then. Static storage stays
    // valid for I2CTask to read whenever it actually gets to it; the only
    // cost is that two rapid, overlapping calls could race on this shared
    // buffer, which NTP syncs are far too infrequent to ever trigger.
    static DateTime s_dt;
    s_dt = DateTime(unix_time);
    return i2c_run_on_owner(job_rtc_adjust, &s_dt, s_done, pdMS_TO_TICKS(200));
}

// Callback function (get's called when time adjusts via NTP)
// WARNING: This is called from SNTP task context - keep it minimal!
// Do NOT use blocking operations, I2C access, or delay() here!
static void sync_time_cb(struct timeval *t) {
  if (t != nullptr) {
    // Just set flag and time - actual RTC update will be done in main loop
    ntp_sync_time = t->tv_sec;
    ntp_sync_flag = true;

    // System time is already set by SNTP, no need to call settimeofday()
    // RTC update will be done safely in main loop to avoid I2C conflicts
  }
  // else: ignore empty sync events
}

void setup_ntp() {
  // WiFi API Arduino already initializes esp_netif and event loop
  // Only initialize if they haven't been initialized yet (check for
  // ESP_ERR_INVALID_STATE)
  esp_err_t ret = esp_netif_init();
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(ret);
  }

  ret = esp_event_loop_create_default();
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(ret);
  }

  // Configure SNTP with primary NTP server
  esp_sntp_config_t config =
      ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_SNTP_TIME_SERVER);

  config.start = false; // start SNTP service explicitly (after connecting)
  config.server_from_dhcp = true; // accept NTP offers from DHCP server, if any
                                  // (need to enable *before* connecting)
  config.renew_servers_after_new_IP =
      true; // let esp-netif update configured SNTP server(s) after receiving
            // DHCP lease
  config.index_of_first_server =
      1; // updates from server num 1, leaving server 0 (from DHCP) intact
  // configure the event on which we renew servers
  config.ip_event_to_renew = IP_EVENT_STA_GOT_IP;
  config.sync_cb = sync_time_cb; // callback function when time is synchronized

  ESP_ERROR_CHECK(esp_netif_sntp_init(&config));

  // Add additional NTP servers manually
  esp_sntp_setservername(1, ntpServer1);
  esp_sntp_setservername(2, ntpServer2);

  // Apply the compiled-in default now (covers the config-portal path, which
  // never reaches time_config_load() below); time_config_load() re-applies
  // it after wifi.json is read, in case it overrides the default.
  apply_timezone();
}

void setup_rtc() {
  if (!rtc.begin()) {
    oled.clear();
    oled.println("RTC ERROR");
    vTaskDelay(pdMS_TO_TICKS(500));
  } else {
    oled.clear();
    oled.println("RTC OK");
  }

  bool lost = false;
  if (rtc.lostPower(lost) && lost) {
    rtc.adjust(DateTime(__DATE__, __TIME__));
    //      rtc.adjust(DateTime(2024, 8, 4, 20, 25, 20));
  }

  rtc.now(now);
  start = now;

  // Seed system time from RTC so time(NULL) is valid before NTP sync
  // (event log timestamps, TLS, etc.). SNTP will correct it later.
  struct timeval tv = {.tv_sec = (time_t)now.unixtime(), .tv_usec = 0};
  settimeofday(&tv, NULL);
}
