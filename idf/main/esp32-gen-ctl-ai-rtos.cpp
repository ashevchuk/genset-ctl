// FROZEN 2026-09-23 — this is now real, hand-maintained C++ source, not an
// auto-regenerated snapshot. It started life as arduino-builder's own
// preprocessed intermediate output, copied in one last time by
// ../compile.sh (see git/build_archive history for that final sync) purely
// to inherit its ctags-generated forward-declaration block for free —
// hand-writing prototypes for ~5500 lines of call-order-dependent functions
// from scratch wasn't worth the risk. ../../esp32-gen-ctl-ai-rtos.ino is
// STILL the arduino-builder build's own separate source (compile.sh/
// compile_fast.sh keep building it, unrelated to this file now) — the two
// have diverged as of this freeze and will keep diverging; do not expect
// them to stay in sync, and do not try to restore auto-sync between them.
//
// Going forward: this file is edited directly, like any ordinary C++
// source. If you add a function that's *called* before its own definition
// appears further down in this file, add its forward declaration to the
// prototype block below by hand — same discipline any non-Arduino C++
// project already needs, nothing special. Every #line directive inherited
// from arduino-builder's preprocessed output (mapping this file's lines
// back to the .ino's own numbering) has been stripped, including the ~190
// scattered through the auto-generated prototype block below — leaving any
// of them in place would have kept every compiler diagnostic and crash
// backtrace for the rest of this file pointing at a .ino that no longer
// exists at that path, with line numbers that only drift further from
// reality with each hand-edit. All diagnostics now report this file's own,
// real line numbers.
#include <Arduino.h>
/*
  To upload through terminal you can use: curl -F "image=@firmware.bin"
  esp8266-webupdate.local/update
*/

// #define DEBUG 1
// #define DEBUG_CHARGING    // закомментируй чтобы отключить отладку зарядки

// #define POWERMON

#define MCP39F511A_CONFIGURE   // restore/clone the known-good MCP39F511A calibration
                                   // at boot — see setup(). Uncomment only when actually
                                   // replacing the chip with a factory-default one; re-flash,
                                   // let it apply once, then comment out again.

#define I2C_BUS_CLOCK (100000L)
#define I2C_BUS_TIMEOUT (50)

// WIFI_POWER/WiFi.setTxPower() removed entirely (2026-09-24) — the radio
// now just runs at its default TX power in every mode (AP/STA/update_mode's
// recovery connect).

#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_partition.h"
#include "rtos_config.h"
// CURRENT_TIME/ADC2_SMOOTH/FUEL_PUMP_RISE_HYSTERESIS_L/WIFI_HOSTNAME_DEF —
// included early (not down with the other module headers) since
// wifiHostname's initializer, below, needs WIFI_HOSTNAME_DEF right away.
#include "shared_defines.h"

#include <ESPmDNS.h>
#include <FFat.h>
#include <FS.h>
#include <Update.h>
#include "esp_ota_ops.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <AsyncWiFiMulti.h>
#include <string.h>
#include <ArduinoJson.h>
#include "fram_dev.h" //FM24V10-GTR
// Hoisted above LimitsConfig_t (below) since that struct's
// maint_interval_hours[] needs FLOG_MAINT_TYPES from this header — it used
// to live right before fram_log_service_tick()'s first use, further down.
#include "fram_log.h"

// MenuListItem_t/MenuListDef_t/MenuConfirmDef_t and everything that uses
// them (the whole OLED menu subsystem, DisplayTask included) moved to
// oled_menu.cpp/oled_menu.h on 2026-09-24 — the ctags-hoisting workaround
// that used to force these type definitions up here no longer applies now
// that arduino-builder itself is retired (idf.py has no auto-prototype
// scanner to trip), and with the whole subsystem in its own file there's no
// longer any other reason to declare these types before their first use.

// HTTP (WebServer) + WebSocket (WebSocketsServer) + WiFi captive portal
// (WiFiManager), replacing ESPAsyncWebServer+AsyncTCP+ESPAsync_WiFiManager.
// The old stack's connection-lifecycle allocation churn was root-caused to
// unrecoverable heap fragmentation (heap_caps_get_largest_free_block
// trending down across ordinary WS connect/disconnect cycles, never
// recovering).
//
// WEBSOCKETS_TCP_TIMEOUT: the library defaults this to 5000ms per client
// socket (set via client->tcp->setTimeout() when it accepts a connection —
// WebSocketsServer.cpp). ReportTask's wsServer.broadcastTXT() call
// (~1x/sec) iterates every connected client SEQUENTIALLY and synchronously —
// if a client's TCP write stalls (dropped WiFi association, a phone that
// went out of range mid-connection, ...), that one client can block the
// whole broadcast for up to this many ms before WebSocketsServer gives up
// on it and moves to the next. With 3+ simultaneously-stalled clients that
// alone can exceed the 15s Task Watchdog window ReportTask registers for —
// same failure shape as the MQTT socket-timeout/TWDT race already fixed in
// MqttTask (see project memory: wdt-mqtt-race), just on the WS side instead
// of MQTT. A FLOG_EV_TASK_WDT record naming "Report" as the starved task
// was found live 2026-08-25 (uptime_s 2238) — its exact cause couldn't be
// confirmed (the .elf that produced that build didn't survive to run
// addr2line against; see build_archive/ and compile.sh, added the same day
// specifically so this gap doesn't recur), but this is the only unbounded
// blocking call in ReportTask's loop body, so it's the leading suspect.
// The actual 1500ms override lives in libraries/WebSockets/CMakeLists.txt
// now (target_compile_definitions on that component), not here — a
// `#define` in this file only ever affected this translation unit's own
// preprocessor state, never the library's separately-compiled src/*.cpp
// files that actually read the macro. Found live 2026-09-24: this "fix"
// had been a silent no-op since the day it was added, every build since
// 2026-08-25 ran on the library's real 5000ms default regardless.
#include <WebServer.h>
#include <WebSockets4WebServer.h>
#include <WiFiManager.h>

// Use FFat filesystem through FS interface
FS *filesystem = &FFat;

// WiFi config: stored in /wifi.json on FFat. WIFI_JSON_PATH/_SIZE/
// WIFI_NETWORKS_MAX moved to config.h (2026-09-24) — non-static: web.cpp's
// setup_server_routes_ws()/parse_wifi_core_fields() also need them.
// WIFI_PORTAL_SSID/WIFI_PORTAL_PASS moved to wifi_setup.cpp (2026-09-24) —
// only run_wifi_config_portal() used them. WIFI_HOSTNAME_DEF (needed right
// below, for wifiHostname's own initializer) now comes from
// shared_defines.h (2026-09-25) instead of a local duplicate.

GuLinux::AsyncWiFiMulti asyncWifiMulti;
String wifiHostname = WIFI_HOSTNAME_DEF;
bool wifiModeIsAP = false;  // true = AP, false = STA

// s_wifi_ssid/s_wifi_pass/wifi_get_last_sta_creds() moved to wifi_setup.cpp
// (2026-09-24), alongside wifi_config_apply() — the only writer of those
// two buffers.

// MqttConfig_t/g_mqtt_config/the PubSubClient instance moved to mqtt.cpp
// (2026-09-24) — see mqtt.h.
TaskHandle_t xMqttTask = NULL;
TaskHandle_t xWebTask = NULL;

// ============================================================================
// Safety Limits Configuration (temperature / power protection thresholds)
// ============================================================================
// Channel mapping (see data/dashboard.json chart labels):
//   temp_air_max      <- adc_1_1 (ADC0_1, "Air")
//   temp_oil_max      <- adc_1_0 (ADC0_0, "Oil")
//   temp_engine_max   <- flog_head_temp() / MCP9600 thermocouple ("Engine")
//   temp_inverter_max <- adc_1_2 (ADC0_2, "Inverter")
// A value of 0 disables that limit.
// LimitsConfig_t itself now lives in rtos_config.h (same reason as
// SharedState_t above — oled_menu.cpp's MENU_MAINT screen reads
// maint_interval_hours directly). Non-static: see its own
// `extern LimitsConfig_t g_limits_config;`.
LimitsConfig_t g_limits_config = {
    .temp_air_max            = 0.0f,
    .temp_oil_max            = 0.0f,
    .temp_engine_max         = 0.0f,
    .temp_inverter_max       = 0.0f,
    .power_max               = 0.0f,
    .auto_load_relay_on_run  = true,  // matches the pre-existing unconditional behavior
    .choke_cold_start_temp   = 5.0f,  // matches the old hardcoded CHOKE_COLD_START_TEMP default
    .maint_interval_hours    = {0.0f},
};

// Single source for FLOG_MAINT_* human labels, indexed by subtype — used by
// both ReportTask's MAINT_DUE dashboard banner and the OLED menu's
// MENU_MAINT screen (oled_menu.cpp) — non-static, prototype in
// rtos_config.h, for that reason. data/journal.html.gz's MAINT_TYPES array
// duplicates this on the JS side (can't share a C array with the browser)
// — keep that in sync by hand if a label changes here.
const char *MAINT_TYPE_LABELS[FLOG_MAINT_TYPES] = {
    "Oil change", "Spark plug", "Air filter",
    "Fuel filter", "Valve clearance", "Other"
};

// safety_start_inhibited() prototype now comes from statemachine.h
// (2026-09-24, pass 1 of the StateMachineTask cut) — still called from
// start_working()/start_retry_working() below.

// TimingConfig_t/BatteryConfig_t/EngineConfig_t/TimeConfig_t and their
// g_*_config instances moved to config.cpp/config.h (2026-09-24) — see
// that header for the struct layouts.

// Task-watchdog panic capture (RTC no-init memory + ISR hook) lives in
// fram_log.cpp — a plain .cpp file, unlike this .ino, isn't rewritten by
// Arduino's ctags-based auto-prototype scanner, which chokes on the
// __attribute__((section(...))) that RTC_NOINIT_ATTR expands to.

#include "ads1115_dev.h"
#include "ina233_dev.h"
#include "smoothed_safe.h"
#include <Wire.h>

#include "SSD1306Ascii.h" // font/glyph/cursor engine only, kept from the library — see oled_dev.h
#include "oled_dev.h"

#include "encoder.h"

#include "datetime.h" // DateTime value type — own port of RTClib's, no I2C
#include "rtc_dev.h"

RtcDs3231 rtc;

#include "tacho.h"

// I2C Adresses for the 2 INA233 on board
Ina233Dev IC1(0x45); // IC1 address 0x45 1000101

// 0X3C+SA0 - 0x3C or 0x3D
#define OLED_I2C_ADDRESS 0x3C

#include "shiftreg.h"
#include "buzzer.h"
#include "oled_menu.h"
#include "mqtt.h"
#include "config.h"
#include "powermeter.h"
#include "report.h"
#include "i2c.h"
#include "web.h"
#include "gen_discovery.h"
#include "statemachine.h"
#include "update_mode.h"
#include "wifi_setup.h"
#include "esp_now_link.h"

OledDev oled;

#include "mcp9600_dev.h"
Mcp9600Dev tempSensor;

#define KI2C_ADDRESS (0x60)

#include "mcp39f511a_dev.h"

#include <sys/time.h>
#include <time.h>
#include "esp_wifi.h"

// CONFIG_SNTP_TIME_SERVER/ntpServer1/ntpServer2/esp_netif_sntp.h/esp_sntp.h
// moved to i2c.cpp (2026-09-24) alongside setup_ntp() — time_t itself
// (ntp_sync_time below) is the only reason <time.h>/<sys/time.h> stay
// included here.


// MCP39F511A Configuration structure - project-owned (Mcp39f511aConfig,
// mcp39f511a_dev.h) — datasheet register map, ESP-IDF UART transport.
// mcp39f511a_config moved to powermeter.cpp (2026-09-24) alongside the
// print/load helpers that own it.

// UART2 brought up via mcp39f511a_uart_begin() in setup_powermeter_hw().
Mcp39f511aDev mcp39f511a;

// ============================================================================
// FreeRTOS Handles (SharedState_t defined after state enum below)
// ============================================================================

// Mutexes for resource protection
SemaphoreHandle_t xFlashI2CGuard = NULL;
SemaphoreHandle_t xSerial2Mutex = NULL;
SemaphoreHandle_t xShiftRegMutex = NULL;
SemaphoreHandle_t xStateMutex = NULL;
// Guards WebSocketsServer's internal client list. The library has no lock
// of its own, and three call sites touch that list from two tasks:
//   - wsServer.loop() (WebTask)
//   - the /ws accept hook inside webServer.handleClient() (WebTask)
//   - broadcastTXT()/connectedClients() (ReportTask)
// Accept has to take it too: newClient() publishes client->tcp before it
// sets WSC_HEADER, and a concurrent clientIsConnected() treats that window
// as a dead slot and clientDisconnect()s the socket just handed out.
// Observed live without the lock: LoadProhibited in handleClientData()
// reading a client->tcp the other task had just nulled.
SemaphoreHandle_t xWsMutex = NULL;

// Queues for inter-task communication
QueueHandle_t xPowerDataQueue = NULL;
QueueHandle_t xOutputCommandQueue = NULL;
QueueHandle_t xWebCommandQueue = NULL;
QueueHandle_t xBuzzerCommandQueue = NULL;
QueueHandle_t xI2CJobQueue = NULL;
QueueHandle_t xMenuRenderQueue = NULL;  // MenuState_t (see esp32-gen-ctl-ai-rtos.ino's menu code)

// Event group for state machine notifications
EventGroupHandle_t xStateEvents = NULL;

// Task handles
TaskHandle_t xStateMachineTask = NULL;
TaskHandle_t xShiftRegTask = NULL;
TaskHandle_t xPowerMeterTask = NULL;
TaskHandle_t xDisplayTask = NULL;
TaskHandle_t xReportTask = NULL;
TaskHandle_t xI2CTask = NULL;
TaskHandle_t xEspNowTask = NULL;

// Spinlock for critical sections — non-static: see rtos_config.h's own
// `extern portMUX_TYPE g_state_spinlock;` for why.
portMUX_TYPE g_state_spinlock = portMUX_INITIALIZER_UNLOCKED;

// Flag to indicate RTOS tasks are running
volatile bool g_rtos_running = false;  // non-static: web.cpp also reads this

// ============================================================================

#define MAX_SRV_CLIENTS 5
WiFiServer sserver(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];

/*
ADC0_0 - oil temperature
ADC0_1 - ambient temperature
ADC0_2 - inverter temperature
ADC0_3 - alternator temperature
ADC1_0 - fuel level(litres)
ADC1_1 - battery voltage
ADC1_2 - core voltage
*/

/*
  CD4021BE
  ----------------
  CLK - IO18 (Shift register clock pin)
  P/S - IO23 (Storage register clock pin (latch pin))
  Q8  - IO19 (DATA_PIN)
  P1  - OVERLOAD
  P2  - OIL
  P3  - OUTPUT
  P4  - OVERHEAT
  P5  - PSU SENSE
  P6  - GEN SENSE
  P7  - BAT SENSE
  P8  - N/C

  SN74HC595N
  ---------------
  SER    - IO27 (DATA_PIN)
  OE#    - IO26 (Output enable[active low])
  RCLK   - IO25 (Storage register clock pin (latch pin))
  SRCLK  - IO33 (Shift register clock pin)
  SRCLR# - IO32 (Master Reclear[active low])

  U5_0
  --------------
  QA - BATT FORCE
  QB - BATT OFF
  QC - IGNITION
  QD - START
  QE - ECO MODE
  QF - LOAD RELAY
  QG - CHARGER
  QH - N/C

  U3_1
  --------------
  QA - CHOKE REVERSE
  QB - CHOKE
  QC - FUEL PUMP
  QD - BUZZER
  QE - LED
  QF - N/C
  QG - N/C
  QH - N/C
*/

// The 8 PIN_SHIFT_REG_*/PIN_SHIFT_REG_IN_* pins moved to shiftreg.cpp
// (2026-09-24) — private to that module, nothing outside it ever
// referenced them by name (confirmed via grep before the extraction).
enum PIN_CONFIG {
  PIN_BTN_MODE = 35,
  PIN_ENC_BTN = 34,
  PIN_ENC_A = 39, // SENSOR_VN
  PIN_ENC_B = 36, // SENSOR_VP
};

// Encoder steps are defined in encoder.h (ENCODER_STEPS = 4)

// SHIFT_REG/SHIFT_REG_IN themselves now live in rtos_config.h — oled_menu.cpp
// reads REG_BATT_OFF/REG_FUEL_PUMP/REG_IN_OIL/etc. directly (menu register
// toggles, the status screen's safety-trip readout).

// ShiftRegItem/ShiftRegList/ShiftRegs/ShiftRegsIn/lengths moved to
// shiftreg.cpp (2026-09-24), non-static — this file's raw-WS-command
// handler and dashboard-state handler still read them by name.

/*
  In order to set the address,
  connect the address pin to:

  GND -> 0x48 (or leave unconnected)
  VCC -> 0x49
  SDA -> 0x4A
  SCL -> 0x4B
*/

#define I2C_ADDRESS_1 0x49
#define I2C_ADDRESS_2 0x48

// Higher numbers will result in less filtering/smoothing. Lower number result
// in more filtering/smoothing #define ADC1_SMOOTH 6 #define ADC1_SMOOTH_METHOD
// SMOOTHED_EXPONENTIAL

// The more values you store, the more memory will be used.
#define ADC1_SMOOTH 30
#define ADC1_SMOOTH_METHOD SMOOTHED_AVERAGE

// ADC2_SMOOTH itself now comes from shared_defines.h (2026-09-25) —
// statemachine.cpp needed the identical value too and used to carry its own
// duplicate. _METHOD has no other caller and stays defined here.
#define ADC2_SMOOTH_METHOD SMOOTHED_AVERAGE

#define RPM_SMOOTH 30
#define RPM_SMOOTH_METHOD SMOOTHED_AVERAGE

#define BUS_VOLTAGE (3287)
uint16_t adc_bus_voltage = BUS_VOLTAGE;

#define RT0 10000 // Ω
#define B 3977    // K
#define R 10000   // R=10KΩ

#define K 9.5 // Dissipation factor (mW/°C)

double resistance = 10000.0;

double a = 1.129148e-3;
double b = 2.34125e-4;
double c = 8.76741e-8;

/*
1 -1.2
2 -5.6
3 -5.6
4 -6.4
*/
Ads1115Dev adc_1(I2C_ADDRESS_1);

// SmoothedSafe<T> itself now lives in smoothed_safe.h (shared with
// oled_menu.cpp's i2c_render_display(), which reads adc_1_0../adc_2_1
// directly). smoothed_mux is defined here, once, non-static — see the
// header's own comment for why it must not be `static` anywhere it's
// included.
portMUX_TYPE smoothed_mux = portMUX_INITIALIZER_UNLOCKED;

SmoothedSafe<double> adc_1_0;
SmoothedSafe<double> adc_1_1;
SmoothedSafe<double> adc_1_2;
SmoothedSafe<double> adc_1_3;

Ads1115Dev adc_2(I2C_ADDRESS_2);
SmoothedSafe<double> adc_2_0;
SmoothedSafe<double> adc_2_1;
SmoothedSafe<double> adc_2_2;

SmoothedSafe<uint32_t> rpm_smoothed;

// FRAM32: FM24V10 is 128 KB in two 64 KB pages selected by the A16 bit in the
// slave address (0x56 + 0x57); the base FRAM class only reaches page 0.
FRAM32 fram;


DateTime now;
DateTime start;

// wifi_fb_host/wifi_fb_ssid/wifi_fb_password moved to update_mode.cpp
// (2026-09-24) — only that file's fallback-AP connect logic used them.

// HTTP (WebServer) + WebSocket (WebSockets4WebServer) transport, replacing
// ESPAsyncWebServer+AsyncTCP — see CLAUDE.md's "Web/MQTT interfaces"
// section for the full migration rationale (heap fragmentation under
// AsyncTCP's per-connection allocation churn). Both share port 80 now
// (WS at /ws) — arduino-esp32's own WebServer never had the hook mechanism
// WebSockets4WebServer needs to do this (unlike ESP8266/RP2040), so
// libraries/WebServer/ vendors a patched copy that adds it; see that
// library's own CMakeLists.txt/WebServer.h comments and
// idf/patch_arduino_esp32.sh (disables arduino-esp32's own built-in
// WebServer so the two don't collide at link time).
//
// run_wifi_config_portal() creates its own short-lived internal WebServer
// (owned by its WiFiManager instance) on port 80, torn down by
// shutdownConfigPortal() before startConfigPortal() returns — no port
// conflict with this webServer.begin(), called later in setup().
WebServer webServer(80);
WebSockets4WebServer wsServer;

// Global variables for file upload
File uploadFile;
String uploadPath;

// Separate state for the WebServer (sync) equivalents of /api/upload and
// /update (Phase 3c) — kept distinct from the Async* globals above so the
// two stacks can be exercised side-by-side during migration testing without
// one clobbering the other's in-flight upload.
File uploadFileWs;
String uploadPathWs;


// shift_reg/shift_reg_in and every REG_*/IS_REG_* macro moved to
// shiftreg.h (2026-09-24) — included below, near the other local module
// headers, so every call site in this file keeps working unchanged.

// Battery voltage thresholds and read periods now live in g_battery_config
// (see BatteryConfig_t, configurable via /wifi-config).

// CURRENT_TIME now comes from shared_defines.h (2026-09-25) — statemachine.cpp
// needed the identical macro too and used to carry its own duplicate.

// afterRunPeriodTime/afterRunPeriod/readBatTime/startTime/startRetryTime/
// startTry/stoppingTime/loadOffTime/stoppingToRetry/runningTime moved to
// statemachine.cpp (2026-09-24, pass 1 of the StateMachineTask cut) —
// declared extern in statemachine.h, still read/written directly by the
// switch_STATE_*/process_STATE_* handlers below until pass 2 moves those
// here too.

// Non-static: config.cpp's battery_config_load() resyncs this on a config
// change. Initializer is BatteryConfig_t's own default read_period_ms
// literal, not a reference to g_battery_config itself — that instance now
// lives in a different translation unit (config.cpp), and cross-TU static
// initialization order is unspecified, so referencing it here directly
// would be a real (if usually-invisible) hazard rather than just style.
unsigned long long readBatPeriod = 1000ULL * 60 * 60 * 12;

// batt_low_since_ms/charge_pending_since_ms moved to statemachine.cpp
// (2026-09-24, pass 2 of the StateMachineTask cut).

// READ_REG_PERIOD/readRegTime moved to shiftreg.cpp (2026-09-24) — both
// unused outside read_regs() itself even before the move (readRegTime is
// write-only, READ_REG_PERIOD had no reader at all); left as dead weight
// rather than deleted, a pure-move refactor is the wrong moment to also
// judge whether they're safe to drop.

// Start/stop/run timing and retry count now live in g_timing_config (see
// TimingConfig_t, configurable via /wifi-config).
//
// Load relay must physically drop before ignition/choke/start are touched,
// so the alternator isn't still feeding the load at the instant the engine
// dies. STATE_STOPPING_LOAD_OFF only cuts REG_LOAD_RELAY and waits
// g_timing_config.load_relay_stop_delay_ms before the real stop sequence
// (STATE_STOPPING/STATE_STOPPING_RETRY) runs.

#define RPM_DIV (20)

// RPM detection thresholds now live in g_engine_config (see EngineConfig_t,
// configurable via /wifi-config).

// charging moved to statemachine.cpp (2026-09-24, pass 2).
bool mcp9600_present = false; // set in setup(), avoids repeated NACK if sensor absent; non-static: i2c.cpp also reads this
bool ina233_present  = false; // set in setup(), avoids repeated NACK if sensor absent; non-static: i2c.cpp also reads this

// NTP sync flag - written by i2c.cpp's sync_time_cb() (SNTP task context),
// processed here in loop(). Non-static: moved to i2c.cpp (2026-09-24)
// alongside setup_ntp()/setup_rtc(), this file just reads them now.
extern volatile bool ntp_sync_flag;
extern volatile time_t ntp_sync_time;

uint32_t tachoRPM_last = 0;
float tachoFREQ_last = 0.0f;

//==============================================================================
// state/SharedState_t themselves live in rtos_config.h. runtimeState/
// prevState and the PREV_STATE/CURRENT_STATE/STATE(...)/etc. macros moved to
// statemachine.h/.cpp (2026-09-24, pass 2 of the StateMachineTask cut) —
// still non-static/extern-declared there, since is_not_working()/
// start_working()/start_retry_working()/stop_working() below still read/
// drive the state machine through them.

// Global shared state instance — non-static: see rtos_config.h's own
// `extern SharedState_t g_state;` for why.
SharedState_t g_state = {
    .current_state = STATE_INIT,
    .previous_state = STATE_INIT,
    .state_entry_time = 0,
    .sensors = {0},
    .power = {0},
    .shift_reg_inputs = 0,
    .shift_reg_outputs = 0,
    .start_time = 0,
    .start_retry_time = 0,
    .stopping_time = 0,
    .running_time = 0,
    .start_try = 0,
    .charging = false,
    .after_run_period = false,
    .after_run_period_time = 0,
    .read_bat_period = 0,
    .read_bat_time = 0
};
// ============================================================================

// #define SDEBUG(data) serial_debug(data)

#include "debug.h"
//==============================================================================

struct FuelLevelCurvePoint_t {
  double fuel_level;
  double adc_value;
};

typedef FuelLevelCurvePoint_t FuelLevelCurvePoints_t[];

// fuel level => adc voltage
static FuelLevelCurvePoints_t FuelLevelCurvePoints = {
    /*
        { 0,	2965      },
        { 3,	2470      },
        { 4,	2218      },
        { 5,	1975      },
        { 5.85,	1738      },
        { 6.85,	1555      },
        { 7.75,	1405      },
        { 8.65,	1150      },
    */

    {0.0f, 2900},  {0.01f, 2999},

    {3.0f, 2400},  {3.01f, 2499},

    {4.0f, 2100},  {4.01f, 2299},

    {5.0f, 1900},  {5.01f, 1999},

    {5.85f, 1700}, {5.851f, 1799},

    {6.85f, 1500}, {6.851f, 1599},

    {7.75f, 1300}, {7.751f, 1499},

    {8.65f, 1100}, {8.651f, 1199},
};

//=============================

struct BatVoltageCurvePoint_t {
  double bat_voltage;
  double adc_value;
};

typedef BatVoltageCurvePoint_t BatVoltageCurvePoints_t[];

// bat voltage => adc voltage
static BatVoltageCurvePoints_t BatVoltageCurvePoints = {
    {
        0,
        1,
    },

    {4, 713.5},
    {4.5, 799},
    {5, 875},
    {5.5, 928},
    {6, 961},
    {6.5, 983},
    {7, 999},
    {7.5, 1011},
    {8, 1023},
    {8.5, 1031.5},
    {9, 1038.5},
    {9.5, 1045},
    {10, 1050},
    {10.5, 1055},
    {11, 1059.5},
    {11.5, 1063.5},
    {12, 1067},
    {12.5, 1070.5},
    {13, 1073.5},
    {13.5, 1076.5},
    {14, 1079.5},
    {14.5, 1082},
    {15, 1084.5},
    {15.5, 1087},
    {16, 1089.5},

    {24, 1125},
    {99, 3200},
};

double calcBatVoltageCurve(BatVoltageCurvePoints_t curve, const uint8_t curveLength, const double adc_value);
double calcFuelLevelCurve(FuelLevelCurvePoints_t curve, const uint8_t curveLength, const double adc_value);
double steinhartHart(double r);
double adc_temp_pos(const double adc_voltage);
double adc_temp(const double adc_voltage);
double bat_voltage(const double adc_voltage);
const char * resetReasonName(esp_reset_reason_t r);
// shiftreg_lock/unlock/reg_read/read_regs/setup_shift_reg_in/reg_write/
// reg_set_state/reg_set/setup_shift_reg prototypes now come from
// shiftreg.h (2026-09-24).
double readChannel1(ADS1115_MUX channel);
double readChannel2(ADS1115_MUX channel);
uint32_t tachoRPM(void);
float tachoFREQ(void);
bool is_running(void);
bool is_stopped(void);
bool is_not_working(void);
void start_working();
void start_retry_working();
void stop_working();
void rotary_loop();
// onWsServerEvent()/setup_server_routes_ws()/serve_static_ws()/
// parse_wifi_core_fields() prototypes now come from web.h (2026-09-24).
// WiFiEvent()/WiFiGotIP()/wifi_config_load()/wifi_config_save()/
// wifi_config_apply()/run_wifi_config_portal()/wifi_get_last_sta_creds()
// prototypes now come from wifi_setup.h (2026-09-24).
// sync_time_cb()/setup_ntp()/setup_rtc() prototypes now come from i2c.h
// (2026-09-24). apply_timezone() prototype comes from config.h.
// mcp39f511a_load_config_from_flash()/mcp39f511a_print_config()/
// mcp39f511a_print_config_serial()/mcp39f511a_print_full_config_serial()/
// mcp39f511a_print_energy_counters_serial() prototypes now come from
// powermeter.h (2026-09-24).
static constexpr int fw_month_num(const char *d);
static constexpr char fw_digit(int v);
void setup(void);
// job_read_ina233_now()/read_ina233_now()/measure_bat_voltage()/
// choke_ensure_battery()/choke_release_battery()/psu_backup_check()/
// flog_head_temp()/flog_head_temp_or_warm()/flog_charge_started()/
// flog_charge_stopped()/flog_on_transition() prototypes now come from
// statemachine.h (2026-09-24, pass 1 of the StateMachineTask cut).
void serial_debug(const char *data);
// switch_STATE_*/process_STATE_*/switch_state()/process_current_state()/
// process_state_check()/sendOutputCommand()/sendSyncCommand()/
// StateMachineTask() prototypes now come from statemachine.h (2026-09-24,
// pass 2 of the StateMachineTask cut).
// sensor_log_fail_once/I2CTask/i2c_read_sensors/rtc_now_unixtime/
// i2c_service_rtc/job_rtc_adjust/rtc_adjust_now prototypes now come from
// i2c.h (2026-09-24).
// ShiftRegTask prototype now comes from shiftreg.h (2026-09-24).
// PowerMeterTask prototype now comes from powermeter.h (2026-09-24).
// Every menu_*/mm_*/gc_*/rc_*/mt_*/mc_*/DisplayTask/i2c_render_display
// prototype that used to sit here moved to oled_menu.cpp along with their
// definitions (2026-09-24) — DisplayTask/i2c_render_display are the only
// two still called from this file, and their declarations now come from
// oled_menu.h instead.
// ws_fmt_num()/ReportTask prototypes now come from report.h (2026-09-24).
// mqtt_config_load()/MqttTask() prototypes now come from mqtt.h (2026-09-24).
// limits/timing/battery/engine/time_config_load() prototypes now come from
// config.h (2026-09-24).
// state_name() prototype now comes from rtos_config.h (non-static — see there).
void WebTask(void *pvParameters);
void setup_rtos();
void loop_serial(void);
void loop(void);
double calcBatVoltageCurve(BatVoltageCurvePoints_t curve,
                           const uint8_t curveLength, const double adc_value) {
  BatVoltageCurvePoint_t *start = nullptr;
  BatVoltageCurvePoint_t *end = nullptr;

  for (int i = 0; i < curveLength; i++) {

    if (curve[i].adc_value > adc_value) {
      end = &curve[i];
      break;
    }
    start = &curve[i];
  }

  if (start == nullptr) {
    return end->bat_voltage;
  }
  if (end == nullptr) {
    return start->bat_voltage;
  }

  // interpolate
  double pitch = (end->bat_voltage - start->bat_voltage) /
                 (end->adc_value - start->adc_value);
  double offset = start->bat_voltage - pitch * start->adc_value;
  return pitch * adc_value + offset;
}

#define BAT_VOLTAGE_CURVE(adc_value)                                           \
  calcBatVoltageCurve(BatVoltageCurvePoints,                                   \
                      sizeof(BatVoltageCurvePoints) /                          \
                          sizeof(BatVoltageCurvePoint_t),                      \
                      adc_value)

double calcFuelLevelCurve(FuelLevelCurvePoints_t curve,
                          const uint8_t curveLength, const double adc_value) {
  FuelLevelCurvePoint_t *start = nullptr;
  FuelLevelCurvePoint_t *end = nullptr;

  for (int i = 0; i < curveLength; i++) {

    if (curve[i].adc_value < adc_value) {
      end = &curve[i];
      break;
    }
    start = &curve[i];
  }

  if (start == nullptr) {
    return end->fuel_level;
  }
  if (end == nullptr) {
    return start->fuel_level;
  }

  // interpolate
  double pitch = (end->fuel_level - start->fuel_level) /
                 (end->adc_value - start->adc_value);
  double offset = start->fuel_level - pitch * start->adc_value;
  return pitch * adc_value + offset;
}

#define FUEL_LEVEL_CURVE(adc_value)                                            \
  calcFuelLevelCurve(                                                          \
      FuelLevelCurvePoints,                                                    \
      sizeof(FuelLevelCurvePoints) / sizeof(FuelLevelCurvePoint_t), adc_value)

// Same computation, callable from another translation unit — FUEL_LEVEL_CURVE
// itself can't cross files, since it textually expands to a reference to the
// static FuelLevelCurvePoints table above. oled_menu.cpp's fuel gauge and
// status screen use this instead of the macro.
double fuel_level_from_adc(double adc_value) {
  return FUEL_LEVEL_CURVE(adc_value);
}

// Fuel level at which FUEL_PUMP must be turned off and blocked from turning on
// now lives in g_engine_config.fuel_pump_stop_level (configurable via /wifi-config).

// FUEL_PUMP_RISE_HYSTERESIS_L now comes from shared_defines.h (2026-09-25) —
// statemachine.cpp needed the identical macro too and used to carry its own
// duplicate.

/**
   Returns the temperature in kelvin for the given resistance value
   using the Steinhart-Hart polynom.
*/
double steinhartHart(double r) {
  double log_r = log(r);
  double log_r3 = log_r * log_r * log_r;

  return 1.0 / (a + b * log_r + c * log_r3);
}



double adc_temp_pos(const double adc_voltage) {
  double _resistance = resistance * ((adc_bus_voltage / adc_voltage) - 1);
  return steinhartHart(_resistance) - 273.15;
}

double adc_temp(const double adc_voltage) { return adc_temp_pos(adc_voltage); }

double bat_voltage(const double adc_voltage) {
  if (!ina233_present) return 12.4f;
  return BAT_VOLTAGE_CURVE(adc_voltage);
}

const char *resetReasonName(esp_reset_reason_t r) {
  switch (r) {
  case ESP_RST_UNKNOWN:
    return "Unknown";
  case ESP_RST_POWERON:
    return "PowerOn"; // Power on or RST pin toggled
  case ESP_RST_EXT:
    return "ExtPin"; // External pin - not applicable for ESP32
  case ESP_RST_SW:
    return "Reboot"; // esp_restart()
  case ESP_RST_PANIC:
    return "Crash"; // Exception/panic
  case ESP_RST_INT_WDT:
    return "WDT_Int"; // Interrupt watchdog (software or hardware)
  case ESP_RST_TASK_WDT:
    return "WDT_Task"; // Task watchdog
  case ESP_RST_WDT:
    return "WDT_Other"; // Other watchdog
  case ESP_RST_DEEPSLEEP:
    return "Sleep"; // Reset after exiting deep sleep mode
  case ESP_RST_BROWNOUT:
    return "BrownOut"; // Brownout reset (software or hardware)
  case ESP_RST_SDIO:
    return "SDIO"; // Reset over SDIO
  default:
    return "";
  }
}

// Low-level shift-register I/O (shiftreg_lock/unlock, reg_read, read_regs,
// setup_shift_reg_in, reg_write, reg_set_state, reg_set, setup_shift_reg)
// moved to shiftreg.cpp (2026-09-24).

double readChannel1(ADS1115_MUX channel) {
  adc_1.setCompareChannels(channel);
  return adc_1.getResult_mV();
}

double readChannel2(ADS1115_MUX channel) {
  adc_2.setCompareChannels(channel);
  return adc_2.getResult_mV();
}

uint32_t tachoRPM(void) {
  tachoRPM_last = tacho_get_rpm() / RPM_DIV;

  if (tachoRPM_last > 0) {
    if (tachoRPM_last < g_engine_config.rpm_max) {
      rpm_smoothed.add(tachoRPM_last);
    }
    tachoRPM_last = rpm_smoothed.get();
  } else {
    for (uint8_t i = 0; i < RPM_SMOOTH; i++) {
      rpm_smoothed.add(0);
    }

    tachoRPM_last = 0;
  }

  return tachoRPM_last;
}

float tachoFREQ(void) {
  tachoFREQ_last = tacho_get_hz();
  return tachoFREQ_last;
}

bool is_running(void) {
  uint32_t _tachoRPM = tachoRPM();

  if ((_tachoRPM >= g_engine_config.rpm_running_threshold) && (_tachoRPM > g_engine_config.rpm_starting)) {
    SDEBUG("Is running true by RPM");
    SDEBUG("RPM:", _tachoRPM);
    return true;
  }

  // Use shift_reg_in maintained by ShiftRegTask (via xShiftRegMutex + spinlock).
  // Do NOT call read_regs() here — it races with ShiftRegTask's own reg_read()
  // calls on the same GPIO pins (LATCH/CLOCK/DATA), both on Core0, producing
  // corrupt shift register reads that falsely set OUTPUT=1.
  if (IS_REG_IN_ON(REG_IN_OUTPUT)) {
    SDEBUG("Is running true by OUTPUT");
    return true;
  }

  return false;
}

bool is_stopped(void) {
  uint32_t _tachoRPM = tachoRPM();

  if (_tachoRPM == 0) {
    if (IS_REG_IN_OFF(REG_IN_OUTPUT) && IS_REG_IN_OFF(REG_IN_GEN_SENSE)) {
      return true;
    }
  }

  return false;
}

bool is_not_working(void) {
  uint32_t _tachoRPM = tachoRPM();

  if (_tachoRPM == 0) {
    if (IS_REG_IN_OFF(REG_IN_OUTPUT) && IS_REG_IN_OFF(REG_IN_GEN_SENSE) &&
        IS_CURRENT_STATE(STATE_IDLE)) {
      return true;
    }
  }

  return false;
}

void start_working() {
  if (is_not_working() && !safety_start_inhibited()) {
    STATE(STATE_STARTING_SINGLE);
  }
}

void start_retry_working() {
  if (is_not_working() && !safety_start_inhibited()) {
    STATE(STATE_STARTING_RETRY);
  }
}

// Always the full stop sequence, including from IDLE — same reason as
// WEB_CMD_STOP in StateMachineTask: a hand-started engine the controller
// never saw as running still has to be killable.
void stop_working() { STATE(STATE_STOPPING_LOAD_OFF); }

void rotary_loop() {
  // Encoder events are now consumed by menu_process_encoder() in DisplayTask.
}

// TachoISR removed — PCNT counts edges in hardware (see tacho.cpp)

// Encoder ISRs are now internal to encoder.cpp (hardware PCNT + minimal button ISR)

// onWsServerEvent() moved to web.cpp (2026-09-24).

// WARNING: This function is called from a separate FreeRTOS task (thread)!
// WiFiEvent()/WiFiGotIP()/wifi_config_load()/wifi_config_save()/
// wifi_config_apply()/run_wifi_config_portal() moved to wifi_setup.cpp
// (2026-09-24).
// sync_time_cb()/setup_ntp()/setup_rtc() moved to i2c.cpp (2026-09-24).
// printLocalTime() moved there too, then deleted outright — confirmed
// dead code, never called from anywhere in the firmware.


// Read configuration registers from MCP39F511A
// mcp39f511a_read_config()/mcp39f511a_load_config_from_flash()/
// mcp39f511a_print_config()/mcp39f511a_print_config_serial()/
// mcp39f511a_print_full_config_serial()/
// mcp39f511a_print_energy_counters_serial() moved to powermeter.cpp
// (2026-09-24).

// static_request_rate_exceeded()/mime_type_for_path()/serve_static_ws()
// moved to web.cpp (2026-09-24).

// Overrides the weak `esp_app_desc` symbol that arduino-esp32's precompiled
// libesp_app_format.a otherwise provides (a fixed vendor placeholder
// describing the esp32-libs release itself, not this sketch — arduino-builder
// never regenerates a per-project app descriptor the way esp-idf's cmake
// build does). __DATE__/__TIME__ are evaluated when THIS .cpp is compiled.
// ninja skips this TU on incremental rebuilds of other files, so every
// intentional `idf.py build` path (compile*.sh / flash*.sh) runs
// idf/bump_fw_version.sh first — touches this file so version[] always
// tracks wall-clock build time, not the last time this TU happened to
// change. __DATE__ is "Mmm dd yyyy" (dd space-padded), __TIME__ is
// "hh:mm:ss" — folded into version[] below as "yyyymmddhhmm".
static constexpr int fw_month_num(const char *d) {
  return (d[0] == 'J' && d[1] == 'a') ? 1 :
         (d[0] == 'F')                ? 2 :
         (d[0] == 'M' && d[2] == 'r') ? 3 :
         (d[0] == 'A' && d[1] == 'p') ? 4 :
         (d[0] == 'M' && d[2] == 'y') ? 5 :
         (d[0] == 'J' && d[2] == 'n') ? 6 :
         (d[0] == 'J' && d[2] == 'l') ? 7 :
         (d[0] == 'A' && d[1] == 'u') ? 8 :
         (d[0] == 'S')                ? 9 :
         (d[0] == 'O')                ? 10 :
         (d[0] == 'N')                ? 11 : 12;
}
static constexpr char fw_digit(int v) { return static_cast<char>('0' + v); }
static constexpr int fw_build_month = fw_month_num(__DATE__);
static constexpr int fw_build_day = (__DATE__[4] == ' ' ? 0 : __DATE__[4] - '0') * 10 + (__DATE__[5] - '0');

static const esp_app_desc_t k_fw_app_desc = {
    ESP_APP_DESC_MAGIC_WORD,       // magic_word
    0,                              // secure_version
    {0, 0},                         // reserv1[2]
    {__DATE__[7], __DATE__[8], __DATE__[9], __DATE__[10],
     fw_digit(fw_build_month / 10), fw_digit(fw_build_month % 10),
     fw_digit(fw_build_day / 10), fw_digit(fw_build_day % 10),
     __TIME__[0], __TIME__[1], __TIME__[3], __TIME__[4], 0},  // version[32]: yyyymmddhhmm
    "esp32-gen-ctl-ai-rtos",        // project_name[32]
    __TIME__,                       // time[16]
    __DATE__,                       // date[16]
    "arduino-esp32",                // idf_ver[32]
    {0},                             // app_elf_sha256[32]
    0, 0,                            // min/max_efuse_blk_rev_full
    0,                               // mmu_page_size
    0,                               // spi_flash_mode
    {0, 0},                          // reserv3[2]
    {0},                             // reserv2[18]
};
extern "C" const esp_app_desc_t esp_app_desc
    __attribute__((section(".rodata_desc"))) __attribute__((used)) = k_fw_app_desc;

// webServerNotFound()/fw_ota_state_name()/fw_append_partition_json()/
// fw_build_partitions_json()/setup_server_routes_ws() moved to web.cpp
// (2026-09-24).

// update_mode() moved to update_mode.cpp (2026-09-24).

// Pin/serial/shift-register bring-up, initial relay/LED/buzzer state, WiFi
// event-handler registration, Wire/OLED bring-up — everything before the
// REG_IN_JP1 update-mode check.
static void setup_early_hw(void) {
  pinMode(PIN_BTN_MODE, INPUT);

  // PIN_ENC_A/B/BTN are configured by encoder_setup() via PCNT driver

  setup_shift_reg();
  setup_shift_reg_in();

  read_regs();

  Serial.begin(115200);
  SLOG_PRINTLN();
  SLOG_PRINTLN("Booting Sketch...");

  REG_ON(REG_LED0);

  REG_ON(REG_BUZZER);
  delay(10);
  REG_OFF(REG_BUZZER);

  REG_ON(REG_LOAD_RELAY);

  // delete old config
  WiFi.disconnect(true);

  delay(1000);

  WiFi.onEvent(WiFiEvent);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);

  Wire.begin();

  oled.begin(&Adafruit128x32, OLED_I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
  oled.clear();

  Wire.setClock(I2C_BUS_CLOCK);
  Wire.setTimeOut(I2C_BUS_TIMEOUT); // 50ms I2C transaction timeout to prevent ISR spin on NACK → Interrupt WDT
}

// Mounts FFat (auto-formats on first-ever boot if mount fails), logs
// capacity to Serial + a short OLED status.
static void setup_ffat(void) {
  // Initialize FFat filesystem
  oled.clear();
  oled.println("init FFat...");
  SLOG_PRINTLN("Initializing FFat filesystem...");

  // FFat.begin(true) will automatically format the filesystem if it fails to
  // mount
  if (!FFat.begin(true)) {
    SLOG_PRINTLN("FFat Mount Failed!");
    SLOG_PRINTF("Error: FFat partition may not exist or is corrupted\n");
    SLOG_PRINTLN("Note: Make sure partition table includes 'fat' partition");
    SLOG_PRINTLN("Partition scheme should be: app3M_fat9M_16MB");
    oled.println("FFat FAIL");
    delay(500);
  } else {
    SLOG_PRINTLN("FFat Mount Success");
    size_t totalBytes = FFat.totalBytes();
    size_t usedBytes = FFat.usedBytes();
    size_t freeBytes = FFat.totalBytes() - FFat.usedBytes();
    SLOG_PRINTF("FFat Total: %u bytes (%.2f MB)\n", totalBytes,
                  totalBytes / (1024.0 * 1024.0));
    SLOG_PRINTF("FFat Used: %u bytes (%.2f MB)\n", usedBytes,
                  usedBytes / (1024.0 * 1024.0));
    SLOG_PRINTF("FFat Free: %u bytes (%.2f MB)\n", freeBytes,
                  freeBytes / (1024.0 * 1024.0));
    oled.println("FFat OK");
  }
}

// Loads wifi.json (or runs the first-boot config portal if it doesn't
// exist yet), applies it, then loads every other *_config_load() group
// riding the same document (MQTT/limits/timing/battery/engine/time).
static void setup_wifi_config(void) {
  JsonDocument wifiDoc;
  bool wifiLoaded = wifi_config_load(wifiDoc);
  if (!wifiLoaded) {
    oled.clear();
    oled.println("WiFi config...");
    SLOG_PRINTLN("No wifi.json -> config portal");
    run_wifi_config_portal();
  } else {
    if (!wifi_config_apply(wifiDoc)) {
      SLOG_PRINTLN("wifi_config_apply failed");
      oled.println("WiFi apply fail");
      delay(500);
      ESP.restart();
    }
    mqtt_config_load(wifiDoc);
    SLOG_PRINTF("MQTT: %s host=%s:%u prefix=%s\n",
                  g_mqtt_config.enabled ? "enabled" : "disabled",
                  g_mqtt_config.host, g_mqtt_config.port,
                  g_mqtt_config.topic_prefix);
    limits_config_load(wifiDoc);
    SLOG_PRINTF("Limits (0=off): air=%.1fC oil=%.1fC engine=%.1fC inverter=%.1fC power=%.0fW auto_load_relay=%s choke_cold_start=%.1fC\n",
                  g_limits_config.temp_air_max, g_limits_config.temp_oil_max,
                  g_limits_config.temp_engine_max, g_limits_config.temp_inverter_max,
                  g_limits_config.power_max,
                  g_limits_config.auto_load_relay_on_run ? "on" : "off",
                  g_limits_config.choke_cold_start_temp);
    timing_config_load(wifiDoc);
    SLOG_PRINTF("Timing: max_start_try=%u start_check=%ums start=%ums start_retry=%ums stopping=%ums running=%ums load_relay_delay=%ums after_run=%ums\n",
                  g_timing_config.max_start_try, g_timing_config.start_time_check_ms,
                  g_timing_config.start_time_ms, g_timing_config.start_retry_time_ms,
                  g_timing_config.stopping_time_ms, g_timing_config.running_time_ms,
                  g_timing_config.load_relay_stop_delay_ms, g_timing_config.after_run_period_ms);
    battery_config_load(wifiDoc);
    SLOG_PRINTF("Battery: start_chg=%.1fV stop_chg=%.1fV starting_low=%.1fV shutdown=%.1fV shutdown_persist=%us start_charge_persist=%us periods(ms) idle=%u starting=%u running=%u after_run=%u charging=%u\n",
                  g_battery_config.threshold_start_charge_v, g_battery_config.threshold_stop_charge_v,
                  g_battery_config.threshold_starting_low_v, g_battery_config.threshold_shutdown_v,
                  g_battery_config.shutdown_persist_time_s, g_battery_config.start_charge_persist_time_s,
                  g_battery_config.read_period_ms, g_battery_config.read_starting_period_ms,
                  g_battery_config.read_running_period_ms, g_battery_config.read_after_run_period_ms,
                  g_battery_config.read_charging_period_ms);
    engine_config_load(wifiDoc);
    SLOG_PRINTF("Engine: rpm_max=%u rpm_starting=%u rpm_running=%u rpm_starting_thr=%u rpm_running_thr=%u fuel_pump_stop=%.1fL fuel_pump_no_rise_timeout=%us\n",
                  g_engine_config.rpm_max, g_engine_config.rpm_starting, g_engine_config.rpm_running,
                  g_engine_config.rpm_starting_threshold, g_engine_config.rpm_running_threshold,
                  g_engine_config.fuel_pump_stop_level, g_engine_config.fuel_pump_no_rise_timeout_s);
    time_config_load(wifiDoc);
    SLOG_PRINTF("Time: UTC offset %+d:%02u\n",
                  g_time_config.tz_offset_hours, g_time_config.tz_offset_minutes);
    espnow_config_load(wifiDoc);
    SLOG_PRINTF("ESP-NOW: %s\n", g_espnow_config.enabled ? "enabled" : "disabled");
  }
}

// Tachometer/RPM smoothing, INA233, ADC smoothing objects, MCP39F511A UART
// (ESP-IDF UART2) bring-up.
static void setup_powermeter_hw(void) {
  tacho_setup(); // PCNT hardware edge counter, no ISR needed

  rpm_smoothed.begin(RPM_SMOOTH_METHOD, RPM_SMOOTH);

  if (!IC1.begin()) {
    oled.clear();
    oled.println("INA233 fail");
    delay(500);
  }

  adc_1_0.begin(ADC1_SMOOTH_METHOD, ADC1_SMOOTH);
  adc_1_1.begin(ADC1_SMOOTH_METHOD, ADC1_SMOOTH);
  adc_1_2.begin(ADC1_SMOOTH_METHOD, ADC1_SMOOTH);
  adc_1_3.begin(ADC1_SMOOTH_METHOD, ADC1_SMOOTH);

  adc_2_0.begin(ADC2_SMOOTH_METHOD, ADC2_SMOOTH);
  adc_2_1.begin(ADC2_SMOOTH_METHOD, ADC2_SMOOTH);
  adc_2_2.begin(ADC2_SMOOTH_METHOD, ADC2_SMOOTH);

  if (!mcp39f511a_uart_begin()) {
    SLOG_PRINTLN("MCP39F511A: UART2 bring-up failed");
  }
}

// Boot-time MCP39F511A config/calibration/energy-counter read-and-dump —
// diagnostic only, compiled out entirely under POWERMON. The commented-out
// MCP39F511A_CONFIGURE block clones a known-good calibration onto a
// factory-default replacement chip; see its own comment before ever
// uncommenting it.
static void setup_mcp39f511a_debug_dump(void) {
  #ifndef POWERMON
  // Initialize and read MCP39F511A configuration
  delay(100); // Small delay for UART2/MCP to settle after bring-up
  if (mcp39f511a_load_config_from_flash()) {
    mcp39f511a_print_config();
    mcp39f511a_print_config_serial();
  } else {
    SDEBUG("MCP39F511A: Failed to read configuration at startup");
  }

  // Full calibration/configuration snapshot (all registers, not just
  // sysConfig/eventConfig) — read once here and dumped to serial so it can
  // be captured from a working chip. Kept separate from the above: this is
  // read once at boot, not periodically like mcp39f511a_read_config().
  // Addresses/field names verified directly against
  // doc/MCP39F511A-Data-Sheet-20006044A.pdf Table 6-1 — see mcp39f511a_dev.h's
  // header comment: the vendored MCP39F511A_UART.h's own addresses for
  // everything past the measurement registers turned out not to match the
  // real chip (e.g. it claimed System Configuration was at 0x007A; the
  // datasheet says 0x0094), so all config/calibration values captured
  // before this fix are unreliable and were discarded.
  if (mcp39f511a.readFullConfig()) {
    mcp39f511a_print_full_config_serial();

    // Known-good MCP39F511A full calibration/configuration, captured via
    // mcp39f511a_print_full_config_serial() above from the test rig
    // (ttyACM1) on 2026-07-25 — validated: sysConfig's UART field decoded
    // to 4 (9600 baud), matching MCP39F511A_UART_BAUD, confirming
    // the corrected register addresses/bitfield layout are right. Used to
    // clone this chip's configuration onto a factory-default replacement
    // chip if the currently-soldered one is ever swapped out. Commented
    // out; only uncomment (and re-flash) when an actual restore/clone is
    // genuinely needed.
    //
    // NOTE on ttyACM2: a second MCP39F511A is reachable standalone on
    // ttyACM2 via its own USB-UART adapter — this is a REFERENCE unit kept
    // for comparison and must NEVER be written to, only ever read from. Its
    // calibration/gain/offset registers match this snapshot bit-for-bit
    // (see doc/mcp39f511a-config-snapshot-2026-07-27.md), confirming the
    // corrected register addresses/bitfield layout. Its TEMPCOMP/PGA_CH0
    // values (below) differed from the 2026-07-25 capture and were adopted
    // here as the intended hardware setting for this design. Its event/limit
    // registers (voltageSagLimit through voltageHighThreshold, below) also
    // differed but were deliberately left disabled (0) — a product decision,
    // unrelated to the reference unit's own values.
    //
    // ENRG_CNTRL below is set to 1 (energy accumulation ON), unlike the
    // as-captured value (0, accumulation was off on this chip) — so
    // restoring this snapshot onto a replacement chip also starts energy
    // accumulation, rather than just cloning the as-found disabled state.
    #ifdef MCP39F511A_CONFIGURE
    {
      MCP39F511A_FullConfig known_good_full_cfg = {};

      // --- AC-mode calibration (this project runs on AC mains) ---
      known_good_full_cfg.acGainCurrentRms         = 0x84C3;  // gain factor, RMS current (~0x8000 = unity)
      known_good_full_cfg.acGainVoltageRms         = 0x9B07;  // gain factor, RMS voltage
      known_good_full_cfg.acGainActivePower        = 0x7465;  // gain factor, active power
      known_good_full_cfg.acGainReactivePower      = 0x6882;  // gain factor, reactive power
      known_good_full_cfg.acOffsetCurrentRms       = 6;       // small offset trim, RMS current
      known_good_full_cfg.acOffsetActivePower      = 0;       // offset trim, active power (none needed)
      known_good_full_cfg.acOffsetReactivePower    = 0;       // offset trim, reactive power (none needed)
      known_good_full_cfg.gainLineFrequency        = 0x8067;  // gain factor, line frequency
      known_good_full_cfg.phaseCompensation        = 172;     // phase compensation (AC mode)

      // --- EMI filter compensation (contact Microchip before hand-tuning) ---
      known_good_full_cfg.voltageDropComp          = 0x0000;  // unused on this hardware
      known_good_full_cfg.inCapCurrentComp         = 0x0000;  // unused on this hardware
      known_good_full_cfg.rangeVdropInCapComp      = 0x1C20;  // scale factors for the two above

      // --- DC-mode calibration (not exercised — this project is AC-only,
      //     captured anyway for a complete clone) ---
      known_good_full_cfg.dcGainCurrentRms         = 0x89AB;
      known_good_full_cfg.dcGainVoltageRms         = 0xB7DC;
      known_good_full_cfg.dcGainActivePower        = 0x8F1E;
      known_good_full_cfg.dcOffsetCurrentRms       = 0;
      known_good_full_cfg.dcOffsetActivePower      = 0;

      // --- ADC offset calibration (internal channel trim) ---
      known_good_full_cfg.offcalMsb                = 0xFFFF;
      known_good_full_cfg.offcalCh0                = 0x7D80;  // current channel offset
      known_good_full_cfg.offcalCh1                = 0xEC00;  // voltage channel offset

      // --- Temperature compensation coefficients (above/below calibration temp) ---
      known_good_full_cfg.tempPosCompFreq          = 0xCD00;
      known_good_full_cfg.tempNegCompFreq          = 0xDC00;
      known_good_full_cfg.tempPosCompCurrent       = 0x0BE0;
      known_good_full_cfg.tempNegCompCurrent       = 0x0200;
      known_good_full_cfg.tempPosCompPower         = 0x0A00;
      known_good_full_cfg.tempNegCompPower         = 0xFD00;

      // --- System Configuration (Register 6-2) — decoded field by field
      //     instead of one raw uDWord, so it's clear what's actually set ---
      known_good_full_cfg.sysConfig.ENRG_CNTRL      = 1;  // energy accumulation ON (see note above)
      known_good_full_cfg.sysConfig.PWM_CNTRL       = 0;  // PWM output off
      known_good_full_cfg.sysConfig.VREFEXT         = 0;  // internal voltage reference enabled
      known_good_full_cfg.sysConfig.SHUTDOWN        = 0;  // neither ADC in shutdown
      known_good_full_cfg.sysConfig.RESET           = 0;  // neither ADC in reset
      known_good_full_cfg.sysConfig.TEMPCOMP        = 1;  // temperature compensation enabled
      known_good_full_cfg.sysConfig.SINGLE_WIRE     = 0;  // single-wire UART disabled
      known_good_full_cfg.sysConfig.ZCD_OUTPUT_DIS  = 0;  // zero-crossing output pin enabled
      known_good_full_cfg.sysConfig.ZCD_PULS        = 0;  // ZCD changes level on crossing (not pulse mode)
      known_good_full_cfg.sysConfig.ZCD_INV         = 0;  // ZCD not inverted
      known_good_full_cfg.sysConfig.UART            = 4;  // 100b = 9600 baud (matches MCP39F511A_UART_BAUD)
      known_good_full_cfg.sysConfig.PGA_CH0         = 3;  // current channel PGA gain (per reference unit, ttyACM2 — was 0 in the 2026-07-25 capture)
      known_good_full_cfg.sysConfig.PGA_CH1         = 0;  // voltage channel PGA gain x1

      // --- Event Configuration (Register 7-1) — all zero: no simulated
      //     test events, no event latching, no pins mapped to Sag/Surge/
      //     Overcurrent/Overpower/Overtemp. Every field listed explicitly
      //     (rather than left at the struct's zero-init default) so this
      //     stays correct if the struct ever grows a non-zero-default field. ---
      known_good_full_cfg.eventConfig.OVERCUR_TST    = 0;
      known_good_full_cfg.eventConfig.OVERPOW_TST    = 0;
      known_good_full_cfg.eventConfig.VSAG_TST       = 0;
      known_good_full_cfg.eventConfig.VSUR_TST       = 0;
      known_good_full_cfg.eventConfig.OVERCUR_LA     = 0;
      known_good_full_cfg.eventConfig.OVERPOW_LA     = 0;
      known_good_full_cfg.eventConfig.VSAG_LA        = 0;
      known_good_full_cfg.eventConfig.VSUR_LA        = 0;
      known_good_full_cfg.eventConfig.VSAG_CL        = 0;
      known_good_full_cfg.eventConfig.VSUR_CL        = 0;
      known_good_full_cfg.eventConfig.OVERPOW_CL     = 0;
      known_good_full_cfg.eventConfig.OVERCUR_CL     = 0;
      known_good_full_cfg.eventConfig.EVENT1_MC      = 0;
      known_good_full_cfg.eventConfig.EVENT2_MC      = 0;
      known_good_full_cfg.eventConfig.VSAG_PIN1      = 0;
      known_good_full_cfg.eventConfig.VSURGE_PIN1    = 0;
      known_good_full_cfg.eventConfig.OVERCUR_PIN1   = 0;
      known_good_full_cfg.eventConfig.OVERPOW_PIN1   = 0;
      known_good_full_cfg.eventConfig.VSAG_PIN2      = 0;
      known_good_full_cfg.eventConfig.VSURGE_PIN2    = 0;
      known_good_full_cfg.eventConfig.OVERCUR_PIN2   = 0;
      known_good_full_cfg.eventConfig.OVERPOW_PIN2   = 0;
      known_good_full_cfg.eventConfig.OVERTEMP_TST   = 0;
      known_good_full_cfg.eventConfig.OVERTEMP_LA    = 0;
      known_good_full_cfg.eventConfig.OVERTEMP_CL    = 0;
      known_good_full_cfg.eventConfig.OVERTEMP_PIN1  = 0;
      known_good_full_cfg.eventConfig.OVERTEMP_PIN2  = 0;

      // --- Range (Register 6-3): per-byte right-shift scale factors for
      //     Energy/Power/Current/Voltage outputs — raw, no named bits ---
      known_good_full_cfg.range                    = 0x1B130C12UL;

      // --- Single-point calibration reference targets (same scale as the
      //     corresponding measurement register: current/1e4=A, voltage/10=V,
      //     power/1e3=W or VAR) ---
      known_good_full_cfg.calibrationCurrent       = 160000UL;  // 16.0000 A reference
      known_good_full_cfg.calibrationVoltage       = 2200;      // 220.0 V reference
      // Active/Reactive Power calibration targets: raw register units are
      // 0.01 W(or VAR)/LSB on this chip, not 0.001 (see mcp39f511a_dev.cpp's
      // readMeasurements() comment) — so these are 3520.00 W and 3048.40 VAR,
      // matching 220.0V*16.0000A at PF=1 and PF=0.5 respectively, exactly as
      // expected for the AC gain calibration procedure (datasheet Sec 9.3).
      known_good_full_cfg.calibrationActivePower   = 352000UL;  // 3520.00 W reference
      known_good_full_cfg.calibrationReactivePower = 304840UL;  // 3048.40 VAR reference

      known_good_full_cfg.apparentPowerDivisor     = 3;      // divisor digits for apparent power scaling
      known_good_full_cfg.accumulatorIntervalParameter = 2;  // 2^2 = 4 line cycles per computation cycle
      known_good_full_cfg.pwmPeriod                = 16128;  // inert: PWM_CNTRL=0 above (PWM output off)
      known_good_full_cfg.pwmDutyCycle             = 8192;   // inert: PWM_CNTRL=0 above
      known_good_full_cfg.minMaxPointer1           = 0;      // Min/Max tracking pointer 1 (unset)
      known_good_full_cfg.minMaxPointer2           = 0;      // Min/Max tracking pointer 2 (unset)
      known_good_full_cfg.lineFrequencyReference   = 50000;  // 50.000 Hz nominal line frequency
      known_good_full_cfg.thermistorVoltageCal     = 249;    // ambient-temp reference voltage for temp comp
      // Event/limit registers: briefly set to nonzero values 2026-07-27 after
      // comparing against a reference MCP39F511A unit on ttyACM2 (see
      // doc/mcp39f511a-config-snapshot-2026-07-27.md) — reverted back to
      // disabled (0) on 2026-07-27 per explicit decision, except
      // overTemperatureLimit which stays configured. eventConfig (below) is
      // still all-zero regardless, so none of these ever latched or mapped
      // to an EVENT pin — they only fed the chip's internal SystemStatus
      // flags, which nothing in this firmware polls.
      known_good_full_cfg.voltageSagLimit          = 0;      // event disabled
      known_good_full_cfg.voltageSurgeLimit        = 0;      // event disabled
      known_good_full_cfg.overCurrentLimit         = 0;      // event disabled
      known_good_full_cfg.overPowerLimit           = 0;      // event disabled
      known_good_full_cfg.overTemperatureLimit     = 383;    // raw thermistor ADC count (0-1023) — left configured
      known_good_full_cfg.voltageLowThreshold      = 0;      // save-to-EE low bracket disabled
      known_good_full_cfg.voltageHighThreshold     = 0;      // save-to-EE high bracket disabled
      known_good_full_cfg.noLoadThreshold          = 100;    // 1.00 W — matches datasheet default exactly

      // saveToFlash() writes the chip's internal flash, which has limited
      // write endurance — skip the write entirely if the chip already
      // matches (e.g. MCP39F511A_CONFIGURE left enabled across reboots).
      // readFullConfig() above already populated fullConfig_ with the live
      // config, so .valid is reliably true there — mirror that on our
      // side so memcmp() isn't comparing against a stale default.
      known_good_full_cfg.valid = true;
      MCP39F511A_FullConfig *current_cfg = mcp39f511a.getFullConfig();
      if (memcmp(current_cfg, &known_good_full_cfg, sizeof(MCP39F511A_FullConfig)) == 0) {
        SLOG_PRINTLN("MCP39F511A: configuration already matches known-good — skipping write");
      } else if (mcp39f511a.writeFullConfig(known_good_full_cfg)) {
        SLOG_PRINTLN("MCP39F511A: known-good full configuration restored and saved to flash");
      } else {
        SLOG_PRINTLN("MCP39F511A: FAILED to restore known-good full configuration");
      }
    }
    #endif  // MCP39F511A_CONFIGURE
  } else {
    SDEBUG("MCP39F511A: Failed to read full configuration at startup");
  }

  // Lifetime energy accumulators — read-only dump at boot. Resetting them
  // (Mcp39f511aDev::resetEnergyCounters()) is destructive and irreversible —
  // that's wired up as the explicit /energy/reset HTTP route instead, not
  // here at boot.
  if (mcp39f511a.readEnergyCounters()) {
    mcp39f511a_print_energy_counters_serial();
  } else {
    SDEBUG("MCP39F511A: Failed to read energy counters at startup");
  }
  #endif
}

// Logs every responding I2C address, flags INA233 presence (0x45).
static void setup_i2c_scan(void) {
  // I2C device scan — log all responding addresses, detect optional devices
  {
    SLOG_PRINTLN("I2C scan:");
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        SLOG_PRINTF("  ACK  0x%02X\n", addr);
        found++;
        if (addr == 0x45) ina233_present = true;
      }
    }
    if (!found) SLOG_PRINTLN("  no devices found!");
    SLOG_PRINTF("INA233 (0x45): %s\n", ina233_present ? "found" : "NOT FOUND");
    SLOG_PRINTLN("I2C scan done.");
  }
}

// MCP9600 ("KSEN" thermocouple amp) presence check + one-shot read.
static void setup_mcp9600(void) {
  tempSensor.begin(0x60);

  mcp9600_present = tempSensor.isConnected();
  if (mcp9600_present) {
    oled.clear();
    oled.println("KSEN ACK");
    tempSensor.checkDeviceID();
  } else {
    oled.clear();
    oled.println("KSEN NAK");
    delay(500);
  }
}

// ADS1115 #1 (temperature channels) init + initial smoothing-buffer fill.
static void setup_adc1(void) {
  if (!adc_1.init()) {
    SLOG_PRINT("ADS1115 No 1 not connected!");
    oled.clear();
    oled.println("ADC0 fail");
    delay(500);
  } else {
    // Rate/range/mode are fixed inside Ads1115Dev::init() — this is the
    // only configuration this project uses.
    for (uint8_t i = 0; i < ADC1_SMOOTH; i++) {
      adc_1_0.add(adc_temp(readChannel1(ADS1115_COMP_0_GND)));
      adc_1_1.add(adc_temp(readChannel1(ADS1115_COMP_1_GND)));
      adc_1_2.add(adc_temp(readChannel1(ADS1115_COMP_2_GND)));
      adc_1_3.add(adc_temp(readChannel1(ADS1115_COMP_3_GND)));
    }
  }
}

// ADS1115 #2 (fuel/voltage channels) init + initial smoothing-buffer fill.
static void setup_adc2(void) {
  if (!adc_2.init()) {
    SLOG_PRINT("ADS1115 No 2 not connected!");
    oled.clear();
    oled.println("ADC1 fail");
    delay(500);
  } else {
    // Rate/range/mode are fixed inside Ads1115Dev::init() — this is the
    // only configuration this project uses.
    for (uint8_t i = 0; i < ADC2_SMOOTH; i++) {
      adc_2_0.add(readChannel2(ADS1115_COMP_0_GND));
      adc_2_1.add(readChannel2(ADS1115_COMP_1_GND));
      adc_2_2.add(readChannel2(ADS1115_COMP_3_GND));
    }

    adc_bus_voltage = adc_2_2.get();
  }
}

// FM24V10 FRAM bring-up: presence check, journal init, boot event, and
// the two previous-run crash-capture reads (TWDT starvation marker +
// coredump panic summary) that only make sense once FRAM is up.
static void setup_fram(void) {
  int rv = fram.begin(0x56);

  if (rv != 0) {
    oled.clear();
    oled.println("NO FRAM");
    delay(500);
  } else {
    oled.clear();
    oled.println("FRAM at: 0x56");
  }

  if (rv == 0) {
    fram_log_begin(&fram);
    FramLogRecord boot_ev = {};
    boot_ev.type = FLOG_EV_BOOT;
    boot_ev.d.boot.reset_reason = (uint8_t)esp_reset_reason();
    fram_log_event(&boot_ev);

    // A TWDT panic on the previous run left task-starvation info in RTC
    // memory (captured by esp_task_wdt_isr_user_handler in fram_log.cpp) —
    // log it now that FRAM is up, then clear the marker.
    fram_log_check_wdt_capture();

    // If this boot's own reset reason indicates a crash, read whatever
    // ESP-IDF's coredump-to-flash already wrote (unsymbolized — see
    // decode_crash.sh/crash_archive/ for a real backtrace) and log it too.
    fram_log_check_panic_summary();
  }
}

// One-shot boot-time battery-voltage read for the adc_2_1 smoothing
// buffer (real runtime data, not just a display) — forces REG_BATT_FORCE
// briefly if BAT_SENSE isn't already on, same settle-and-release pattern as
// measure_bat_voltage() (statemachine.cpp) uses at runtime.
static void setup_battery_voltage(void) {
  float voltage_IC1 = 0;

  if (IS_REG_IN_ON(REG_IN_BAT_SENSE)) {
    IC1.getBusVoltage_V(voltage_IC1);
  } else {
    if (IS_REG_OFF(REG_BATT_FORCE)) {
      REG_ON(REG_BATT_FORCE);

      unsigned long long waitTime = CURRENT_TIME;

      while ((CURRENT_TIME - waitTime <= 1000) &&
      (IS_REG_IN_OFF(REG_IN_BAT_SENSE))) {
        read_regs();
        delay(1);
      };

      delay(200);

      if (IS_REG_IN_ON(REG_IN_BAT_SENSE)) {
        IC1.getBusVoltage_V(voltage_IC1);
      }

      REG_OFF(REG_BATT_FORCE);

      REG_ON(REG_BATT_OFF);

      waitTime = CURRENT_TIME;

      while ((CURRENT_TIME - waitTime <= 1000) &&
      (IS_REG_IN_ON(REG_IN_BAT_SENSE))) {
        read_regs();
        delay(1);
      };

      REG_OFF(REG_BATT_OFF);
    }
  }

  if (voltage_IC1 > 0.0) {
    for (uint8_t i = 0; i < ADC2_SMOOTH; i++) {
      adc_2_1.add(voltage_IC1);
    }
  }
}

// Starts the WiFi radio in whichever mode wifi_config_apply() selected —
// AP (mDNS only, already listening) or STA (asyncWifiMulti's own
// non-blocking scan-and-connect).
static void setup_start_wifi(void) {
  oled.clear();
  if (wifiModeIsAP) {
    oled.println("AP mode");
    MDNS.begin(wifiHostname.c_str());
    MDNS.addService("http", "tcp", 80);
  } else {
    oled.println("WiFi connecting...");
    // asyncWifiMulti.start() is fully non-blocking: disconnect → mode(STA) → async scan
    // On scan done it picks the best AP and calls WiFi.begin() automatically
    // The core's auto-reconnect retries back to back on NO_AP_FOUND, each
    // attempt sweeping every channel — with ESP-NOW enabled that keeps the
    // radio off the shared channel for the whole AP outage. loop()'s
    // AsyncWiFiMulti restart + espnow_wifi_supervise() reconnect instead.
    if (g_espnow_config.enabled) WiFi.setAutoReconnect(false);
    asyncWifiMulti.start();
  }
}

// Telnet (SDEBUG clients only — no MCP UART passthrough), WebSocket, and
// HTTP server bring-up — WebTask itself isn't created until setup_rtos()
// right after this.
static void setup_web_servers(void) {
  sserver.begin();
  sserver.setNoDelay(true);

  // wsServer's own onEvent()/begin() already ran inside its constructor
  // (WebSockets4WebServer) — setup_server_routes_ws() (web.cpp) is what
  // actually registers it onto webServer via addHook(), alongside every
  // other route. Not polled (wsServer.loop()) until WebTask exists (created
  // in setup_rtos(), right after this).
  setup_server_routes_ws();
  webServer.begin();
  gen_discovery_begin();
}

void setup(void) {
  // See report.cpp for the full rationale — must run before anything else
  // has a chance to crash.
  ota_reject_if_aborted();
  // One-shot 10 min soak only if still PENDING_VERIFY; ReportTask does the
  // actual esp_ota_mark_app_valid_cancel_rollback() when the timer fires.
  ota_arm_confirm_soak();

  // Same DEBUG_SERIAL_ENABLED flag also kills ESP-IDF/Arduino-core's own
  // ESP_LOG* output (see debug.h) — called first, before setup_early_hw()
  // even touches WiFi.disconnect(true), so the WiFi driver's own init
  // chatter is caught too. Does not silence the bootloader banner/earliest
  // cpu_start lines — those print before this function is ever reached.
  SLOG_SILENCE_IDF_LOGS();

  setup_early_hw();

  if (!IS_REG_IN_ON(REG_IN_JP1)) {
    return update_mode();
  }

  setup_rtc();

  setup_ffat();

  // Setup NTP BEFORE WiFi.begin() to enable DHCP NTP option
  setup_ntp();

  setup_wifi_config();

  setup_powermeter_hw();

  setup_mcp39f511a_debug_dump();

  setup_i2c_scan();

  setup_mcp9600();

  encoder_setup(); // PCNT hardware quadrature decoder + minimal button ISR

  setup_adc1();

  setup_adc2();

  setup_fram();

  setup_battery_voltage();

  setup_start_wifi();

  setup_web_servers();

  oled.println("ready");

  if (wifiModeIsAP)
    SLOG_PRINTF("AP ready. Connect to WiFi then http://%s.local or %s\n",
                  wifiHostname.c_str(), WiFi.softAPIP().toString().c_str());
  else
    SLOG_PRINTF("Ready! Open http://%s.local in your browser\n",
                  wifiHostname.c_str());

  REG_ON(REG_BUZZER);
  delay(10);
  REG_OFF(REG_BUZZER);
  delay(5);
  REG_ON(REG_BUZZER);
  delay(10);
  REG_OFF(REG_BUZZER);

  REG_OFF(REG_LED0);
  REG_OFF(REG_LED1);

  oled.clear();

  // Initialize FreeRTOS tasks
  setup_rtos();

  oled.println("RTOS running");
  vTaskDelay(pdMS_TO_TICKS(500));
  oled.clear();
}


// job_read_ina233_now()/read_ina233_now()/measure_bat_voltage()/
// choke_ensure_battery() moved to statemachine.cpp (2026-09-24, pass 1 of
// the StateMachineTask cut).

// battery_dependency_forced/fuel_pump_last_rise_ms/fuel_pump_last_rise_level
// and switch_STATE_*/process_STATE_*/switch_state()/process_current_state()/
// process_state_check()/sendOutputCommand()/sendSyncCommand()/
// StateMachineTask() moved to statemachine.cpp (2026-09-24, pass 2 of the
// StateMachineTask cut).


// ws_fmt_num()/ReportTask moved to report.cpp (2026-09-24).

// MQTT config load/callback/MqttTask moved to mqtt.cpp (2026-09-24).

// limits_config_load()/timing_config_load()/battery_config_load()/
// engine_config_load()/time_config_load() moved to config.cpp (2026-09-24).

// state_name() moved to statemachine.cpp (2026-09-24, pass 2).

// ----------------------------------------------------------------------------
// WebTask moved to web.cpp (2026-09-24).

// ----------------------------------------------------------------------------
// setup_rtos: Initialize all RTOS primitives and create tasks
// ----------------------------------------------------------------------------
void setup_rtos() {
    SLOG_PRINTLN("Initializing FreeRTOS...");

    // Initialize Task Watchdog Timer with 15 second timeout
    // Default is 5 seconds which can trigger during large file transfers from FFat
    // Don't watch idle tasks (idle_core_mask = 0) - they get blocked during file I/O
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 15000,
        .idle_core_mask = 0,  // Don't watch idle tasks
        .trigger_panic = true
    };
    esp_err_t wdt_err = esp_task_wdt_reconfigure(&wdt_config);
    if (wdt_err == ESP_OK) {
        SLOG_PRINTLN("WDT configured (15s timeout, no idle watch)");
    } else {
        SLOG_PRINTF("WDT reconfigure failed: %s\n", esp_err_to_name(wdt_err));
    }

    // Create mutexes
    xFlashI2CGuard = xSemaphoreCreateMutex();
    xSerial2Mutex = xSemaphoreCreateMutex();
    // Recursive: reg_write()/reg_read() take it internally and ShiftRegTask
    // calls them while already holding the mutex.
    xShiftRegMutex = xSemaphoreCreateRecursiveMutex();
    xStateMutex = xSemaphoreCreateMutex();
    xWsMutex = xSemaphoreCreateMutex();

    if (xFlashI2CGuard == NULL || xSerial2Mutex == NULL ||
        xShiftRegMutex == NULL || xStateMutex == NULL ||
        xWsMutex == NULL) {
        SLOG_PRINTLN("ERROR: Failed to create mutexes!");
        return;
    }
    SLOG_PRINTLN("Mutexes created");

    // Create queues
    xPowerDataQueue = xQueueCreate(1, sizeof(PowerData_t));
    xOutputCommandQueue = xQueueCreate(QUEUE_SIZE_OUTPUT_COMMAND, sizeof(OutputCommand_t));
    xWebCommandQueue = xQueueCreate(QUEUE_SIZE_WEB_COMMAND, sizeof(WebCommand_t));
    xBuzzerCommandQueue = xQueueCreate(QUEUE_SIZE_BUZZER_COMMAND, sizeof(BuzzerCommand_t));
    xI2CJobQueue = xQueueCreate(QUEUE_SIZE_I2C_JOB, sizeof(I2CJob_t));
    xMenuRenderQueue = xQueueCreate(1, oled_menu_state_size());

    if (xPowerDataQueue == NULL ||
        xOutputCommandQueue == NULL || xWebCommandQueue == NULL ||
        xBuzzerCommandQueue == NULL || xI2CJobQueue == NULL ||
        xMenuRenderQueue == NULL) {
        SLOG_PRINTLN("ERROR: Failed to create queues!");
        return;
    }
    SLOG_PRINTLN("Queues created");

    // HTTP/WS handlers enqueue onto xWebCommandQueue once this is true.
    // Must flip BEFORE WebTask is created: WebTask begins polling
    // handleClient() the instant xTaskCreate returns, and the pre-RTOS
    // fallback in /reg used to call reg_set() with no REG_START check.
    // Found 2026-09-28. Defense in depth: that fallback also refuses
    // REG_START now (web.cpp).
    g_rtos_running = true;

    // Create event group
    xStateEvents = xEventGroupCreate();
    if (xStateEvents == NULL) {
        SLOG_PRINTLN("ERROR: Failed to create event group!");
        return;
    }
    SLOG_PRINTLN("Event group created");

    // Initialize shared state
    g_state.current_state = STATE_INIT;
    g_state.previous_state = STATE_INIT;
    g_state.state_entry_time = millis();
    g_state.shift_reg_inputs = shift_reg_in;
    g_state.shift_reg_outputs = shift_reg;

    // Create tasks pinned to specific cores
    BaseType_t result;

    result = xTaskCreatePinnedToCore(
        StateMachineTask,
        "StateMachine",
        STACK_SIZE_STATE_MACHINE,
        NULL,
        TASK_PRIORITY_STATE_MACHINE,
        &xStateMachineTask,
        CORE_STATE_MACHINE
    );
    if (result != pdPASS) SLOG_PRINTLN("ERROR: StateMachineTask creation failed!");

    result = xTaskCreatePinnedToCore(
        ShiftRegTask,
        "ShiftReg",
        STACK_SIZE_SHIFT_REG,
        NULL,
        TASK_PRIORITY_SHIFT_REG,
        &xShiftRegTask,
        CORE_SHIFT_REG
    );
    if (result != pdPASS) SLOG_PRINTLN("ERROR: ShiftRegTask creation failed!");

    // SensorTask retired — its body is now i2c_read_sensors(), called from
    // I2CTask (created below), the sole remaining owner of Wire.

    result = xTaskCreatePinnedToCore(
        PowerMeterTask,
        "PowerMeter",
        STACK_SIZE_POWER_METER,
        NULL,
        TASK_PRIORITY_POWER_METER,
        &xPowerMeterTask,
        CORE_POWER_METER
    );
    if (result != pdPASS) SLOG_PRINTLN("ERROR: PowerMeterTask creation failed!");

    result = xTaskCreatePinnedToCore(
        DisplayTask,
        "Display",
        STACK_SIZE_DISPLAY,
        NULL,
        TASK_PRIORITY_DISPLAY,
        &xDisplayTask,
        CORE_DISPLAY
    );
    if (result != pdPASS) SLOG_PRINTLN("ERROR: DisplayTask creation failed!");

    result = xTaskCreatePinnedToCore(
        ReportTask,
        "Report",
        STACK_SIZE_REPORT,
        NULL,
        TASK_PRIORITY_REPORT,
        &xReportTask,
        CORE_REPORT
    );
    if (result != pdPASS) SLOG_PRINTLN("ERROR: ReportTask creation failed!");

    result = xTaskCreatePinnedToCore(
        MqttTask,
        "Mqtt",
        STACK_SIZE_MQTT,
        NULL,
        TASK_PRIORITY_MQTT,
        &xMqttTask,
        CORE_MQTT
    );
    if (result != pdPASS) SLOG_PRINTLN("ERROR: MqttTask creation failed!");

    result = xTaskCreatePinnedToCore(
        WebTask,
        "Web",
        STACK_SIZE_WEB,
        NULL,
        TASK_PRIORITY_WEB,
        &xWebTask,
        CORE_WEB
    );
    if (result != pdPASS) SLOG_PRINTLN("ERROR: WebTask creation failed!");

    // FRAM event log writer (owns all FRAM writes + metrics accumulation) —
    // must run before I2CTask is created, not after: I2CTask calls
    // fram_log_service_tick() on every tick, which only guards on
    // s_present (true since setup_fram()'s earlier fram_log_begin() call)
    // but not on xLogQueue itself — a real, always-present race, just
    // never lost before because boot-time Serial output happened to delay
    // this point long enough for I2CTask (pinned to its own core, running
    // truly in parallel from the moment it's created) to never get there
    // first. Found live 2026-09-25 with DEBUG_SERIAL_ENABLED=0's much
    // faster boot: I2CTask's very first tick reliably won the race,
    // xQueueReceive(NULL, ...) asserted every reboot, and — since the
    // crash happens well before ReportTask's 10-minute OTA-confirm mark —
    // the image was never marked valid, yet nothing rolled it back either
    // (CONFIG_APP_ROLLBACK_ENABLE isn't set for this project, see
    // idf.py-build-details in this file's own header). fram_log_service_tick()
    // itself also gained a defensive xLogQueue!=NULL guard (fram_log.cpp)
    // so a future ordering regression degrades instead of crash-looping.
    fram_log_start_task();

    result = xTaskCreatePinnedToCore(
        I2CTask,
        "I2C",
        STACK_SIZE_I2C,
        NULL,
        TASK_PRIORITY_I2C,
        &xI2CTask,
        CORE_I2C
    );
    if (result != pdPASS) SLOG_PRINTLN("ERROR: I2CTask creation failed!");

    result = xTaskCreatePinnedToCore(
        EspNowTask,
        "EspNow",
        STACK_SIZE_ESPNOW,
        NULL,
        TASK_PRIORITY_ESPNOW,
        &xEspNowTask,
        CORE_ESPNOW
    );
    if (result != pdPASS) SLOG_PRINTLN("ERROR: EspNowTask creation failed!");

    SLOG_PRINTLN("All RTOS tasks created");

    // Worker tasks self-register in the WDT (esp_task_wdt_add(NULL) at the top
    // of each task) — registering them here raced their first wdt_reset and
    // spammed "task not found" at boot.

    // Add Arduino loopTask (current task) to watchdog
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());

    SLOG_PRINTLN("FreeRTOS initialization complete");
}

//================================================================

// Accept telnet clients for SDEBUG() output only. Former Serial2↔telnet
// MCP passthrough removed 2026-09-29 (UART2 owned exclusively by the
// ESP-IDF MCP driver / PowerMeterTask).
void loop_serial(void) {
  uint8_t i;
  if (sserver.hasClient()) {
    for (i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (!serverClients[i] || !serverClients[i].connected()) {
        if (serverClients[i])
          serverClients[i].stop();
        serverClients[i] = sserver.accept();
        break;
      }
    }
    if (i >= MAX_SRV_CLIENTS) {
      sserver.accept().stop();
    }
  }
  for (i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (serverClients[i] && !serverClients[i].connected()) {
      serverClients[i].stop();
    }
  }
}

void loop(void) {
  // ============================================================================
  // Simplified loop - most work is now handled by FreeRTOS tasks
  // ============================================================================

  // Feed watchdog
  esp_task_wdt_reset();

  // STA mode: asyncWifiMulti handles connection; restart it when Idle (disconnected/failed).
  // AsyncWiFiMulti has no auto-retry of its own (confirmed by reading its
  // source — onFailure()/a post-connect drop both just leave it at Idle
  // forever), so this is the only thing that ever calls start() again after
  // a failure. Retrying on a fixed 10s cadence hammers WiFi.disconnect() +
  // WiFi.mode() + scanNetworks() indefinitely against a flapping/unreachable
  // AP without giving the driver time to settle between attempts — back off
  // exponentially (10s, 20s, 40s, 80s, 160s, capped at 5 min) instead, and
  // reset to 10s the moment a connection attempt makes any progress.
  if (!wifiModeIsAP) {
    static unsigned long lastWifiCheck = 0;
    static uint8_t wifiRetryBackoff = 0;
    bool multiIdle = asyncWifiMulti.status() == GuLinux::AsyncWiFiMulti::Idle;
    bool apBack = espnow_wifi_supervise(WiFi.status() == WL_CONNECTED, multiIdle);
    if (multiIdle && apBack) {
      SLOG_PRINTLN("WiFi: AP seen on home channel — reconnecting on that channel");
      lastWifiCheck = CURRENT_TIME;
      wifiRetryBackoff = 0;
      // Single-channel scan: a full one would take the radio off the
      // ESP-NOW channel for seconds right as the link is recovering.
      asyncWifiMulti.start(espnow_home_channel());
    } else if (multiIdle && !espnow_wifi_probe_running()) {
      unsigned long backoffMs = 10000UL << (wifiRetryBackoff < 5 ? wifiRetryBackoff : 5);
      if (backoffMs > 300000UL) backoffMs = 300000UL;
      if (CURRENT_TIME - lastWifiCheck >= backoffMs) {
        lastWifiCheck = CURRENT_TIME;
        if (wifiRetryBackoff < 5) wifiRetryBackoff++;
        SLOG_PRINTF("WiFi: AsyncWiFiMulti idle — restarting scan (next backoff %lus)\n",
                      backoffMs / 1000);
        asyncWifiMulti.start();
      }
    } else if (!multiIdle) {
      lastWifiCheck = CURRENT_TIME;  // reset so retry starts fresh after next disconnect
      wifiRetryBackoff = 0;
    }
  }

#if defined(DEBUG)
  loop_serial();
#endif

  // Handle rotary encoder (interrupt-based but needs polling)
  rotary_loop();

  // Periodic RTC read moved into I2CTask's i2c_service_rtc() (was here for
  // no reason beyond historical inertia — predates the FreeRTOS task
  // split). The old FLOG_EV_I2C_STUCK detection that lived here (polling
  // TAKE_MUTEX(xFlashI2CGuard) once a second) is replaced by a cache-
  // staleness check in ReportTask — see there.
  //
  // NTP-triggered adjustment stays here, though — deciding whether a sync
  // is worth journaling is ordinary housekeeping, not I2C-owner code. Only
  // the actual RTC write (rtc_adjust_now(), a thin wrapper over
  // i2c_run_on_owner()) touches I2CTask.
  if (ntp_sync_flag) {
    ntp_sync_flag = false; // Clear flag first
    if (ntp_sync_time > 0) {
      SLOG_PRINTLN("Got time adjustment from NTP!");
      uint32_t old_ts = rtc_now_unixtime();
      if (rtc_adjust_now(ntp_sync_time)) {
        SLOG_PRINTF("RTC updated to: %ld\n", ntp_sync_time);
        // Journal the correction so pre-sync records can be re-dated
        if ((uint32_t)ntp_sync_time > old_ts + 2 || old_ts > (uint32_t)ntp_sync_time + 2) {
          FramLogRecord r = {};
          r.type = FLOG_EV_TIME_SYNC;
          r.d.time_sync.old_ts = old_ts;
          r.d.time_sync.new_ts = (uint32_t)ntp_sync_time;
          fram_log_event(&r);
        }
      }
    }
  }

  // Give time to other tasks
  vTaskDelay(pdMS_TO_TICKS(10));
}

