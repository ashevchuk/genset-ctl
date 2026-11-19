/*
 * rtos_config.h
 * FreeRTOS Configuration for ESP32 Generator Control
 *
 * Defines task priorities, stack sizes, synchronization primitives,
 * and shared data structures for multi-tasking architecture.
 */

#ifndef RTOS_CONFIG_H
#define RTOS_CONFIG_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>

// FLOG_MAINT_TYPES — sizes LimitsConfig_t::maint_interval_hours below. This
// is the one place rtos_config.h reaches outside its own generic RTOS/queue
// scope; fram_log.h itself stays config-layer-agnostic (see its own
// fram_metrics_json() comment) — the dependency only runs this direction.
#include "fram_log.h"

// ============================================================================
// Task Priorities (higher number = higher priority)
// ============================================================================
#define TASK_PRIORITY_STATE_MACHINE   5   // Highest - generator safety critical
#define TASK_PRIORITY_SHIFT_REG       6   // GPIO relay control
#define TASK_PRIORITY_POWER_METER     2   // MCP39F511A UART
#define TASK_PRIORITY_DISPLAY         2   // OLED updates
#define TASK_PRIORITY_REPORT          2   // WebSocket broadcast
#define TASK_PRIORITY_MQTT            2   // MQTT client
#define TASK_PRIORITY_WEB             2   // WebServer::handleClient() + WebSocketsServer::loop()
#define TASK_PRIORITY_I2C             2   // Sole owner of Wire — sensors, RTC, OLED, FRAM
#define TASK_PRIORITY_ESPNOW          2   // ESP-NOW fallback link to the display

// ============================================================================
// Task Stack Sizes (in bytes)
// ============================================================================
#define STACK_SIZE_STATE_MACHINE   (4 * 1024)
#define STACK_SIZE_SHIFT_REG       (2 * 1024)
#define STACK_SIZE_POWER_METER     (3 * 1024)
#define STACK_SIZE_DISPLAY         (6 * 1024)
#define STACK_SIZE_REPORT          (4 * 1024)
#define STACK_SIZE_MQTT            (5 * 1024)
// Generous: this task also does the manual chunked-read/upload loops
// in-task (see serve_static_ws()), instead of via an async callback.
#define STACK_SIZE_WEB             (8 * 1024)
// Generous: renders the OLED, reads all I2C sensors/RTC, and services FRAM
// journal writes/reads — everything that used to be split across
// SensorTask/DisplayTask's render half/LogTask.
#define STACK_SIZE_I2C             (8 * 1024)
#define STACK_SIZE_ESPNOW          (3 * 1024)

// ============================================================================
// Task Periods (in milliseconds)
// ============================================================================
#define PERIOD_SHIFT_REG_MS        100
#define PERIOD_POWER_METER_MS      1000
#define PERIOD_DISPLAY_MS          50    // encoder poll rate; OLED renders every 4th tick (200 ms)
#define PERIOD_REPORT_MS           1000
// Base tick for I2CTask's multi-rate loop (matches PERIOD_DISPLAY_MS) —
// render every 4th tick (200ms), sensors every 10th (500ms), RTC every 20th (1000ms).
#define PERIOD_I2C_MS              50

// ============================================================================
// Core Assignments
// ============================================================================
#define CORE_STATE_MACHINE         0
#define CORE_SHIFT_REG             0
#define CORE_POWER_METER           1
#define CORE_DISPLAY               1
#define CORE_REPORT                0   // WebSocket on Core 0 with WiFi
#define CORE_MQTT                  0   // MQTT on Core 0 with WiFi
#define CORE_I2C                   1   // Same core as today's Sensor/Display/PowerMeter
#define CORE_WEB                   0   // WebServer/WebSocketsServer on Core 0 with WiFi
#define CORE_ESPNOW                0   // Shares the WiFi driver with Report/Mqtt/Web

// ============================================================================
// Mutex Timeout (milliseconds)
// ============================================================================
#define MUTEX_TIMEOUT_MS           100
#define MUTEX_TIMEOUT_TICKS        pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)

// ============================================================================
// Queue Sizes
// ============================================================================
#define QUEUE_SIZE_OUTPUT_COMMAND  10
#define QUEUE_SIZE_WEB_COMMAND     10
#define QUEUE_SIZE_BUZZER_COMMAND  5
#define QUEUE_SIZE_I2C_JOB         4

// ============================================================================
// Event Group Bits
// ============================================================================
#define EVENT_SENSOR_UPDATE        (1 << 0)
#define EVENT_POWER_UPDATE         (1 << 1)
#define EVENT_SHIFT_REG_UPDATE     (1 << 2)
#define EVENT_START_REQUESTED      (1 << 3)
#define EVENT_STOP_REQUESTED       (1 << 4)
#define EVENT_STATE_CHANGED        (1 << 5)

// ============================================================================
// Data Structures
// ============================================================================

// Sensor data from I2C devices (ADS1115, INA233, MCP9600)
typedef struct {
    // ADS1115 #1 (temperature sensors)
    float adc1_ch0;      // Temperature 0
    float adc1_ch1;      // Temperature 1
    float adc1_ch2;      // Temperature 2
    float adc1_ch3;      // Temperature 3

    // ADS1115 #2 (voltage/fuel sensors)
    float adc2_ch0;      // Fuel level
    float adc2_ch1;      // Battery voltage (from INA233 or ADC)
    float adc2_ch2;      // Bus voltage reference

    // INA233 battery monitor
    float bat_voltage;
    float bat_current;

    // MCP9600 thermocouple
    float head_temp;
    bool  head_temp_valid;  // false if absent or the last read attempt failed

    // Tachometer
    uint32_t rpm;
    float frequency;

    // Metadata
    uint32_t timestamp;
    bool valid;
} SensorData_t;

// Power meter data from MCP39F511A
typedef struct {
    float rms_voltage;
    float rms_current;
    float active_power;
    float reactive_power;
    float apparent_power;
    float frequency;
    float power_factor;
    // Lifetime energy accumulators, 1 mWh/LSB (datasheet default resolution).
    // Read less often than the measurements above (see PowerMeterTask) —
    // energy_valid tracks whether these 4 fields hold a real reading,
    // independent of `valid` (which covers the measurement fields only).
    uint64_t import_active_energy_mwh;
    uint64_t export_active_energy_mwh;
    uint64_t import_reactive_energy_mwh;
    uint64_t export_reactive_energy_mwh;
    bool energy_valid;
    uint32_t timestamp;
    bool valid;
} PowerData_t;

// Generator state machine states. Moved here from the main .cpp (2026-09-24,
// during the OLED-menu extraction) so a separate translation unit
// (oled_menu.cpp) can declare `state` locals and read g_state below —
// STATE_ERROR is kept last/max deliberately, see CLAUDE.md's state-machine
// section for why (state_id in MQTT/HTTP JSON).
typedef enum state {
  STATE_INIT = 0,
  STATE_IDLE,
  STATE_STOPPING_LOAD_OFF,
  STATE_STOPPING,
  STATE_STOPPING_RETRY,
  STATE_STOPPED,
  STATE_STOPPED_RETRY,
  STATE_STARTING_SINGLE,
  STATE_STARTING,
  STATE_STARTING_RETRY,
  STATE_STARTING_RETRY_TRY,
  STATE_STARTED,
  STATE_RUNNING,
  STATE_ERROR
} state;

// Shared state/sensor cache — refreshed by I2CTask/PowerMeterTask/
// StateMachineTask, read by everyone else (ReportTask, WebTask, MqttTask,
// oled_menu.cpp's menu screens) under g_state_spinlock (see the two extern
// declarations further down). Moved here alongside `state` for the same
// cross-translation-unit reason.
typedef struct {
    volatile state current_state;
    volatile state previous_state;
    volatile uint32_t state_entry_time;

    // Latest sensor data
    SensorData_t sensors;

    // Latest power meter data
    PowerData_t power;

    // Shift register state
    volatile uint8_t shift_reg_inputs;
    volatile uint16_t shift_reg_outputs;

    // Timing variables
    volatile uint32_t start_time;
    volatile uint32_t start_retry_time;
    volatile uint32_t stopping_time;
    volatile uint32_t running_time;
    volatile uint8_t start_try;

    // Flags
    volatile bool charging;
    volatile bool after_run_period;
    volatile uint32_t after_run_period_time;
    volatile uint32_t read_bat_period;
    volatile uint32_t read_bat_time;
} SharedState_t;

// Configurable safety/maintenance limits (/wifi-config). Moved here (same
// reason as SharedState_t/state above) since oled_menu.cpp's MENU_MAINT
// screen reads maint_interval_hours directly.
struct LimitsConfig_t {
    float temp_air_max;
    float temp_oil_max;
    float temp_engine_max;
    float temp_inverter_max;
    float power_max;              // active power, W — 0 disables
    bool  auto_load_relay_on_run; // force REG_LOAD_RELAY on when entering STATE_RUNNING
    float choke_cold_start_temp;  // engine head temp (°C, may be negative) below which choke engages during start
    // Engine-hours interval per FLOG_MAINT_* subtype (see fram_log.h) after
    // which that maintenance item is considered overdue — compared against
    // FramMetrics.maint_seconds/[i] via fram_metrics_json()'s "due" field
    // and ReportTask's MAINT_DUE dashboard banner. 0 disables that type.
    float maint_interval_hours[FLOG_MAINT_TYPES];
};

// Output shift register (SN74HC595N) bit assignments and input shift
// register (CD4021BE) bit assignments. Moved here (same 2026-09-24 reason as
// state/SharedState_t/LimitsConfig_t above) — oled_menu.cpp's menu screens
// read REG_BATT_OFF/REG_FUEL_PUMP/REG_IN_OIL/etc. directly.
enum SHIFT_REG {
  REG_CHOKE = 0,
  REG_FUEL_PUMP = 1,
  REG_BUZZER = 2,
  REG_RESERVED_3 = 3,
  REG_RESERVED_4 = 4,
  REG_RESERVED_5 = 5,
  REG_RESERVED_6 = 6,
  REG_LED0 = 7,

  REG_BATT_FORCE = 8,
  REG_BATT_OFF = 9,
  REG_IGNITION = 10,
  REG_START = 11,
  REG_ECO_MODE = 12,
  REG_LOAD_RELAY = 13,
  REG_CHARGER = 14,
  REG_LED1 = 15,
};

enum SHIFT_REG_IN {
  REG_IN_OVERLOAD = 0,
  REG_IN_OIL = 1,
  REG_IN_OUTPUT = 2,

  REG_IN_OVERHEAT = 3,

  REG_IN_PSU_SENSE = 4,
  REG_IN_GEN_SENSE = 5,
  REG_IN_BAT_SENSE = 6,
  REG_IN_JP1 = 7,
};

// Command types for output control
typedef enum {
    CMD_REG_SET,         // Set specific register state
    CMD_REG_ON,          // Turn register on
    CMD_REG_OFF,         // Turn register off
    CMD_REG_TOGGLE,      // Toggle register
    CMD_REG_SYNC         // Sync all register states
} OutputCommandType_t;

// Output command for shift register task
typedef struct {
    OutputCommandType_t type;
    uint8_t reg;         // Register number (0-15)
    bool state;          // Desired state (for CMD_REG_SET)
    uint16_t full_state; // Full 16-bit state (for CMD_REG_SYNC)
} OutputCommand_t;

// Web command types
typedef enum {
    WEB_CMD_START,
    WEB_CMD_START_RETRY,
    WEB_CMD_STOP,
    WEB_CMD_REG_TOGGLE,
    WEB_CMD_REG_SET      // Set register to specific state (MQTT)
} WebCommandType_t;

// Web command from WebSocket / MQTT
typedef struct {
    WebCommandType_t type;
    uint8_t reg;         // For REG_TOGGLE / REG_SET command
    bool state;          // For REG_SET command: desired state
    uint8_t source;      // FLOG_SRC_* — who issued the command (for event log)
} WebCommand_t;

// Generic synchronous job for I2CTask (the sole owner of Wire). A caller on
// any other task builds one of these (with its own reusable static
// SemaphoreHandle_t for `done`), sends it to xI2CJobQueue, and blocks on
// `done` — see i2c_run_on_owner() in the main .ino. Used for the handful of
// I2C operations that need a real synchronous answer (FRAM reads/wipe/set-
// hours, an on-demand INA233 read for measure_bat_voltage()) rather than a
// passively-refreshed cache.
//
// No `ok`/result field here on purpose: xQueueSend()/xQueueReceive() copy
// this struct BY VALUE, so I2CTask only ever sees its own private copy —
// writing a result into that copy can never be observed by the original
// caller's copy back in i2c_run_on_owner(). (An earlier version of this
// struct had exactly that field and silently always reported failure —
// found via /log always returning an empty event list; see fram_log.cpp's
// job functions for how success/failure is actually reported now: each job
// writes its own outcome into its own static storage, e.g. `arg`'s struct,
// which i2c_run_on_owner()'s caller reads directly after a successful wait.)
typedef struct {
    bool (*fn)(void *arg);
    void *arg;
    SemaphoreHandle_t done;
} I2CJob_t;

// ============================================================================
// Buzzer Control Structures
// ============================================================================

// Buzzer command types
typedef enum {
    BUZZER_CMD_BEEP,     // Finite beep pattern
    BUZZER_CMD_STOP      // Stop buzzer immediately
} BuzzerCommandType_t;

// Buzzer command structure
typedef struct {
    BuzzerCommandType_t type;
    uint16_t duration_ms;   // Duration of each beep (ms)
    uint16_t pause_ms;      // Pause between beeps (ms)
    uint8_t count;          // Number of beeps (0 = infinite)
} BuzzerCommand_t;

// Buzzer state (used internally by ShiftRegTask)
typedef struct {
    bool active;            // Buzzer pattern is running
    bool buzzer_on;         // Current buzzer output state
    uint16_t duration_ms;   // Duration of each beep
    uint16_t pause_ms;      // Pause between beeps
    uint8_t count;          // Remaining beeps (0 = infinite)
    uint8_t total_count;    // Total beeps requested (0 = infinite)
    uint32_t next_toggle;   // Timestamp for next state change
} BuzzerState_t;

// ============================================================================
// External Handle Declarations (defined in main .ino file)
// ============================================================================

// Mutexes
// xFlashI2CGuard: NOT about arbitrating logical I2C ownership (I2CTask is
// the sole owner of Wire — see I2CJob_t above). Guards a distinct, already-
// documented ESP32 hardware hazard instead: concurrent SPI-flash operations
// (FFat reads in WebTask's serve_static_ws()/uploads/OTA) and I2C
// transactions can collide via spi_flash_guard_start() briefly disabling
// cache on both cores. Taken by WebTask around its FFat chunk reads and by
// I2CTask around its own real Wire transactions — the two participants on
// either side of that specific collision, nothing else.
extern SemaphoreHandle_t xFlashI2CGuard;
extern SemaphoreHandle_t xSerial2Mutex;
extern SemaphoreHandle_t xShiftRegMutex;
extern SemaphoreHandle_t xStateMutex;
extern SemaphoreHandle_t xWsMutex;

// Queues
extern QueueHandle_t xPowerDataQueue;
extern QueueHandle_t xOutputCommandQueue;
extern QueueHandle_t xWebCommandQueue;
extern QueueHandle_t xBuzzerCommandQueue;
extern QueueHandle_t xI2CJobQueue;
extern QueueHandle_t xMenuRenderQueue;

// Defined in esp32-gen-ctl-ai-rtos.ino, next to I2CTask — see its doc
// comment. Declared here so fram_log.cpp can use it too.
bool i2c_run_on_owner(bool (*fn)(void *), void *arg, SemaphoreHandle_t done_sem, TickType_t timeout);

// Shared state cache + safety/maintenance limits, both defined (non-static)
// in esp32-gen-ctl-ai-rtos.cpp. Exposed here (2026-09-24, OLED-menu
// extraction) so oled_menu.cpp can read them too — always under
// g_state_spinlock for g_state, per the existing portENTER_CRITICAL/
// portEXIT_CRITICAL convention every reader already follows.
extern SharedState_t g_state;
extern portMUX_TYPE g_state_spinlock;
extern LimitsConfig_t g_limits_config;

// Also defined (non-static) in esp32-gen-ctl-ai-rtos.cpp, exposed for the
// same reason as g_state/g_limits_config above.
const char *state_name(state s);

// Human-readable per-FLOG_MAINT_* labels, shared by ReportTask's MAINT_DUE
// banner (main .cpp) and oled_menu.cpp's MENU_MAINT screen.
extern const char *MAINT_TYPE_LABELS[FLOG_MAINT_TYPES];

// Event Group
extern EventGroupHandle_t xStateEvents;

// Task Handles
extern TaskHandle_t xStateMachineTask;
extern TaskHandle_t xShiftRegTask;
extern TaskHandle_t xPowerMeterTask;
extern TaskHandle_t xDisplayTask;
extern TaskHandle_t xReportTask;
extern TaskHandle_t xMqttTask;
extern TaskHandle_t xWebTask;
extern TaskHandle_t xI2CTask;
extern TaskHandle_t xEspNowTask;

// ============================================================================
// Helper Macros
// ============================================================================

// Take mutex with timeout, returns true if acquired
#define TAKE_MUTEX(mutex) (xSemaphoreTake((mutex), MUTEX_TIMEOUT_TICKS) == pdTRUE)

// Give mutex
#define GIVE_MUTEX(mutex) xSemaphoreGive(mutex)

// Take mutex from ISR context
#define TAKE_MUTEX_ISR(mutex, pxHigherPriorityTaskWoken) \
    (xSemaphoreTakeFromISR((mutex), (pxHigherPriorityTaskWoken)) == pdTRUE)

// Give mutex from ISR context
#define GIVE_MUTEX_ISR(mutex, pxHigherPriorityTaskWoken) \
    xSemaphoreGiveFromISR((mutex), (pxHigherPriorityTaskWoken))

// Send to queue from ISR
#define QUEUE_SEND_ISR(queue, item, pxHigherPriorityTaskWoken) \
    xQueueSendFromISR((queue), (item), (pxHigherPriorityTaskWoken))

// Overwrite queue (for latest-data-only queues)
#define QUEUE_OVERWRITE(queue, item) xQueueOverwrite((queue), (item))

#endif // RTOS_CONFIG_H
