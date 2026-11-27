/*
 * shiftreg.cpp — physical shift-register I/O.
 *
 * Extracted 2026-09-24 from esp32-gen-ctl-ai-rtos.cpp (see CLAUDE.md's "Code
 * organization" section and [[main-cpp-modularization]] project memory for
 * the pattern this follows). Moved verbatim — the REG_ON/REG_OFF/IS_REG_ON
 * (etc.) macros that every other part of the firmware uses to touch
 * registers are unchanged, still expand to the exact same calls, just now
 * declared in shiftreg.h instead of inline in the main .cpp.
 */

#include "shiftreg.h"
#include "buzzer.h"
#include "debug.h"
#include "config.h"
#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include <esp_rom_sys.h>

// ---- pins (private to this module — nothing outside it ever touched these
// by name; confirmed via grep before the extraction) --------------------
enum {
  PIN_SHIFT_REG_DATA = 27,          //(DATA_PIN)
  PIN_SHIFT_REG_OUTPUT_ENABLE = 26, //(Output enable[active low])
  PIN_SHIFT_REG_LATCH = 25,         //(Storage register clock pin (latch pin))
  PIN_SHIFT_REG_CLOCK = 33,         //(Shift register clock pin)
  PIN_SHIFT_REG_MASTER_CLEAR = 32,  //(Master Reclear[active low])

  PIN_SHIFT_REG_IN_CLOCK = 18, //(Shift register clock pin)
  PIN_SHIFT_REG_IN_LATCH = 23, //(Storage register clock pin (latch pin))
  PIN_SHIFT_REG_IN_DATA = 19,  //(DATA_PIN)
};

// Native GPIO calls are several times faster than the Arduino wrappers this
// bit-banging was tuned on; this pause keeps every clock phase at least as
// long as before (the CD4021 input register is slow CMOS).
#define SHIFT_CLOCK_PHASE_US 1

static void pin_init(int pin, gpio_mode_t mode) {
  gpio_config_t conf = {};
  conf.pin_bit_mask = 1ULL << pin;
  conf.mode = mode;
  conf.pull_up_en = GPIO_PULLUP_DISABLE;
  conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&conf);
}

static inline void pin_write(int pin, uint32_t level) {
  gpio_set_level((gpio_num_t)pin, level);
}

static inline int pin_read(int pin) {
  return gpio_get_level((gpio_num_t)pin);
}

static void shift_out_msb_first(uint8_t val) {
  for (int8_t i = 7; i >= 0; i--) {
    pin_write(PIN_SHIFT_REG_DATA, (val >> i) & 1);
    esp_rom_delay_us(SHIFT_CLOCK_PHASE_US);
    pin_write(PIN_SHIFT_REG_CLOCK, 1);
    esp_rom_delay_us(SHIFT_CLOCK_PHASE_US);
    pin_write(PIN_SHIFT_REG_CLOCK, 0);
  }
}

uint8_t shift_reg_in = 0;
uint16_t shift_reg = 0;

// Named-register lookup tables — see shiftreg.h's own comment for who reads
// these (debug/status JSON in the main .cpp).
ShiftRegList ShiftRegs = {
    {REG_CHOKE, "Choke"},
    {REG_FUEL_PUMP, "Fuel_pump"},

    {REG_BATT_FORCE, "Enable_battery"},
    {REG_BATT_OFF, "Disable_battery"},
    {REG_IGNITION, "Ignition"},
    {REG_ECO_MODE, "ECO"},
    {REG_LOAD_RELAY, "Load_switch"},
    {REG_CHARGER, "Charger"},
};

const uint8_t ShiftRegListLength =
    sizeof(ShiftRegs) / sizeof(ShiftRegItem);

ShiftRegList ShiftRegsIn = {
    {REG_IN_OVERLOAD, "Overload"},       {REG_IN_OIL, "Oil"},
    {REG_IN_OUTPUT, "Output"},

    {REG_IN_OVERHEAT, "Overheat"},

    {REG_IN_PSU_SENSE, "PSU_sense"},     {REG_IN_GEN_SENSE, "Generator_sense"},
    {REG_IN_BAT_SENSE, "Battery_sense"},
};

const uint8_t ShiftRegInListLength =
    sizeof(ShiftRegsIn) / sizeof(ShiftRegItem);

// Serialize all shift-register bit-bang GPIO access (LATCH/CLOCK/DATA pins)
// and shift_reg RMW updates. ShiftRegTask, StateMachineTask and the web
// fallback path all call reg_read()/reg_write(); concurrent bit-banging
// corrupts both the relay outputs and the input snapshot. The mutex is
// recursive because ShiftRegTask calls reg_write()/reg_read() while already
// holding it. Before the RTOS starts (setup) the mutex is NULL and access is
// single-threaded, so we proceed without locking.
static bool shiftreg_lock(void) {
  if (xShiftRegMutex == NULL) return false;
  return xSemaphoreTakeRecursive(xShiftRegMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE;
}

static void shiftreg_unlock(bool locked) {
  if (locked) xSemaphoreGiveRecursive(xShiftRegMutex);
}

uint8_t reg_read() {
  bool locked = shiftreg_lock();
  uint8_t myDataIn = 0;

  pin_write(PIN_SHIFT_REG_IN_LATCH, 1);

  // set it to 1 to collect parallel data, wait

  esp_rom_delay_us(20);

  // set it to 0 to transmit data serially

  pin_write(PIN_SHIFT_REG_IN_LATCH, 0);

  // we will be holding the clock pin high 8 times (0,..,7) at the
  // end of each time through the for loop

  // at the beginning of each loop when we set the clock low, it will
  // be doing the necessary low to high drop to cause the shift
  // register's DataPin to change state based on the value
  // of the next bit in its serial information flow.
  // The register transmits the information about the pins from pin 7 to pin 0
  // so that is why our function counts down

  for (int8_t i = 7; i >= 0; i--) {
    esp_rom_delay_us(SHIFT_CLOCK_PHASE_US);

    pin_write(PIN_SHIFT_REG_IN_CLOCK, 0);

    esp_rom_delay_us(SHIFT_CLOCK_PHASE_US);

    if (!pin_read(PIN_SHIFT_REG_IN_DATA)) {
      // set the bit to 0 no matter what
      myDataIn = myDataIn | (1 << i);
    }

    pin_write(PIN_SHIFT_REG_IN_CLOCK, 1);
  }

  shiftreg_unlock(locked);

  return myDataIn;
}

uint8_t read_regs() {
  shift_reg_in = reg_read();

  return shift_reg_in;
}

void setup_shift_reg_in() {
  shift_reg_in = 0;

  pin_init(PIN_SHIFT_REG_IN_CLOCK, GPIO_MODE_INPUT_OUTPUT);
  pin_write(PIN_SHIFT_REG_IN_CLOCK, 1);

  pin_init(PIN_SHIFT_REG_IN_LATCH, GPIO_MODE_INPUT_OUTPUT);
  pin_write(PIN_SHIFT_REG_IN_LATCH, 1);

  pin_init(PIN_SHIFT_REG_IN_DATA, GPIO_MODE_INPUT);

  read_regs();
}

void reg_write() {
  bool locked = shiftreg_lock();

  // выключаем на время передачи битов
  pin_write(PIN_SHIFT_REG_LATCH, 0);

  // разбиваем наши 16 бит на два байта
  // для записи в первый и второй регистр
  uint8_t registerOne = (uint8_t)(shift_reg >> 8);
  uint8_t registerTwo = (uint8_t)(shift_reg & 0xFF);

  // "проталкиваем" байты в регистры
  shift_out_msb_first(registerTwo);
  shift_out_msb_first(registerOne);

  // "защелкиваем" регистр, чтобы биты появились на выходах регистра
  pin_write(PIN_SHIFT_REG_LATCH, 1);

  shiftreg_unlock(locked);
}

void reg_set_state(uint8_t reg, bool reg_state) {
  if (reg < 16) {
    // Lock: SET_SHIFT_REG on shift_reg is a read-modify-write racing with
    // ShiftRegTask's command processing.
    bool locked = shiftreg_lock();
    SET_SHIFT_REG(reg, reg_state);
    shiftreg_unlock(locked);
  }
}

void reg_set(uint8_t reg, bool reg_state) {
  if (reg < 16) {
    // Hold the lock across set+write so the pair is atomic.
    bool locked = shiftreg_lock();
    reg_set_state(reg, reg_state);
    reg_write();
    shiftreg_unlock(locked);
  }
}

// Buzzer control (buzzerBeep()/buzzerAlarm()/etc., buzzer_process()) lives
// in buzzer.cpp/buzzer.h — it only needs a physical on/off pin write, which
// stays here since this file owns shift_reg/SET_SHIFT_REG.
void shiftreg_write_buzzer(bool on) {
    SET_SHIFT_REG(REG_BUZZER, on ? 1 : 0);
    reg_write();
}

void setup_shift_reg() {
  shift_reg = 0;

  pin_init(PIN_SHIFT_REG_OUTPUT_ENABLE, GPIO_MODE_INPUT_OUTPUT); //(Output enable[active low])
  pin_write(PIN_SHIFT_REG_OUTPUT_ENABLE, 1);

  pin_init(PIN_SHIFT_REG_DATA, GPIO_MODE_INPUT_OUTPUT); //(DATA_PIN)
  pin_write(PIN_SHIFT_REG_DATA, 0);

  pin_init(PIN_SHIFT_REG_LATCH, GPIO_MODE_INPUT_OUTPUT); //(Storage register clock pin (latch pin))
  pin_write(PIN_SHIFT_REG_LATCH, 0);

  pin_init(PIN_SHIFT_REG_CLOCK, GPIO_MODE_INPUT_OUTPUT); //(Shift register clock pin)
  pin_write(PIN_SHIFT_REG_CLOCK, 0);

  pin_init(PIN_SHIFT_REG_MASTER_CLEAR, GPIO_MODE_INPUT_OUTPUT); //(Master Reclear[active low])
  pin_write(PIN_SHIFT_REG_MASTER_CLEAR, 0);
  vTaskDelay(pdMS_TO_TICKS(2));
  pin_write(PIN_SHIFT_REG_MASTER_CLEAR, 1);
  vTaskDelay(pdMS_TO_TICKS(2));

  reg_write();

  vTaskDelay(pdMS_TO_TICKS(2));

  pin_write(PIN_SHIFT_REG_OUTPUT_ENABLE, 0);
}

// PSU-loss backup, fast path. The relay driver runs from the main supply rail,
// which only a hold-up capacitor (about 1 s) keeps alive once PSU and genset
// output are both gone. StateMachineTask can be inside a blocking wait of up
// to ~2 s (battery measurement, choke battery force), so the battery is
// connected here, at input-read time, on the same falling edge
// psu_backup_check() reacts to; that function then takes ownership.
// Caller holds xShiftRegMutex.
static void psu_backup_fast_engage(uint8_t inputs) {
  static bool prev_ext_power_ok = true;  // same boot assumption as psu_backup_check()
  bool ext_power_ok = (inputs & ((1 << REG_IN_PSU_SENSE) | (1 << REG_IN_GEN_SENSE))) != 0;
  if (g_battery_config.psu_loss_backup_enabled && prev_ext_power_ok && !ext_power_ok &&
      !IS_REG_ON(REG_BATT_FORCE)) {
    SET_SHIFT_REG(REG_BATT_FORCE, 1);
    reg_write();
  }
  prev_ext_power_ok = ext_power_ok;
}

// ----------------------------------------------------------------------------
// ShiftRegTask: Handle shift register I/O
// ----------------------------------------------------------------------------
void ShiftRegTask(void *pvParameters) {
    (void)pvParameters;
    esp_task_wdt_add(NULL);  // self-register before first wdt_reset
    OutputCommand_t cmd;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    SLOG_PRINTLN("ShiftRegTask started");

    // Wait for queues to be ready (safety delay)
    while (xOutputCommandQueue == NULL || xBuzzerCommandQueue == NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    for (;;) {
        // Process all pending commands
        while (xQueueReceive(xOutputCommandQueue, &cmd, 0) == pdTRUE) {
            if (xSemaphoreTakeRecursive(xShiftRegMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
                switch (cmd.type) {
                    case CMD_REG_SET:
                        SET_SHIFT_REG(cmd.reg, cmd.state);
                        reg_write();
                        break;
                    case CMD_REG_ON:
                        SET_SHIFT_REG(cmd.reg, 1);
                        reg_write();
                        break;
                    case CMD_REG_OFF:
                        SET_SHIFT_REG(cmd.reg, 0);
                        reg_write();
                        break;
                    case CMD_REG_TOGGLE:
                        SET_SHIFT_REG(cmd.reg, !IS_REG_ON(cmd.reg));
                        reg_write();
                        break;
                    case CMD_REG_SYNC:
                        shift_reg = cmd.full_state;
                        reg_write();
                        break;
                }
                xSemaphoreGiveRecursive(xShiftRegMutex);
            }
        }

        // Process buzzer state machine and read input registers
        // Combined under single mutex to reduce lock contention
        if (xSemaphoreTakeRecursive(xShiftRegMutex, MUTEX_TIMEOUT_TICKS) == pdTRUE) {
            // Process buzzer
            buzzer_process();

            // Read inputs
            uint8_t inputs = reg_read();
            psu_backup_fast_engage(inputs);
            uint16_t outputs = shift_reg;
            xSemaphoreGiveRecursive(xShiftRegMutex);

            // Update shared state (outside mutex, using spinlock)
            portENTER_CRITICAL(&g_state_spinlock);
            g_state.shift_reg_inputs = inputs;
            g_state.shift_reg_outputs = outputs;
            shift_reg_in = inputs;
            portEXIT_CRITICAL(&g_state_spinlock);
        }

        // Notify state machine
        if (xStateEvents != NULL) {
            xEventGroupSetBits(xStateEvents, EVENT_SHIFT_REG_UPDATE);
        }

        // Feed watchdog
        esp_task_wdt_reset();

        // Wait for next period
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PERIOD_SHIFT_REG_MS));
    }
}
