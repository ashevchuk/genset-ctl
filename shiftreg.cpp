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
#include <esp_task_wdt.h>

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

  digitalWrite(PIN_SHIFT_REG_IN_LATCH, HIGH);

  // set it to 1 to collect parallel data, wait

  delayMicroseconds(20);

  // set it to 0 to transmit data serially

  digitalWrite(PIN_SHIFT_REG_IN_LATCH, LOW);

  // we will be holding the clock pin high 8 times (0,..,7) at the
  // end of each time through the for loop

  // at the beginning of each loop when we set the clock low, it will
  // be doing the necessary low to high drop to cause the shift
  // register's DataPin to change state based on the value
  // of the next bit in its serial information flow.
  // The register transmits the information about the pins from pin 7 to pin 0
  // so that is why our function counts down

  for (int8_t i = 7; i >= 0; i--) {
    delayMicroseconds(0.2);

    digitalWrite(PIN_SHIFT_REG_IN_CLOCK, LOW);

    delayMicroseconds(0.2);

    if (!digitalRead(PIN_SHIFT_REG_IN_DATA)) {
      // set the bit to 0 no matter what
      myDataIn = myDataIn | (1 << i);
    }

    digitalWrite(PIN_SHIFT_REG_IN_CLOCK, HIGH);
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

  pinMode(PIN_SHIFT_REG_IN_CLOCK, OUTPUT);
  digitalWrite(PIN_SHIFT_REG_IN_CLOCK, HIGH);

  pinMode(PIN_SHIFT_REG_IN_LATCH, OUTPUT);
  digitalWrite(PIN_SHIFT_REG_IN_LATCH, HIGH);

  pinMode(PIN_SHIFT_REG_IN_DATA, INPUT);

  read_regs();
}

void reg_write() {
  bool locked = shiftreg_lock();

  // выключаем на время передачи битов
  digitalWrite(PIN_SHIFT_REG_LATCH, LOW);

  // разбиваем наши 16 бит на два байта
  // для записи в первый и второй регистр
  byte registerOne = highByte(shift_reg);
  byte registerTwo = lowByte(shift_reg);

  // "проталкиваем" байты в регистры
  shiftOut(PIN_SHIFT_REG_DATA, PIN_SHIFT_REG_CLOCK, MSBFIRST, registerTwo);
  shiftOut(PIN_SHIFT_REG_DATA, PIN_SHIFT_REG_CLOCK, MSBFIRST, registerOne);

  // "защелкиваем" регистр, чтобы биты появились на выходах регистра
  digitalWrite(PIN_SHIFT_REG_LATCH, HIGH);

  shiftreg_unlock(locked);
}

void reg_set_state(uint8_t reg, bool reg_state) {
  if (reg < 16) {
    // Lock: bitWrite on shift_reg is a read-modify-write racing with
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
    SET_SHIFT_REG(REG_BUZZER, on ? HIGH : LOW);
    reg_write();
}

void setup_shift_reg() {
  shift_reg = 0;

  pinMode(PIN_SHIFT_REG_OUTPUT_ENABLE, OUTPUT); //(Output enable[active low])
  digitalWrite(PIN_SHIFT_REG_OUTPUT_ENABLE, HIGH);

  pinMode(PIN_SHIFT_REG_DATA, OUTPUT); //(DATA_PIN)
  digitalWrite(PIN_SHIFT_REG_DATA, LOW);

  pinMode(PIN_SHIFT_REG_LATCH,
          OUTPUT); //(Storage register clock pin (latch pin))
  digitalWrite(PIN_SHIFT_REG_LATCH, LOW);

  pinMode(PIN_SHIFT_REG_CLOCK, OUTPUT); //(Shift register clock pin)
  digitalWrite(PIN_SHIFT_REG_CLOCK, LOW);

  pinMode(PIN_SHIFT_REG_MASTER_CLEAR, OUTPUT); //(Master Reclear[active low])
  digitalWrite(PIN_SHIFT_REG_MASTER_CLEAR, LOW);
  delay(2);
  digitalWrite(PIN_SHIFT_REG_MASTER_CLEAR, HIGH);
  delay(2);

  reg_write();

  delay(2);

  digitalWrite(PIN_SHIFT_REG_OUTPUT_ENABLE, LOW);
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
                        SET_SHIFT_REG(cmd.reg, HIGH);
                        reg_write();
                        break;
                    case CMD_REG_OFF:
                        SET_SHIFT_REG(cmd.reg, LOW);
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
