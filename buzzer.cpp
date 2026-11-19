#include "buzzer.h"
#include "rtos_config.h"

// Buzzer state - managed by ShiftRegTask via buzzer_process()
static BuzzerState_t g_buzzer_state = {
    .active = false,
    .buzzer_on = false,
    .duration_ms = 0,
    .pause_ms = 0,
    .count = 0,
    .total_count = 0,
    .next_toggle = 0
};

void buzzerBeep(uint16_t duration_ms, uint8_t count, uint16_t pause_ms) {
    if (xBuzzerCommandQueue == NULL) return;

    BuzzerCommand_t cmd;
    cmd.type = BUZZER_CMD_BEEP;
    cmd.duration_ms = duration_ms;
    cmd.count = count;
    cmd.pause_ms = pause_ms;

    xQueueSend(xBuzzerCommandQueue, &cmd, 0);
}

void buzzerBeepInfinite(uint16_t duration_ms, uint16_t pause_ms) {
    buzzerBeep(duration_ms, 0, pause_ms);
}

void buzzerStop(void) {
    if (xBuzzerCommandQueue == NULL) return;

    BuzzerCommand_t cmd;
    cmd.type = BUZZER_CMD_STOP;
    cmd.duration_ms = 0;
    cmd.count = 0;
    cmd.pause_ms = 0;

    xQueueSend(xBuzzerCommandQueue, &cmd, 0);
}

bool buzzerIsActive(void) {
    return g_buzzer_state.active;
}

void buzzerBeepOnce(uint16_t duration_ms) {
    buzzerBeep(duration_ms, 1, 0);
}

void buzzerBeepOnce(void) {
    buzzerBeepOnce(BUZZER_DEFAULT_DURATION_MS);
}

void buzzerBeepDouble(uint16_t duration_ms, uint16_t pause_ms) {
    buzzerBeep(duration_ms, 2, pause_ms);
}

void buzzerBeepDouble(void) {
    buzzerBeepDouble(BUZZER_DEFAULT_DURATION_MS, BUZZER_DEFAULT_PAUSE_MS);
}

void buzzerBeepTriple(uint16_t duration_ms, uint16_t pause_ms) {
    buzzerBeep(duration_ms, 3, pause_ms);
}

void buzzerBeepTriple(void) {
    buzzerBeepTriple(BUZZER_DEFAULT_DURATION_MS, BUZZER_DEFAULT_PAUSE_MS);
}

void buzzerAlarm(uint16_t duration_ms, uint16_t pause_ms) {
    buzzerBeepInfinite(duration_ms, pause_ms);
}

void buzzerAlarm(void) {
    buzzerAlarm(BUZZER_ALARM_DURATION_MS, BUZZER_ALARM_PAUSE_MS);
}

void buzzer_process(void) {
    if (xBuzzerCommandQueue == NULL) return;

    // Process commands from queue
    BuzzerCommand_t cmd;
    while (xQueueReceive(xBuzzerCommandQueue, &cmd, 0) == pdTRUE) {
        switch (cmd.type) {
            case BUZZER_CMD_BEEP:
                g_buzzer_state.active = true;
                g_buzzer_state.buzzer_on = true;
                g_buzzer_state.duration_ms = cmd.duration_ms;
                g_buzzer_state.pause_ms = cmd.pause_ms;
                g_buzzer_state.count = cmd.count;
                g_buzzer_state.total_count = cmd.count;
                g_buzzer_state.next_toggle = millis() + cmd.duration_ms;

                shiftreg_write_buzzer(true);
                break;

            case BUZZER_CMD_STOP:
                g_buzzer_state.active = false;
                g_buzzer_state.buzzer_on = false;
                g_buzzer_state.count = 0;

                shiftreg_write_buzzer(false);
                break;
        }
    }

    // Process ongoing beep pattern
    if (g_buzzer_state.active) {
        uint32_t now = millis();

        // Use signed comparison to handle millis() overflow correctly
        if ((int32_t)(now - g_buzzer_state.next_toggle) >= 0) {
            if (g_buzzer_state.buzzer_on) {
                g_buzzer_state.buzzer_on = false;
                shiftreg_write_buzzer(false);

                // Check if this was the last beep
                if (g_buzzer_state.total_count != 0) {  // Not infinite
                    g_buzzer_state.count--;
                    if (g_buzzer_state.count == 0) {
                        g_buzzer_state.active = false;
                        return;
                    }
                }

                g_buzzer_state.next_toggle = now + g_buzzer_state.pause_ms;
            } else {
                g_buzzer_state.buzzer_on = true;
                shiftreg_write_buzzer(true);

                g_buzzer_state.next_toggle = now + g_buzzer_state.duration_ms;
            }
        }
    }
}
