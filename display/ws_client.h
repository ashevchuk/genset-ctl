#pragma once
#include <Arduino.h>
#include "freertos/portmacro.h"

// Spinlock protecting g_gen between the WS task (core 0) and loop() (core 1).
// Use portENTER_CRITICAL(&ws_gen_mux) / portEXIT_CRITICAL(&ws_gen_mux).
extern portMUX_TYPE ws_gen_mux;

// Register indices (match generator shift register mapping)
#define REG_CHOKE      0
#define REG_FUEL_PUMP  1
#define REG_ECO_MODE   12
#define REG_LOAD_RELAY 13
#define REG_CHARGER    14

struct GenData {
    // State. `state` is value.STATE from the controller (state_name():
    // "idle", "starting", "running", …). Empty when that field is absent,
    // in which case the UI falls back to running/stopped.
    bool running;
    bool stopped;
    char state[20];

    // Outputs
    bool choke;
    bool fuel_pump;
    bool eco_mode;
    bool load_relay;
    bool charger;
    bool ignition;

    // Inputs
    bool overload;
    bool oil;
    bool output_ok;
    bool overheat;
    bool psu_sense;
    bool gen_sense;
    bool bat_sense;

    // Values
    float rpm;
    float freq;
    float vrms;
    float irms;
    float power_active;
    float power_apparent;
    float power_reactive;
    float power_factor;
    float head_temp;
    float fuel_level;      // ADC1_0 — litres
    float oil_temp;        // ADC0_0 — oil temperature, °C
    float ambient_temp;    // ADC0_1 — ambient temperature, °C
    float inverter_temp;   // ADC0_2 — inverter temperature, °C
    float alternator_temp; // ADC0_3 — alternator temperature, °C
    float bat_voltage;     // ADC1_1 — battery voltage, V
    float core_voltage;    // ADC1_2 — core voltage, V
    int   rssi;

    // Bit i set => FLOG_MAINT_* index i overdue (controller's ReportTask,
    // same computation as the web dashboard's MAINT_DUE banner). Consumed by
    // ui.cpp's Controls-tab maintenance bar; index<->name/label tables live
    // in journal.h, kept in sync with the controller's fram_log.cpp by hand.
    uint8_t maint_due_mask;

    uint32_t uptime;
    bool     valid;
    uint32_t last_update;
};

extern GenData g_gen;

void ws_client_init(const char *host, uint16_t port);
void ws_client_disconnect(void);
void ws_client_reconnect(void);
bool ws_client_connected(void);
// Connected AND a report arrived within the last few seconds.
bool ws_client_fresh(void);
// Connected, but no report for this long (since the last report or since
// connecting). 0 when not connected.
uint32_t ws_client_silent_ms(void);
void ws_send(const char *msg);
void ws_send_toggle(uint8_t reg);
