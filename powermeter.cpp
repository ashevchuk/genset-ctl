/*
 * powermeter.cpp — PowerMeterTask, reads the MCP39F511A power meter over
 * UART (ESP-IDF uart driver on UART2).
 *
 * Extracted 2026-09-24 from esp32-gen-ctl-ai-rtos.cpp (see CLAUDE.md's "Code
 * organization" section and [[main-cpp-modularization]] project memory for
 * the pattern this follows). Moved verbatim. Transport migrated off
 * HardwareSerial/Serial2 to mcp39f511a_uart_begin() (ESP-IDF) 2026-09-29.
 */

#include "powermeter.h"
#include "rtos_config.h"
#include "fram_log.h"
#include "mcp39f511a_dev.h"
#include "debug.h"
#include <esp_task_wdt.h>
#include "uptime.h"

// Boot-time snapshot of the device's config registers — read once in
// mcp39f511a_read_config()/mcp39f511a_load_config_from_flash(), only ever
// consumed by this file's own print helpers below.
static Mcp39f511aConfig mcp39f511a_config = {};

// #define POWERMON to compile the chip out entirely (bench-testing without
// the meter attached) — never actually defined anywhere in this project, so
// this #ifndef is always true; kept for parity with the main .cpp's own
// (equally inert) copy of the same flag.
// #define POWERMON

// Energy counters accumulate slowly (mWh) — no need to re-read every
// PERIOD_POWER_METER_MS cycle like the measurements above. Values persist
// across cycles (memset() below only clears the local power_data, not
// these) and are only refreshed every MCP39F511A_ENERGY_READ_PERIOD_MS.
#define MCP39F511A_ENERGY_READ_PERIOD_MS (30000)
static uint64_t s_last_import_active_mwh = 0;
static uint64_t s_last_export_active_mwh = 0;
static uint64_t s_last_import_reactive_mwh = 0;
static uint64_t s_last_export_reactive_mwh = 0;
static bool s_energy_ever_valid = false;
static unsigned long long s_energyReadTime = 0;

void powermeter_note_energy_reset(void) {
    s_last_import_active_mwh = 0;
    s_last_export_active_mwh = 0;
    s_last_import_reactive_mwh = 0;
    s_last_export_reactive_mwh = 0;
    s_energy_ever_valid = true;
    s_energyReadTime = 0;  // force a fresh chip read on the next PowerMeterTask tick

    portENTER_CRITICAL(&g_state_spinlock);
    g_state.power.import_active_energy_mwh = 0;
    g_state.power.export_active_energy_mwh = 0;
    g_state.power.import_reactive_energy_mwh = 0;
    g_state.power.export_reactive_energy_mwh = 0;
    g_state.power.energy_valid = true;
    portEXIT_CRITICAL(&g_state_spinlock);
}

void PowerMeterTask(void *pvParameters) {
    (void)pvParameters;
    esp_task_wdt_add(NULL);  // self-register before first wdt_reset
    PowerData_t power_data;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    SLOG_PRINTLN("PowerMeterTask started");

    for (;;) {
        memset(&power_data, 0, sizeof(power_data));
        power_data.timestamp = uptime_ms();

        // Carry forward the last energy reading — refreshed below on its
        // own, slower period.
        power_data.import_active_energy_mwh = s_last_import_active_mwh;
        power_data.export_active_energy_mwh = s_last_export_active_mwh;
        power_data.import_reactive_energy_mwh = s_last_import_reactive_mwh;
        power_data.export_reactive_energy_mwh = s_last_export_reactive_mwh;
        power_data.energy_valid = s_energy_ever_valid;

        // Acquire MCP UART mutex
        if (TAKE_MUTEX(xSerial2Mutex)) {
#ifndef POWERMON
            if (mcp39f511a.readMeasurements()) {
                Mcp39f511aMeasurements *libData = mcp39f511a.getMeasurements();
                power_data.rms_voltage = libData->rmsVoltage;
                power_data.rms_current = libData->rmsCurrent;
                power_data.active_power = libData->activePower;
                power_data.reactive_power = libData->reactivePower;
                power_data.apparent_power = libData->apparentPower;
                power_data.frequency = libData->frequency;
                power_data.power_factor = libData->powerFactor;
                power_data.valid = libData->valid;
            }

            if (uptime_ms() - s_energyReadTime >= MCP39F511A_ENERGY_READ_PERIOD_MS) {
                s_energyReadTime = uptime_ms();
                if (mcp39f511a.readEnergyCounters()) {
                    Mcp39f511aEnergyCounters *e = mcp39f511a.getEnergyCounters();
                    s_last_import_active_mwh = e->importActiveEnergy;
                    s_last_export_active_mwh = e->exportActiveEnergy;
                    s_last_import_reactive_mwh = e->importReactiveEnergy;
                    s_last_export_reactive_mwh = e->exportReactiveEnergy;
                    s_energy_ever_valid = true;

                    power_data.import_active_energy_mwh = s_last_import_active_mwh;
                    power_data.export_active_energy_mwh = s_last_export_active_mwh;
                    power_data.import_reactive_energy_mwh = s_last_import_reactive_mwh;
                    power_data.export_reactive_energy_mwh = s_last_export_reactive_mwh;
                    power_data.energy_valid = true;
                }
            }
#endif
            GIVE_MUTEX(xSerial2Mutex);
        }

        // Track lifetime peak power (RAM only, flushed to FRAM by LogTask)
        if (power_data.valid) {
            fram_metrics_note_power(power_data.active_power);
        }

        // Send to queue (overwrite old data)
        if (xPowerDataQueue != NULL) {
            QUEUE_OVERWRITE(xPowerDataQueue, &power_data);
        }

        // Copy to shared state
        portENTER_CRITICAL(&g_state_spinlock);
        g_state.power = power_data;
        portEXIT_CRITICAL(&g_state_spinlock);

        // Notify state machine
        if (xStateEvents != NULL) {
            xEventGroupSetBits(xStateEvents, EVENT_POWER_UPDATE);
        }

        // Feed watchdog
        esp_task_wdt_reset();

        // Wait for next period
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PERIOD_POWER_METER_MS));
    }
}

// Only read internally by mcp39f511a_load_config_from_flash() below.
static bool mcp39f511a_read_config() {
  if (mcp39f511a.readConfig()) {
    mcp39f511a_config = *mcp39f511a.getConfig();
    return true;
  }
  return false;
}

// Load configuration from flash (device loads automatically on power-up,
// but this function can be used to verify current configuration)
bool mcp39f511a_load_config_from_flash() {
  // Device automatically loads configuration from flash on power-up
  // This function reads current configuration from registers
  // (which should match flash after power-up)
  return mcp39f511a_read_config();
}

// Print configuration registers to SDEBUG with detailed bit information.
// Field names/addresses verified directly against doc/MCP39F511A-Data-Sheet-20006044A.pdf
// (Register 6-2/7-1) — see mcp39f511a_dev.h's header comment for why the
// vendored MCP39F511A_UART.h's own config types/addresses aren't used here.
void mcp39f511a_print_config() {
  if (!mcp39f511a_config.valid) {
    SDEBUG("MCP39F511A Config: Not valid");
    return;
  }

  char buf[256];
  SDEBUG("MCP39F511A Configuration Registers:");

  // System Configuration Register (0x0094)
  snprintf(buf, sizeof(buf), "  System Configuration (0x0094): 0x%08lX",
           (unsigned long)mcp39f511a_config.sysConfig.uDWord);
  SDEBUG(buf);
  snprintf(buf, sizeof(buf),
           "    ENRG_CNTRL: %d, PWM_CNTRL: %d, VREFEXT: %d, SHUTDOWN: %d, RESET: %d",
           mcp39f511a_config.sysConfig.ENRG_CNTRL, mcp39f511a_config.sysConfig.PWM_CNTRL,
           mcp39f511a_config.sysConfig.VREFEXT, mcp39f511a_config.sysConfig.SHUTDOWN,
           mcp39f511a_config.sysConfig.RESET);
  SDEBUG(buf);
  snprintf(buf, sizeof(buf),
           "    TEMPCOMP: %d, SINGLE_WIRE: %d, ZCD_OUTPUT_DIS: %d, ZCD_PULS: %d, ZCD_INV: %d",
           mcp39f511a_config.sysConfig.TEMPCOMP, mcp39f511a_config.sysConfig.SINGLE_WIRE,
           mcp39f511a_config.sysConfig.ZCD_OUTPUT_DIS, mcp39f511a_config.sysConfig.ZCD_PULS,
           mcp39f511a_config.sysConfig.ZCD_INV);
  SDEBUG(buf);
  snprintf(buf, sizeof(buf), "    UART: %d, PGA_CH0: %d, PGA_CH1: %d",
           mcp39f511a_config.sysConfig.UART,
           mcp39f511a_config.sysConfig.PGA_CH0,
           mcp39f511a_config.sysConfig.PGA_CH1);
  SDEBUG(buf);

  // Event Configuration Register (0x0098)
  snprintf(buf, sizeof(buf), "  Event Configuration (0x0098): 0x%08lX",
           (unsigned long)mcp39f511a_config.eventConfig.uDWord);
  SDEBUG(buf);
  snprintf(buf, sizeof(buf),
           "    OVERCUR_TST: %d, OVERPOW_TST: %d, VSAG_TST: %d, VSUR_TST: %d",
           mcp39f511a_config.eventConfig.OVERCUR_TST,
           mcp39f511a_config.eventConfig.OVERPOW_TST,
           mcp39f511a_config.eventConfig.VSAG_TST,
           mcp39f511a_config.eventConfig.VSUR_TST);
  SDEBUG(buf);
  snprintf(buf, sizeof(buf),
           "    OVERCUR_LA: %d, OVERPOW_LA: %d, VSAG_LA: %d, VSUR_LA: %d",
           mcp39f511a_config.eventConfig.OVERCUR_LA,
           mcp39f511a_config.eventConfig.OVERPOW_LA,
           mcp39f511a_config.eventConfig.VSAG_LA,
           mcp39f511a_config.eventConfig.VSUR_LA);
  SDEBUG(buf);
  snprintf(buf, sizeof(buf),
           "    VSAG_CL: %d, VSUR_CL: %d, OVERPOW_CL: %d, OVERCUR_CL: %d",
           mcp39f511a_config.eventConfig.VSAG_CL,
           mcp39f511a_config.eventConfig.VSUR_CL,
           mcp39f511a_config.eventConfig.OVERPOW_CL,
           mcp39f511a_config.eventConfig.OVERCUR_CL);
  SDEBUG(buf);
  snprintf(buf, sizeof(buf),
           "    OVERTEMP_TST: %d, OVERTEMP_LA: %d, OVERTEMP_CL: %d",
           mcp39f511a_config.eventConfig.OVERTEMP_TST,
           mcp39f511a_config.eventConfig.OVERTEMP_LA,
           mcp39f511a_config.eventConfig.OVERTEMP_CL);
  SDEBUG(buf);
}

// Same dump as mcp39f511a_print_config(), but to the physical USB serial
// port instead of SDEBUG (which only reaches connected Telnet clients) —
// so the configuration can be read straight off a serial monitor with no
// DEBUG build flag and no network client needed.
void mcp39f511a_print_config_serial(void) {
  if (!mcp39f511a_config.valid) {
    SLOG_PRINTLN("MCP39F511A Config: Not valid");
    return;
  }

  SLOG_PRINTLN("MCP39F511A Configuration Registers:");

  SLOG_PRINTF("  System Configuration (0x0094): 0x%08lX\n",
                (unsigned long)mcp39f511a_config.sysConfig.uDWord);
  SLOG_PRINTF(
      "    ENRG_CNTRL: %d, PWM_CNTRL: %d, VREFEXT: %d, SHUTDOWN: %d, RESET: %d\n",
      mcp39f511a_config.sysConfig.ENRG_CNTRL, mcp39f511a_config.sysConfig.PWM_CNTRL,
      mcp39f511a_config.sysConfig.VREFEXT, mcp39f511a_config.sysConfig.SHUTDOWN,
      mcp39f511a_config.sysConfig.RESET);
  SLOG_PRINTF(
      "    TEMPCOMP: %d, SINGLE_WIRE: %d, ZCD_OUTPUT_DIS: %d, ZCD_PULS: %d, ZCD_INV: %d\n",
      mcp39f511a_config.sysConfig.TEMPCOMP, mcp39f511a_config.sysConfig.SINGLE_WIRE,
      mcp39f511a_config.sysConfig.ZCD_OUTPUT_DIS, mcp39f511a_config.sysConfig.ZCD_PULS,
      mcp39f511a_config.sysConfig.ZCD_INV);
  SLOG_PRINTF("    UART: %d (sanity check: should be 4 == 100b == 9600 baud), PGA_CH0: %d, PGA_CH1: %d\n",
                mcp39f511a_config.sysConfig.UART,
                mcp39f511a_config.sysConfig.PGA_CH0,
                mcp39f511a_config.sysConfig.PGA_CH1);

  SLOG_PRINTF("  Event Configuration (0x0098): 0x%08lX\n",
                (unsigned long)mcp39f511a_config.eventConfig.uDWord);
  SLOG_PRINTF("    OVERCUR_TST: %d, OVERPOW_TST: %d, VSAG_TST: %d, VSUR_TST: %d\n",
                mcp39f511a_config.eventConfig.OVERCUR_TST,
                mcp39f511a_config.eventConfig.OVERPOW_TST,
                mcp39f511a_config.eventConfig.VSAG_TST,
                mcp39f511a_config.eventConfig.VSUR_TST);
  SLOG_PRINTF("    OVERCUR_LA: %d, OVERPOW_LA: %d, VSAG_LA: %d, VSUR_LA: %d\n",
                mcp39f511a_config.eventConfig.OVERCUR_LA,
                mcp39f511a_config.eventConfig.OVERPOW_LA,
                mcp39f511a_config.eventConfig.VSAG_LA,
                mcp39f511a_config.eventConfig.VSUR_LA);
  SLOG_PRINTF("    VSAG_CL: %d, VSUR_CL: %d, OVERPOW_CL: %d, OVERCUR_CL: %d\n",
                mcp39f511a_config.eventConfig.VSAG_CL,
                mcp39f511a_config.eventConfig.VSUR_CL,
                mcp39f511a_config.eventConfig.OVERPOW_CL,
                mcp39f511a_config.eventConfig.OVERCUR_CL);
  SLOG_PRINTF("    OVERTEMP_TST: %d, OVERTEMP_LA: %d, OVERTEMP_CL: %d\n",
                mcp39f511a_config.eventConfig.OVERTEMP_TST,
                mcp39f511a_config.eventConfig.OVERTEMP_LA,
                mcp39f511a_config.eventConfig.OVERTEMP_CL);
}

// Dumps every calibration/configuration register (mcp39f511a.readFullConfig())
// to the physical serial port — used to capture a working chip's exact
// configuration so it can be cloned onto a factory-default replacement chip
// via Mcp39f511aDev::writeFullConfig() (see the commented-out restore block
// in setup()).
void mcp39f511a_print_full_config_serial(void) {
  MCP39F511A_FullConfig *c = mcp39f511a.getFullConfig();
  if (!c->valid) {
    SLOG_PRINTLN("MCP39F511A Full Config: Not valid");
    return;
  }

  SLOG_PRINTLN("MCP39F511A Full Calibration/Configuration:");
  SLOG_PRINTF("  acGainCurrentRms            = 0x%04X\n", c->acGainCurrentRms);
  SLOG_PRINTF("  acGainVoltageRms            = 0x%04X\n", c->acGainVoltageRms);
  SLOG_PRINTF("  acGainActivePower           = 0x%04X\n", c->acGainActivePower);
  SLOG_PRINTF("  acGainReactivePower         = 0x%04X\n", c->acGainReactivePower);
  SLOG_PRINTF("  acOffsetCurrentRms          = %d\n", c->acOffsetCurrentRms);
  SLOG_PRINTF("  acOffsetActivePower         = %d\n", c->acOffsetActivePower);
  SLOG_PRINTF("  acOffsetReactivePower       = %d\n", c->acOffsetReactivePower);
  SLOG_PRINTF("  gainLineFrequency           = 0x%04X\n", c->gainLineFrequency);
  SLOG_PRINTF("  phaseCompensation           = %d\n", c->phaseCompensation);
  SLOG_PRINTF("  voltageDropComp             = 0x%04X\n", c->voltageDropComp);
  SLOG_PRINTF("  inCapCurrentComp            = 0x%04X\n", c->inCapCurrentComp);
  SLOG_PRINTF("  rangeVdropInCapComp         = 0x%04X\n", c->rangeVdropInCapComp);
  SLOG_PRINTF("  dcGainCurrentRms            = 0x%04X\n", c->dcGainCurrentRms);
  SLOG_PRINTF("  dcGainVoltageRms            = 0x%04X\n", c->dcGainVoltageRms);
  SLOG_PRINTF("  dcGainActivePower           = 0x%04X\n", c->dcGainActivePower);
  SLOG_PRINTF("  dcOffsetCurrentRms          = %d\n", c->dcOffsetCurrentRms);
  SLOG_PRINTF("  dcOffsetActivePower         = %d\n", c->dcOffsetActivePower);
  SLOG_PRINTF("  offcalMsb                   = 0x%04X\n", c->offcalMsb);
  SLOG_PRINTF("  offcalCh0                   = 0x%04X\n", c->offcalCh0);
  SLOG_PRINTF("  offcalCh1                   = 0x%04X\n", c->offcalCh1);
  SLOG_PRINTF("  tempPosCompFreq             = 0x%04X\n", c->tempPosCompFreq);
  SLOG_PRINTF("  tempNegCompFreq             = 0x%04X\n", c->tempNegCompFreq);
  SLOG_PRINTF("  tempPosCompCurrent          = 0x%04X\n", c->tempPosCompCurrent);
  SLOG_PRINTF("  tempNegCompCurrent          = 0x%04X\n", c->tempNegCompCurrent);
  SLOG_PRINTF("  tempPosCompPower            = 0x%04X\n", c->tempPosCompPower);
  SLOG_PRINTF("  tempNegCompPower            = 0x%04X\n", c->tempNegCompPower);
  SLOG_PRINTF("  sysConfig.uDWord            = 0x%08lX\n", (unsigned long)c->sysConfig.uDWord);
  SLOG_PRINTF("  eventConfig.uDWord          = 0x%08lX\n", (unsigned long)c->eventConfig.uDWord);
  SLOG_PRINTF("  range                       = 0x%08lX\n", (unsigned long)c->range);
  SLOG_PRINTF("  calibrationCurrent          = %lu\n", (unsigned long)c->calibrationCurrent);
  SLOG_PRINTF("  calibrationVoltage          = %u\n", c->calibrationVoltage);
  SLOG_PRINTF("  calibrationActivePower      = %lu\n", (unsigned long)c->calibrationActivePower);
  SLOG_PRINTF("  calibrationReactivePower    = %lu\n", (unsigned long)c->calibrationReactivePower);
  SLOG_PRINTF("  apparentPowerDivisor        = %u\n", c->apparentPowerDivisor);
  SLOG_PRINTF("  accumulatorIntervalParameter= %u\n", c->accumulatorIntervalParameter);
  SLOG_PRINTF("  pwmPeriod                   = %u\n", c->pwmPeriod);
  SLOG_PRINTF("  pwmDutyCycle                = %u\n", c->pwmDutyCycle);
  SLOG_PRINTF("  minMaxPointer1              = %u\n", c->minMaxPointer1);
  SLOG_PRINTF("  minMaxPointer2              = %u\n", c->minMaxPointer2);
  SLOG_PRINTF("  lineFrequencyReference      = %u\n", c->lineFrequencyReference);
  SLOG_PRINTF("  thermistorVoltageCal        = %u\n", c->thermistorVoltageCal);
  SLOG_PRINTF("  voltageSagLimit             = %u\n", c->voltageSagLimit);
  SLOG_PRINTF("  voltageSurgeLimit           = %u\n", c->voltageSurgeLimit);
  SLOG_PRINTF("  overCurrentLimit            = %lu\n", (unsigned long)c->overCurrentLimit);
  SLOG_PRINTF("  overPowerLimit              = %lu\n", (unsigned long)c->overPowerLimit);
  SLOG_PRINTF("  overTemperatureLimit        = %u\n", c->overTemperatureLimit);
  SLOG_PRINTF("  voltageLowThreshold         = %u\n", c->voltageLowThreshold);
  SLOG_PRINTF("  voltageHighThreshold        = %u\n", c->voltageHighThreshold);
  SLOG_PRINTF("  noLoadThreshold             = %u\n", c->noLoadThreshold);
}

// Dumps the 4 lifetime energy accumulators (mcp39f511a.readEnergyCounters())
// to the physical serial port. 1 mWh resolution (datasheet default) — divide
// by 1000 for Wh, 1000000 for kWh.
void mcp39f511a_print_energy_counters_serial(void) {
  Mcp39f511aEnergyCounters *e = mcp39f511a.getEnergyCounters();
  if (!e->valid) {
    SLOG_PRINTLN("MCP39F511A Energy Counters: Not valid");
    return;
  }

  SLOG_PRINTLN("MCP39F511A Energy Counters (mWh):");
  SLOG_PRINTF("  importActiveEnergy   = %llu\n", (unsigned long long)e->importActiveEnergy);
  SLOG_PRINTF("  exportActiveEnergy   = %llu\n", (unsigned long long)e->exportActiveEnergy);
  SLOG_PRINTF("  importReactiveEnergy = %llu\n", (unsigned long long)e->importReactiveEnergy);
  SLOG_PRINTF("  exportReactiveEnergy = %llu\n", (unsigned long long)e->exportReactiveEnergy);
}
