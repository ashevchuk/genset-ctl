#include "mcp39f511a_dev.h"
#include "debug.h"
#include <esp_task_wdt.h>
#include <driver/uart.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

static bool s_uart_ready = false;

bool mcp39f511a_uart_begin(void) {
    if (s_uart_ready) {
        return true;
    }

    const uart_port_t port = (uart_port_t)MCP39F511A_UART_NUM;
    // If Arduino ever claimed this port, uninstall first so we own it cleanly.
    uart_driver_delete(port);

    uart_config_t cfg = {};
    cfg.baud_rate = MCP39F511A_UART_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    if (uart_param_config(port, &cfg) != ESP_OK) {
        SLOG_PRINTLN("[MCP39F511A] uart_param_config failed");
        return false;
    }
    if (uart_set_pin(port, MCP39F511A_UART_TX_GPIO, MCP39F511A_UART_RX_GPIO,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        SLOG_PRINTLN("[MCP39F511A] uart_set_pin failed");
        return false;
    }
    // Modest rings — protocol frames are tiny; 8 KB Arduino buffers were leftover.
    if (uart_driver_install(port, 1024, 256, 0, nullptr, 0) != ESP_OK) {
        SLOG_PRINTLN("[MCP39F511A] uart_driver_install failed");
        return false;
    }

    s_uart_ready = true;
    return true;
}

void Mcp39f511aDev::flushRx(void) {
    if (!s_uart_ready) return;
    uart_flush_input((uart_port_t)MCP39F511A_UART_NUM);
}

bool Mcp39f511aDev::readExact(uint8_t *buf, size_t len, uint32_t timeoutMs) {
    size_t got = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeoutMs);
    while (got < len) {
        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(deadline - now) <= 0) {
            return false;
        }
        uint32_t remain_ms = (uint32_t)((deadline - now) * portTICK_PERIOD_MS);
        if (remain_ms < 1) remain_ms = 1;
        int n = uart_read_bytes((uart_port_t)MCP39F511A_UART_NUM,
                                buf + got, len - got,
                                pdMS_TO_TICKS(remain_ms));
        esp_task_wdt_reset();
        if (n < 0) {
            return false;
        }
        got += (size_t)n;
    }
    return true;
}

uint8_t Mcp39f511aDev::calculateChecksum(const uint8_t *data, uint8_t len) {
    uint16_t checksum = 0;
    for (uint8_t i = 0; i < len; i++) checksum += data[i];
    return (uint8_t)(checksum % 0x0100);
}

// Sends one command frame and waits (bounded by timeoutMs) for a response.
// Confirmed against datasheet Table 4-1: data responses are
// [ACK][Length][Data...][Checksum]; plain ACK is a single 0x06.
uint8_t Mcp39f511aDev::sendCommand(const uint8_t *cmd, uint8_t cmdLen, uint8_t *response, uint8_t *responseLen,
                                    uint32_t timeoutMs, bool expectDataResponse) {
    if (!s_uart_ready) {
        return MCP39F511A_ERROR_TIMEOUT;
    }

    const uart_port_t port = (uart_port_t)MCP39F511A_UART_NUM;
    flushRx();

    for (uint8_t i = 0; i < cmdLen; i++) {
        uart_write_bytes(port, (const char *)&cmd[i], 1);
        esp_rom_delay_us(MCP39F511A_TIMESLOT_US);
    }
    uart_wait_tx_done(port, pdMS_TO_TICKS(timeoutMs));

    uint8_t first = 0;
    if (!readExact(&first, 1, timeoutMs)) {
        return MCP39F511A_ERROR_TIMEOUT;
    }
    if (first == MCP39F511A_NACK) return MCP39F511A_NACK;
    if (first == MCP39F511A_CSFAIL) return MCP39F511A_CSFAIL;
    if (first != MCP39F511A_ACK) return MCP39F511A_ERROR_TIMEOUT;

    if (!expectDataResponse) {
        if (response != nullptr && responseLen != nullptr) {
            response[0] = first;
            *responseLen = 1;
        }
        return MCP39F511A_ACK;
    }

    uint8_t length_byte = 0;
    if (!readExact(&length_byte, 1, timeoutMs)) {
        return MCP39F511A_ERROR_TIMEOUT;
    }
    if (length_byte < 2 || length_byte > 64) {
        return MCP39F511A_ERROR_TIMEOUT;
    }

    uint8_t responseBuffer[64];
    responseBuffer[0] = first;
    responseBuffer[1] = length_byte;
    size_t rest = (size_t)length_byte - 2;
    if (rest > 0 && !readExact(&responseBuffer[2], rest, timeoutMs)) {
        return MCP39F511A_ERROR_TIMEOUT;
    }

    if (response != nullptr && responseLen != nullptr) {
        uint8_t dataToCopy = (length_byte > *responseLen) ? *responseLen : length_byte;
        memcpy(response, responseBuffer, dataToCopy);
        *responseLen = dataToCopy;
    }
    return MCP39F511A_ACK;
}

bool Mcp39f511aDev::readRegister(uint16_t address, uint8_t *data, uint8_t numBytes) {
    uint8_t cmd[8];
    cmd[0] = MCP39F511A_HEADER_BYTE;
    cmd[1] = 8;
    cmd[2] = MCP39F511A_CMD_SET_ADDR_PTR;
    cmd[3] = (address >> 8) & 0xFF;
    cmd[4] = address & 0xFF;
    cmd[5] = MCP39F511A_CMD_REG_READ_N;
    cmd[6] = numBytes;
    cmd[7] = calculateChecksum(cmd, 7);

    uint8_t response[64];
    uint8_t responseLen = sizeof(response);
    if (sendCommand(cmd, 8, response, &responseLen) != MCP39F511A_ACK) return false;
    if (responseLen < (uint8_t)(2 + numBytes + 1)) return false;

    uint8_t recvChecksum = response[responseLen - 1];
    uint8_t calcChecksum = calculateChecksum(response, responseLen - 1);
    if (recvChecksum != calcChecksum) return false;

    memcpy(data, &response[2], numBytes);
    return true;
}

bool Mcp39f511aDev::writeRegister(uint16_t address, const uint8_t *data, uint8_t numBytes) {
    if (numBytes > 16) return false;

    uint8_t cmd[24];
    cmd[0] = MCP39F511A_HEADER_BYTE;
    cmd[1] = 8 + numBytes;
    cmd[2] = MCP39F511A_CMD_SET_ADDR_PTR;
    cmd[3] = (address >> 8) & 0xFF;
    cmd[4] = address & 0xFF;
    cmd[5] = MCP39F511A_CMD_REG_WRITE_N;
    cmd[6] = numBytes;
    memcpy(&cmd[7], data, numBytes);
    cmd[7 + numBytes] = calculateChecksum(cmd, 7 + numBytes);

    uint8_t response[8];
    uint8_t responseLen = sizeof(response);
    uint8_t result = sendCommand(cmd, 8 + numBytes, response, &responseLen,
                                  MCP39F511A_RESPONSE_TIMEOUT_MS, false);
    return result == MCP39F511A_ACK;
}

bool Mcp39f511aDev::writeConfig(const Mcp39f511aConfig &cfg) {
    if (!writeUint32(MCP39F511A_REG_SYS_CONFIG, cfg.sysConfig.uDWord)) return false;
    if (!writeUint32(MCP39F511A_REG_EVENT_CONFIG, cfg.eventConfig.uDWord)) return false;
    return true;
}

bool Mcp39f511aDev::saveToFlash(void) {
    uint8_t cmd[4];
    cmd[0] = MCP39F511A_HEADER_BYTE;
    cmd[1] = 4;
    cmd[2] = MCP39F511A_CMD_SAVE_TO_FLASH;
    cmd[3] = calculateChecksum(cmd, 3);

    uint8_t response[8];
    uint8_t responseLen = sizeof(response);
    return sendCommand(cmd, 4, response, &responseLen, 2000, false) == MCP39F511A_ACK;
}

bool Mcp39f511aDev::readUint16(uint16_t address, uint16_t &out) {
    uint8_t data[2];
    if (!readRegister(address, data, 2)) return false;
    out = (uint16_t)((data[1] << 8) | data[0]);
    return true;
}

bool Mcp39f511aDev::readInt16(uint16_t address, int16_t &out) {
    uint8_t data[2];
    if (!readRegister(address, data, 2)) return false;
    out = (int16_t)((data[1] << 8) | data[0]);
    return true;
}

bool Mcp39f511aDev::readUint32(uint16_t address, uint32_t &out) {
    uint8_t data[4];
    if (!readRegister(address, data, 4)) return false;
    out = ((uint32_t)data[3] << 24) | ((uint32_t)data[2] << 16) |
          ((uint32_t)data[1] << 8) | data[0];
    return true;
}

bool Mcp39f511aDev::readInt32(uint16_t address, int32_t &out) {
    uint32_t raw;
    if (!readUint32(address, raw)) return false;
    out = (int32_t)raw;
    return true;
}

bool Mcp39f511aDev::readUint64(uint16_t address, uint64_t &out) {
    uint8_t data[8];
    if (!readRegister(address, data, 8)) return false;
    out = 0;
    for (int8_t i = 7; i >= 0; i--) out = (out << 8) | data[i];
    return true;
}

bool Mcp39f511aDev::writeUint16(uint16_t address, uint16_t value) {
    const uint8_t data[2] = { (uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF) };
    return writeRegister(address, data, 2);
}

bool Mcp39f511aDev::writeInt16(uint16_t address, int16_t value) {
    return writeUint16(address, (uint16_t)value);
}

bool Mcp39f511aDev::writeUint32(uint16_t address, uint32_t value) {
    const uint8_t data[4] = { (uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF),
                              (uint8_t)((value >> 16) & 0xFF), (uint8_t)((value >> 24) & 0xFF) };
    return writeRegister(address, data, 4);
}

bool Mcp39f511aDev::writeInt32(uint16_t address, int32_t value) {
    return writeUint32(address, (uint32_t)value);
}

bool Mcp39f511aDev::readMeasurements(void) {
    uint16_t vRaw, freqRaw;
    uint32_t iRaw, pRaw, qRaw, sRaw;
    int16_t pfRaw;

    if (!readUint16(MCP39F511A_REG_VRMS, vRaw))           return false;
    if (!readUint32(MCP39F511A_REG_IRMS, iRaw))            return false;
    if (!readUint32(MCP39F511A_REG_POWER_ACTIVE, pRaw))    return false;
    if (!readUint32(MCP39F511A_REG_POWER_REACTIVE, qRaw))  return false;
    if (!readUint32(MCP39F511A_REG_POWER_APPARENT, sRaw))  return false;
    if (!readUint16(MCP39F511A_REG_FREQUENCY, freqRaw))    return false;
    if (!readInt16(MCP39F511A_REG_POWER_FACTOR, pfRaw))    return false;

    measurements_.rmsVoltage = (float)vRaw / 10.0f;
    measurements_.rmsCurrent = (float)iRaw * 0.0001f;
    // Active/Reactive Power are 0.01 W(or VAR)/LSB — see prior live calibration notes.
    measurements_.activePower = (float)pRaw * 0.01f;
    measurements_.reactivePower = (float)qRaw * 0.01f;
    measurements_.apparentPower = (float)sRaw * 0.001f;
    measurements_.frequency = (float)freqRaw / 1000.0f;
    measurements_.powerFactor = pfRaw / 32768.0f;
    measurements_.timestamp = millis();
    measurements_.valid = true;
    return true;
}

bool Mcp39f511aDev::readConfig(void) {
    if (!readUint32(MCP39F511A_REG_SYS_CONFIG, config_.sysConfig.uDWord)) return false;
    if (!readUint32(MCP39F511A_REG_EVENT_CONFIG, config_.eventConfig.uDWord)) return false;
    config_.valid = true;
    return true;
}

bool Mcp39f511aDev::readFullConfig(void) {
    MCP39F511A_FullConfig c = {};

    if (!readUint16(MCP39F511A_REG_AC_GAIN_CURRENT_RMS, c.acGainCurrentRms))       return false;
    if (!readUint16(MCP39F511A_REG_AC_GAIN_VOLTAGE_RMS, c.acGainVoltageRms))       return false;
    if (!readUint16(MCP39F511A_REG_AC_GAIN_ACTIVE_PWR, c.acGainActivePower))       return false;
    if (!readUint16(MCP39F511A_REG_AC_GAIN_REACTIVE_PWR, c.acGainReactivePower))   return false;
    if (!readInt16(MCP39F511A_REG_AC_OFFSET_CURRENT_RMS, c.acOffsetCurrentRms))    return false;
    if (!readInt16(MCP39F511A_REG_AC_OFFSET_ACTIVE_PWR, c.acOffsetActivePower))    return false;
    if (!readInt16(MCP39F511A_REG_AC_OFFSET_REACTIVE_PWR, c.acOffsetReactivePower)) return false;
    if (!readUint16(MCP39F511A_REG_GAIN_LINE_FREQUENCY, c.gainLineFrequency))      return false;
    if (!readInt16(MCP39F511A_REG_PHASE_COMPENSATION, c.phaseCompensation))        return false;
    if (!readUint16(MCP39F511A_REG_VOLTAGE_DROP_COMP, c.voltageDropComp))          return false;
    if (!readUint16(MCP39F511A_REG_INCAP_CURRENT_COMP, c.inCapCurrentComp))        return false;
    if (!readUint16(MCP39F511A_REG_RANGE_VDROP_INCAP_COMP, c.rangeVdropInCapComp)) return false;
    if (!readUint16(MCP39F511A_REG_DC_GAIN_CURRENT_RMS, c.dcGainCurrentRms))       return false;
    if (!readUint16(MCP39F511A_REG_DC_GAIN_VOLTAGE_RMS, c.dcGainVoltageRms))       return false;
    if (!readUint16(MCP39F511A_REG_DC_GAIN_ACTIVE_PWR, c.dcGainActivePower))       return false;
    if (!readInt16(MCP39F511A_REG_DC_OFFSET_CURRENT_RMS, c.dcOffsetCurrentRms))    return false;
    if (!readInt16(MCP39F511A_REG_DC_OFFSET_ACTIVE_PWR, c.dcOffsetActivePower))    return false;
    if (!readUint16(MCP39F511A_REG_OFFCAL_MSB, c.offcalMsb))                       return false;
    if (!readUint16(MCP39F511A_REG_OFFCAL_CH0, c.offcalCh0))                       return false;
    if (!readUint16(MCP39F511A_REG_OFFCAL_CH1, c.offcalCh1))                       return false;
    if (!readUint16(MCP39F511A_REG_TEMP_POS_COMP_FREQ, c.tempPosCompFreq))         return false;
    if (!readUint16(MCP39F511A_REG_TEMP_NEG_COMP_FREQ, c.tempNegCompFreq))         return false;
    if (!readUint16(MCP39F511A_REG_TEMP_POS_COMP_CURRENT, c.tempPosCompCurrent))   return false;
    if (!readUint16(MCP39F511A_REG_TEMP_NEG_COMP_CURRENT, c.tempNegCompCurrent))   return false;
    if (!readUint16(MCP39F511A_REG_TEMP_POS_COMP_POWER, c.tempPosCompPower))       return false;
    if (!readUint16(MCP39F511A_REG_TEMP_NEG_COMP_POWER, c.tempNegCompPower))       return false;
    if (!readUint32(MCP39F511A_REG_SYS_CONFIG, c.sysConfig.uDWord))                return false;
    if (!readUint32(MCP39F511A_REG_EVENT_CONFIG, c.eventConfig.uDWord))            return false;
    if (!readUint32(MCP39F511A_REG_RANGE, c.range))                                return false;
    if (!readUint32(MCP39F511A_REG_CAL_CURRENT, c.calibrationCurrent))             return false;
    if (!readUint16(MCP39F511A_REG_CAL_VOLTAGE, c.calibrationVoltage))             return false;
    if (!readUint32(MCP39F511A_REG_CAL_ACTIVE_PWR, c.calibrationActivePower))      return false;
    if (!readUint32(MCP39F511A_REG_CAL_REACTIVE_PWR, c.calibrationReactivePower))  return false;
    if (!readUint16(MCP39F511A_REG_APPARENT_POWER_DIV, c.apparentPowerDivisor))    return false;
    if (!readUint16(MCP39F511A_REG_ACCUM_INTERVAL, c.accumulatorIntervalParameter)) return false;
    if (!readUint16(MCP39F511A_REG_PWM_PERIOD, c.pwmPeriod))                       return false;
    if (!readUint16(MCP39F511A_REG_PWM_DUTY_CYCLE, c.pwmDutyCycle))                return false;
    if (!readUint16(MCP39F511A_REG_MINMAX_PTR_1, c.minMaxPointer1))                return false;
    if (!readUint16(MCP39F511A_REG_MINMAX_PTR_2, c.minMaxPointer2))                return false;
    if (!readUint16(MCP39F511A_REG_LINE_FREQ_REF, c.lineFrequencyReference))       return false;
    if (!readUint16(MCP39F511A_REG_THERMISTOR_VOLTAGE_CAL, c.thermistorVoltageCal)) return false;
    if (!readUint16(MCP39F511A_REG_VOLTAGE_SAG_LIMIT, c.voltageSagLimit))          return false;
    if (!readUint16(MCP39F511A_REG_VOLTAGE_SURGE_LIMIT, c.voltageSurgeLimit))      return false;
    if (!readUint32(MCP39F511A_REG_OVER_CURRENT_LIMIT, c.overCurrentLimit))        return false;
    if (!readUint32(MCP39F511A_REG_OVER_POWER_LIMIT, c.overPowerLimit))            return false;
    if (!readUint16(MCP39F511A_REG_OVERTEMP_LIMIT, c.overTemperatureLimit))        return false;
    if (!readUint16(MCP39F511A_REG_VOLTAGE_LOW_THRESHOLD, c.voltageLowThreshold))  return false;
    if (!readUint16(MCP39F511A_REG_VOLTAGE_HIGH_THRESHOLD, c.voltageHighThreshold)) return false;
    if (!readUint16(MCP39F511A_REG_NO_LOAD_THRESHOLD, c.noLoadThreshold))          return false;

    c.valid = true;
    fullConfig_ = c;
    return true;
}

bool Mcp39f511aDev::writeFullConfig(const MCP39F511A_FullConfig &cfg) {
    if (!writeUint16(MCP39F511A_REG_AC_GAIN_CURRENT_RMS, cfg.acGainCurrentRms))       return false;
    if (!writeUint16(MCP39F511A_REG_AC_GAIN_VOLTAGE_RMS, cfg.acGainVoltageRms))       return false;
    if (!writeUint16(MCP39F511A_REG_AC_GAIN_ACTIVE_PWR, cfg.acGainActivePower))       return false;
    if (!writeUint16(MCP39F511A_REG_AC_GAIN_REACTIVE_PWR, cfg.acGainReactivePower))   return false;
    if (!writeInt16(MCP39F511A_REG_AC_OFFSET_CURRENT_RMS, cfg.acOffsetCurrentRms))    return false;
    if (!writeInt16(MCP39F511A_REG_AC_OFFSET_ACTIVE_PWR, cfg.acOffsetActivePower))    return false;
    if (!writeInt16(MCP39F511A_REG_AC_OFFSET_REACTIVE_PWR, cfg.acOffsetReactivePower)) return false;
    if (!writeUint16(MCP39F511A_REG_GAIN_LINE_FREQUENCY, cfg.gainLineFrequency))      return false;
    if (!writeInt16(MCP39F511A_REG_PHASE_COMPENSATION, cfg.phaseCompensation))        return false;
    if (!writeUint16(MCP39F511A_REG_VOLTAGE_DROP_COMP, cfg.voltageDropComp))          return false;
    if (!writeUint16(MCP39F511A_REG_INCAP_CURRENT_COMP, cfg.inCapCurrentComp))        return false;
    if (!writeUint16(MCP39F511A_REG_RANGE_VDROP_INCAP_COMP, cfg.rangeVdropInCapComp)) return false;
    if (!writeUint16(MCP39F511A_REG_DC_GAIN_CURRENT_RMS, cfg.dcGainCurrentRms))       return false;
    if (!writeUint16(MCP39F511A_REG_DC_GAIN_VOLTAGE_RMS, cfg.dcGainVoltageRms))       return false;
    if (!writeUint16(MCP39F511A_REG_DC_GAIN_ACTIVE_PWR, cfg.dcGainActivePower))       return false;
    if (!writeInt16(MCP39F511A_REG_DC_OFFSET_CURRENT_RMS, cfg.dcOffsetCurrentRms))    return false;
    if (!writeInt16(MCP39F511A_REG_DC_OFFSET_ACTIVE_PWR, cfg.dcOffsetActivePower))    return false;
    if (!writeUint16(MCP39F511A_REG_OFFCAL_MSB, cfg.offcalMsb))                       return false;
    if (!writeUint16(MCP39F511A_REG_OFFCAL_CH0, cfg.offcalCh0))                       return false;
    if (!writeUint16(MCP39F511A_REG_OFFCAL_CH1, cfg.offcalCh1))                       return false;
    if (!writeUint16(MCP39F511A_REG_TEMP_POS_COMP_FREQ, cfg.tempPosCompFreq))         return false;
    if (!writeUint16(MCP39F511A_REG_TEMP_NEG_COMP_FREQ, cfg.tempNegCompFreq))         return false;
    if (!writeUint16(MCP39F511A_REG_TEMP_POS_COMP_CURRENT, cfg.tempPosCompCurrent))   return false;
    if (!writeUint16(MCP39F511A_REG_TEMP_NEG_COMP_CURRENT, cfg.tempNegCompCurrent))   return false;
    if (!writeUint16(MCP39F511A_REG_TEMP_POS_COMP_POWER, cfg.tempPosCompPower))       return false;
    if (!writeUint16(MCP39F511A_REG_TEMP_NEG_COMP_POWER, cfg.tempNegCompPower))       return false;
    if (!writeUint32(MCP39F511A_REG_SYS_CONFIG, cfg.sysConfig.uDWord))                return false;
    if (!writeUint32(MCP39F511A_REG_EVENT_CONFIG, cfg.eventConfig.uDWord))            return false;
    if (!writeUint32(MCP39F511A_REG_RANGE, cfg.range))                                return false;
    if (!writeUint32(MCP39F511A_REG_CAL_CURRENT, cfg.calibrationCurrent))             return false;
    if (!writeUint16(MCP39F511A_REG_CAL_VOLTAGE, cfg.calibrationVoltage))             return false;
    if (!writeUint32(MCP39F511A_REG_CAL_ACTIVE_PWR, cfg.calibrationActivePower))      return false;
    if (!writeUint32(MCP39F511A_REG_CAL_REACTIVE_PWR, cfg.calibrationReactivePower))  return false;
    if (!writeUint16(MCP39F511A_REG_APPARENT_POWER_DIV, cfg.apparentPowerDivisor))    return false;
    if (!writeUint16(MCP39F511A_REG_ACCUM_INTERVAL, cfg.accumulatorIntervalParameter)) return false;
    if (!writeUint16(MCP39F511A_REG_PWM_PERIOD, cfg.pwmPeriod))                       return false;
    if (!writeUint16(MCP39F511A_REG_PWM_DUTY_CYCLE, cfg.pwmDutyCycle))                return false;
    if (!writeUint16(MCP39F511A_REG_MINMAX_PTR_1, cfg.minMaxPointer1))                return false;
    if (!writeUint16(MCP39F511A_REG_MINMAX_PTR_2, cfg.minMaxPointer2))                return false;
    if (!writeUint16(MCP39F511A_REG_LINE_FREQ_REF, cfg.lineFrequencyReference))       return false;
    if (!writeUint16(MCP39F511A_REG_THERMISTOR_VOLTAGE_CAL, cfg.thermistorVoltageCal)) return false;
    if (!writeUint16(MCP39F511A_REG_VOLTAGE_SAG_LIMIT, cfg.voltageSagLimit))          return false;
    if (!writeUint16(MCP39F511A_REG_VOLTAGE_SURGE_LIMIT, cfg.voltageSurgeLimit))      return false;
    if (!writeUint32(MCP39F511A_REG_OVER_CURRENT_LIMIT, cfg.overCurrentLimit))        return false;
    if (!writeUint32(MCP39F511A_REG_OVER_POWER_LIMIT, cfg.overPowerLimit))            return false;
    if (!writeUint16(MCP39F511A_REG_OVERTEMP_LIMIT, cfg.overTemperatureLimit))        return false;
    if (!writeUint16(MCP39F511A_REG_VOLTAGE_LOW_THRESHOLD, cfg.voltageLowThreshold))  return false;
    if (!writeUint16(MCP39F511A_REG_VOLTAGE_HIGH_THRESHOLD, cfg.voltageHighThreshold)) return false;
    if (!writeUint16(MCP39F511A_REG_NO_LOAD_THRESHOLD, cfg.noLoadThreshold))          return false;

    return saveToFlash();
}

bool Mcp39f511aDev::readEnergyCounters(void) {
    Mcp39f511aEnergyCounters e = {};
    if (!readUint64(MCP39F511A_REG_IMPORT_ACTIVE_ENERGY, e.importActiveEnergy))     return false;
    if (!readUint64(MCP39F511A_REG_EXPORT_ACTIVE_ENERGY, e.exportActiveEnergy))     return false;
    if (!readUint64(MCP39F511A_REG_IMPORT_REACTIVE_ENERGY, e.importReactiveEnergy)) return false;
    if (!readUint64(MCP39F511A_REG_EXPORT_REACTIVE_ENERGY, e.exportReactiveEnergy)) return false;
    e.valid = true;
    energyCounters_ = e;
    return true;
}

bool Mcp39f511aDev::resetEnergyCounters(void) {
    // Datasheet Register 6-2 ENRG_CNTRL: clearing the bit turns accumulation
    // off AND zeroes all four energy registers. Re-enable afterward. Energy
    // is also mirrored in EEPROM (auto-load at power-on), so after RAM reset
    // Save Energy Counters to EEPROM (cmd 0x45) so a later power cycle does
    // not restore the pre-reset totals.
    //
    // Must poll until the counters actually read back as zero before Save /
    // re-enable: a fixed short delay is enough when DEBUG_SERIAL_ENABLED=1
    // (Serial.printf between the writes stalls the sequence) but too short
    // on a nodebug build — found live 2026-09-29: nodebug /energy/reset
    // returned ok while the chip still held the old totals, then Save Energy
    // persisted those old totals and the next PowerMeter read restored them.
    Mcp39f511aSysConfig sys;
    if (!readUint32(MCP39F511A_REG_SYS_CONFIG, sys.uDWord)) {
        SLOG_PRINTLN("[MCP39F511A] resetEnergyCounters: initial read FAILED");
        return false;
    }
    SLOG_PRINTF("[MCP39F511A] resetEnergyCounters: read sysConfig=0x%08lX\n", (unsigned long)sys.uDWord);

    sys.ENRG_CNTRL = 0;
    if (!writeUint32(MCP39F511A_REG_SYS_CONFIG, sys.uDWord)) {
        SLOG_PRINTLN("[MCP39F511A] resetEnergyCounters: write (ENRG_CNTRL=0) FAILED");
        return false;
    }

    bool zeroed = false;
    for (int attempt = 0; attempt < 50; attempt++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (!readEnergyCounters()) {
            SLOG_PRINTLN("[MCP39F511A] resetEnergyCounters: zero-poll read FAILED");
            return false;
        }
        const Mcp39f511aEnergyCounters *e = &energyCounters_;
        if (e->importActiveEnergy == 0 && e->exportActiveEnergy == 0 &&
            e->importReactiveEnergy == 0 && e->exportReactiveEnergy == 0) {
            zeroed = true;
            break;
        }
    }
    if (!zeroed) {
        SLOG_PRINTLN("[MCP39F511A] resetEnergyCounters: counters did not clear");
        // Leave ENRG_CNTRL=0? Better restore accumulation so the meter keeps
        // working even if reset failed mid-way.
        sys.ENRG_CNTRL = 1;
        (void)writeUint32(MCP39F511A_REG_SYS_CONFIG, sys.uDWord);
        return false;
    }

    // Persist zeros while accumulation is still off.
    uint8_t cmd[4];
    cmd[0] = MCP39F511A_HEADER_BYTE;
    cmd[1] = 4;
    cmd[2] = MCP39F511A_CMD_SAVE_ENERGY_EEPROM;
    cmd[3] = calculateChecksum(cmd, 3);
    uint8_t response[8];
    uint8_t responseLen = sizeof(response);
    if (sendCommand(cmd, 4, response, &responseLen, 2000, false) != MCP39F511A_ACK) {
        SLOG_PRINTLN("[MCP39F511A] resetEnergyCounters: SAVE_ENERGY_EEPROM FAILED");
        sys.ENRG_CNTRL = 1;
        (void)writeUint32(MCP39F511A_REG_SYS_CONFIG, sys.uDWord);
        return false;
    }

    sys.ENRG_CNTRL = 1;
    if (!writeUint32(MCP39F511A_REG_SYS_CONFIG, sys.uDWord)) {
        SLOG_PRINTLN("[MCP39F511A] resetEnergyCounters: write (ENRG_CNTRL=1) FAILED");
        return false;
    }
    return true;
}
