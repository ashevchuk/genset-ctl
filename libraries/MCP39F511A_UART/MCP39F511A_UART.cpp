/*********************************************************************
 *
 *       MCP39F511A UART Library for ESP32 - Implementation
 *
 *********************************************************************
 * FileName:        MCP39F511A_UART.cpp
 * Revision:        1.0.0
 * Date:            2024
 *
 * Description:     Simplified UART-only library for MCP39F511A power monitoring IC
 *                  Based on original library by Futura Group srl
 *
 **********************************************************************/

#include "MCP39F511A_UART.h"

//======================================================================
//  Constructor
MCP39F511A_UART::MCP39F511A_UART(HardwareSerial *serial, uint8_t rxPin, uint8_t txPin, uint32_t baudRate) {
    _serial = serial;
    _rxPin = rxPin;
    _txPin = txPin;
    _baudRate = baudRate;
    
    // Initialize data structures
    memset(&_measurements, 0, sizeof(_measurements));
    memset(&_config, 0, sizeof(_config));
    memset(&_calibration, 0, sizeof(_calibration));
}

//======================================================================
//  Initialization
bool MCP39F511A_UART::begin() {
    if (_serial == nullptr) {
        return false;
    }
    
    _serial->begin(_baudRate, SERIAL_8N1, _rxPin, _txPin);
    delay(100);  // Allow time for serial port to stabilize
    
    clearSerialBuffer();
    return true;
}

void MCP39F511A_UART::end() {
    if (_serial != nullptr) {
        _serial->end();
    }
}

//======================================================================
//  Clear serial buffer
void MCP39F511A_UART::clearSerialBuffer() {
    while (_serial->available()) {
        _serial->read();
    }
}

//======================================================================
//  Calculate checksum
uint8_t MCP39F511A_UART::calculateChecksum(uint8_t *data, uint8_t len) {
    uint16_t checksum = 0;
    for (uint8_t i = 0; i < len; i++) {
        checksum += data[i];
    }
    return (uint8_t)(checksum % 0x0100);
}

//======================================================================
//  Send command and receive response
uint8_t MCP39F511A_UART::sendCommand(uint8_t *cmd, uint8_t cmdLen, uint8_t *response, uint8_t *responseLen, uint16_t timeout_ms) {
    // Clear serial buffer
    clearSerialBuffer();
    
    // Send command
    for (uint8_t i = 0; i < cmdLen; i++) {
        _serial->write(cmd[i]);
        delayMicroseconds(MCP39F511A_TIMESLOT_US);
    }
    
    // Wait for response - format is [ACK][Length][Data...][Checksum]
    uint32_t startTime = millis();
    uint8_t responseBuffer[64];
    uint8_t bufferIndex = 0;
    
    while ((millis() - startTime) < timeout_ms) {
        if (_serial->available() > 0) {
            uint8_t byte = _serial->read();
            if (bufferIndex < sizeof(responseBuffer)) {
                responseBuffer[bufferIndex++] = byte;
            }
            
            // Wait a bit to collect all response bytes
            delay(10);
            
            // Check if we have at least the ACK and Length bytes
            if (bufferIndex >= 2) {
                uint8_t firstByte = responseBuffer[0];
                
                if (firstByte == MCP39F511A_ACK) {
                    uint8_t pktLength = responseBuffer[1];  // Total length including ACK, Length, Data, Checksum
                    
                    // Wait for the complete packet to arrive
                    while (_serial->available() && bufferIndex < pktLength) {
                        if (bufferIndex < sizeof(responseBuffer)) {
                            responseBuffer[bufferIndex++] = _serial->read();
                        }
                        delay(1);
                    }
                    
                    if (response != nullptr && responseLen != nullptr && bufferIndex >= pktLength) {
                        // Copy the entire received packet to the response buffer
                        uint8_t dataToCopy = (pktLength > *responseLen) ? *responseLen : pktLength;
                        memcpy(response, responseBuffer, dataToCopy);
                        *responseLen = dataToCopy;
                    }
                    return MCP39F511A_ACK;
                } else if (firstByte == MCP39F511A_NACK) {
                    return MCP39F511A_NACK;
                } else if (firstByte == MCP39F511A_CSFAIL) {
                    return MCP39F511A_CSFAIL;
                }
            }
        }
        delay(1);
    }
    
    return 0;  // Timeout
}

//======================================================================
//  Read register
bool MCP39F511A_UART::readRegister(uint16_t address, uint8_t *data, uint8_t numBytes) {
    uint8_t cmd[8];
    uint8_t response[64];
    uint8_t responseLen = sizeof(response);
    
    // Build command: Set Address Pointer + Read N bytes
    // Format: [0xA5][8][0x41][AddrH][AddrL][0x4E][N][Checksum]
    cmd[0] = MCP39F511A_HEADER_BYTE;
    cmd[1] = 8;  // Frame length
    cmd[2] = MCP39F511A_CMD_SET_ADDR_PTR;
    cmd[3] = (address >> 8) & 0xFF;  // Address High (MSB)
    cmd[4] = address & 0xFF;        // Address Low (LSB)
    cmd[5] = MCP39F511A_CMD_REG_READ_N;
    cmd[6] = numBytes;
    cmd[7] = calculateChecksum(cmd, 7);
    
    uint8_t result = sendCommand(cmd, 8, response, &responseLen, MCP39F511A_RESPONSE_TIMEOUT_MS);
    
    if (result == MCP39F511A_ACK) {
        // Response format: [ACK][Length][Data...][Checksum]
        // Data starts at index 2
        if (responseLen >= (2 + numBytes + 1)) {  // ACK + Length + Data + Checksum
            // Verify checksum
            uint8_t recvChecksum = response[responseLen - 1];
            uint8_t calcChecksum = calculateChecksum(response, responseLen - 1);
            
            if (recvChecksum == calcChecksum) {
                // Extract data (skip ACK[0] and Length[1], data starts at [2])
                memcpy(data, &response[2], numBytes);
                return true;
            }
        }
    }
    
    return false;
}

//======================================================================
//  Write register
bool MCP39F511A_UART::writeRegister(uint16_t address, uint8_t *data, uint8_t numBytes) {
    if (numBytes > 16) return false;  // Maximum write size
    
    uint8_t cmd[24];  // Max command size
    uint8_t response[8];
    uint8_t responseLen = sizeof(response);
    
    // Build command: Set Address Pointer + Write N bytes
    // Format: [0xA5][5+N][0x41][AddrH][AddrL][0x4D][N][Data...][Checksum]
    cmd[0] = MCP39F511A_HEADER_BYTE;
    cmd[1] = 5 + numBytes;  // Frame length
    cmd[2] = MCP39F511A_CMD_SET_ADDR_PTR;
    cmd[3] = (address >> 8) & 0xFF;  // Address High (MSB)
    cmd[4] = address & 0xFF;        // Address Low (LSB)
    cmd[5] = MCP39F511A_CMD_REG_WRITE_N;
    cmd[6] = numBytes;
    
    // Copy data (LSB first as per library)
    for (uint8_t i = 0; i < numBytes; i++) {
        cmd[7 + i] = data[i];
    }
    
    // Calculate checksum (calc from bytes 0 to (6 + numBytes))
    cmd[7 + numBytes] = calculateChecksum(cmd, 7 + numBytes);
    
    uint8_t result = sendCommand(cmd, 8 + numBytes, response, &responseLen, MCP39F511A_RESPONSE_TIMEOUT_MS);
    
    if (result == MCP39F511A_ACK) {
        // For write commands, the response is typically [ACK][Length][Checksum]
        if (responseLen >= 3) {  // ACK + Length + Checksum
            uint8_t recvChecksum = response[responseLen - 1];
            uint8_t calcChecksum = calculateChecksum(response, responseLen - 1);
            if (recvChecksum == calcChecksum) {
                return true;
            }
        }
    }
    
    return false;
}

//======================================================================
//  Save to flash
bool MCP39F511A_UART::saveToFlash() {
    uint8_t cmd[4];
    uint8_t response[8];
    uint8_t responseLen = sizeof(response);
    
    // Format: [0xA5][4][0x53][Checksum]
    cmd[0] = MCP39F511A_HEADER_BYTE;
    cmd[1] = 4;  // Frame length
    cmd[2] = MCP39F511A_CMD_SAVE_TO_FLASH;
    cmd[3] = calculateChecksum(cmd, 3);
    
    uint8_t result = sendCommand(cmd, 4, response, &responseLen, 2000);  // Longer timeout for flash operation
    
    if (result == MCP39F511A_ACK) {
        if (responseLen >= 3) {  // ACK + Length + Checksum
            uint8_t recvChecksum = response[responseLen - 1];
            uint8_t calcChecksum = calculateChecksum(response, responseLen - 1);
            if (recvChecksum == calcChecksum) {
                return true;
            }
        }
    }
    
    return false;
}

//======================================================================
//  Helper functions for reading different data types
int16_t MCP39F511A_UART::readInt16(uint16_t address) {
    uint8_t data[2];
    if (readRegister(address, data, 2)) {
        // Data is stored LSB first (Byte0 = LSB, Byte1 = MSB)
        return (int16_t)((data[1] << 8) | data[0]);
    }
    return 0;
}

uint16_t MCP39F511A_UART::readUint16(uint16_t address) {
    uint8_t data[2];
    if (readRegister(address, data, 2)) {
        // Data is stored LSB first (Byte0 = LSB, Byte1 = MSB)
        return (uint16_t)((data[1] << 8) | data[0]);
    }
    return 0;
}

uint32_t MCP39F511A_UART::readUint32(uint16_t address) {
    uint8_t data[4];
    if (readRegister(address, data, 4)) {
        // Data is stored LSB first (Byte0 = LSB, Byte1, Byte2, Byte3 = MSB)
        return (uint32_t)((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);
    }
    return 0;
}

//======================================================================
//  Read all measurements
bool MCP39F511A_UART::readMeasurements() {
    // Read RMS Voltage (16-bit unsigned, address 0x0006)
    uint16_t vRaw = readUint16(MCP39F511A_REG_VRMS);
    _measurements.rmsVoltage = (float)vRaw / 10.0f;  // Adjusted scaling
    
    // Read RMS Current (32-bit unsigned, address 0x000E)
    uint32_t iRaw = readUint32(MCP39F511A_REG_IRMS);
    _measurements.rmsCurrent = (float)iRaw * 0.0001f;  // 1 LSB = 0.0001A
    
    // Read Active Power (32-bit unsigned, address 0x0012)
    uint32_t pRaw = readUint32(MCP39F511A_REG_POWER_ACTIVE);
    _measurements.activePower = (float)pRaw * 0.001f;  // 1 LSB = 0.001W
    
    // Read Reactive Power (32-bit unsigned, address 0x0016)
    uint32_t qRaw = readUint32(MCP39F511A_REG_POWER_REACTIVE);
    _measurements.reactivePower = (float)qRaw * 0.001f;
    
    // Read Apparent Power (32-bit unsigned, address 0x001A)
    uint32_t sRaw = readUint32(MCP39F511A_REG_POWER_APPARENT);
    _measurements.apparentPower = (float)sRaw * 0.001f;
    
    // Read Frequency (16-bit unsigned, address 0x0008)
    uint16_t freqRaw = readUint16(MCP39F511A_REG_FREQUENCY);
    _measurements.frequency = (float)freqRaw / 1000.0f;  // Divide by 10^3
    
    // Read Power Factor (16-bit signed, address 0x000C)
    int16_t pfRaw = readInt16(MCP39F511A_REG_POWER_FACTOR);
    _measurements.powerFactor = pfRaw / 32768.0f;  // Scaling: 0x7FFF = +1.0, 0x8000 = -1.0
    
    _measurements.timestamp = millis();
    _measurements.valid = true;
    
    return true;
}

MCP39F511A_Measurements* MCP39F511A_UART::getMeasurements() {
    return &_measurements;
}

//======================================================================
//  Configuration functions
bool MCP39F511A_UART::readConfig() {
    // Read System Configuration Register (32-bit)
    // Data is stored LSB first (Byte0 = LSB, Byte1, Byte2, Byte3 = MSB)
    uint8_t sysData[4];
    if (!readRegister(MCP39F511A_REG_SYS_CONFIG, sysData, 4)) {
        return false;
    }
    _config.sysConfig.Byte0 = sysData[0];  // LSB
    _config.sysConfig.Byte1 = sysData[1];
    _config.sysConfig.Byte2 = sysData[2];
    _config.sysConfig.Byte3 = sysData[3];  // MSB
    
    // Read Event Configuration Register (32-bit)
    // Data is stored LSB first (Byte0 = LSB, Byte1, Byte2, Byte3 = MSB)
    uint8_t eventData[4];
    if (!readRegister(MCP39F511A_REG_EVENT_CONFIG, eventData, 4)) {
        return false;
    }
    _config.eventConfig.Byte0 = eventData[0];  // LSB
    _config.eventConfig.Byte1 = eventData[1];
    _config.eventConfig.Byte2 = eventData[2];
    _config.eventConfig.Byte3 = eventData[3];  // MSB
    
    _config.valid = true;
    return true;
}

bool MCP39F511A_UART::writeConfig() {
    bool success = true;
    
    // Write System Configuration Register (32-bit)
    // Data is stored LSB first (Byte0 = LSB, Byte1, Byte2, Byte3 = MSB)
    uint8_t sysData[4] = {
        _config.sysConfig.Byte0,  // LSB
        _config.sysConfig.Byte1,
        _config.sysConfig.Byte2,
        _config.sysConfig.Byte3   // MSB
    };
    if (!writeRegister(MCP39F511A_REG_SYS_CONFIG, sysData, 4)) {
        success = false;
    }
    delay(10);
    
    // Write Event Configuration Register (32-bit)
    // Data is stored LSB first (Byte0 = LSB, Byte1, Byte2, Byte3 = MSB)
    uint8_t eventData[4] = {
        _config.eventConfig.Byte0,  // LSB
        _config.eventConfig.Byte1,
        _config.eventConfig.Byte2,
        _config.eventConfig.Byte3   // MSB
    };
    if (!writeRegister(MCP39F511A_REG_EVENT_CONFIG, eventData, 4)) {
        success = false;
    }
    
    return success;
}

MCP39F511A_Config* MCP39F511A_UART::getConfig() {
    return &_config;
}

//======================================================================
//  Calibration functions
bool MCP39F511A_UART::readCalibration() {
    // Read all calibration registers (AC mode)
    _calibration.calDelimiter = readUint16(MCP39F511A_REG_CAL_DELIMITER);
    _calibration.gainCurrentRMS = readUint16(MCP39F511A_REG_GAIN_CURRENT_RMS);
    _calibration.gainVoltageRMS = readUint16(MCP39F511A_REG_GAIN_VOLTAGE_RMS);
    _calibration.gainActivePower = readUint16(MCP39F511A_REG_GAIN_ACTIVE_PWR);
    _calibration.gainReactivePower = readUint16(MCP39F511A_REG_GAIN_REACTIVE_PWR);
    _calibration.offsetCurrentRMS = readInt16(MCP39F511A_REG_OFFSET_CURRENT_RMS);
    _calibration.offsetActivePower = readInt16(MCP39F511A_REG_OFFSET_ACTIVE_PWR);
    _calibration.offsetReactivePower = readInt16(MCP39F511A_REG_OFFSET_REACTIVE_PWR);
    _calibration.gainFrequency = readUint16(MCP39F511A_REG_GAIN_FREQUENCY);
    _calibration.phaseCompensation = readInt16(MCP39F511A_REG_PHASE_COMP);
    
    _calibration.valid = true;
    return true;
}

bool MCP39F511A_UART::writeCalibration() {
    bool success = true;
    
    // Write calibration delimiter
    uint8_t calDelimData[2] = {
        _calibration.calDelimiter & 0xFF,            // LSB
        (_calibration.calDelimiter >> 8) & 0xFF      // MSB
    };
    if (!writeRegister(MCP39F511A_REG_CAL_DELIMITER, calDelimData, 2)) success = false;
    delay(10);
    
    // Write gain registers (unsigned 16-bit)
    uint8_t gainData[2];
    gainData[0] = _calibration.gainCurrentRMS & 0xFF;
    gainData[1] = (_calibration.gainCurrentRMS >> 8) & 0xFF;
    if (!writeRegister(MCP39F511A_REG_GAIN_CURRENT_RMS, gainData, 2)) success = false;
    delay(10);
    
    gainData[0] = _calibration.gainVoltageRMS & 0xFF;
    gainData[1] = (_calibration.gainVoltageRMS >> 8) & 0xFF;
    if (!writeRegister(MCP39F511A_REG_GAIN_VOLTAGE_RMS, gainData, 2)) success = false;
    delay(10);
    
    gainData[0] = _calibration.gainActivePower & 0xFF;
    gainData[1] = (_calibration.gainActivePower >> 8) & 0xFF;
    if (!writeRegister(MCP39F511A_REG_GAIN_ACTIVE_PWR, gainData, 2)) success = false;
    delay(10);
    
    gainData[0] = _calibration.gainReactivePower & 0xFF;
    gainData[1] = (_calibration.gainReactivePower >> 8) & 0xFF;
    if (!writeRegister(MCP39F511A_REG_GAIN_REACTIVE_PWR, gainData, 2)) success = false;
    delay(10);
    
    // Write offset registers (signed 16-bit)
    uint8_t offsetData[2];
    offsetData[0] = _calibration.offsetCurrentRMS & 0xFF;
    offsetData[1] = (_calibration.offsetCurrentRMS >> 8) & 0xFF;
    if (!writeRegister(MCP39F511A_REG_OFFSET_CURRENT_RMS, offsetData, 2)) success = false;
    delay(10);
    
    offsetData[0] = _calibration.offsetActivePower & 0xFF;
    offsetData[1] = (_calibration.offsetActivePower >> 8) & 0xFF;
    if (!writeRegister(MCP39F511A_REG_OFFSET_ACTIVE_PWR, offsetData, 2)) success = false;
    delay(10);
    
    offsetData[0] = _calibration.offsetReactivePower & 0xFF;
    offsetData[1] = (_calibration.offsetReactivePower >> 8) & 0xFF;
    if (!writeRegister(MCP39F511A_REG_OFFSET_REACTIVE_PWR, offsetData, 2)) success = false;
    delay(10);
    
    // Write frequency gain
    gainData[0] = _calibration.gainFrequency & 0xFF;
    gainData[1] = (_calibration.gainFrequency >> 8) & 0xFF;
    if (!writeRegister(MCP39F511A_REG_GAIN_FREQUENCY, gainData, 2)) success = false;
    delay(10);
    
    // Write phase compensation (signed 16-bit)
    offsetData[0] = _calibration.phaseCompensation & 0xFF;
    offsetData[1] = (_calibration.phaseCompensation >> 8) & 0xFF;
    if (!writeRegister(MCP39F511A_REG_PHASE_COMP, offsetData, 2)) success = false;
    
    return success;
}

MCP39F511A_Calibration* MCP39F511A_UART::getCalibration() {
    return &_calibration;
}
