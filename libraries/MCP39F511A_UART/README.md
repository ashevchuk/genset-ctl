# MCP39F511A_UART Library for ESP32

Simplified UART-only library for MCP39F511A power monitoring IC, adapted for ESP32.

## Description

This library provides a simplified interface to communicate with the MCP39F511A power monitoring IC via UART. It is based on the original library by Futura Group srl, but simplified to work only through UART without requiring ISR or Timer functionality.

## Features

- UART communication only (no ISR/Timer dependencies)
- Read power measurements (voltage, current, power, frequency, power factor)
- Read/write configuration registers
- Read/write calibration registers
- Save configuration to flash
- ESP32 compatible

## Hardware Requirements

- ESP32 development board
- MCP39F511A power monitoring IC
- UART connection between ESP32 and MCP39F511A

## Wiring

```
MCP39F511A    ESP32
-----------   -----
TX       ->   GPIO 16 (Serial2 RX)
RX       ->   GPIO 17 (Serial2 TX)
GND      ->   GND
VDD      ->   3.3V or 5V (check datasheet)
```

**Note:** Adjust Serial2 RX/TX pins according to your ESP32 board configuration.

## Installation

1. Copy the `MCP39F511A_UART` folder to your Arduino `libraries` directory
2. Restart Arduino IDE
3. Include the library in your sketch: `#include "MCP39F511A_UART.h"`

## Usage

### Basic Example

```cpp
#include "MCP39F511A_UART.h"

#define MCP39F511A_RX_PIN  16
#define MCP39F511A_TX_PIN  17
#define MCP39F511A_BAUD    9600

MCP39F511A_UART mcp39f511a(&Serial2, MCP39F511A_RX_PIN, MCP39F511A_TX_PIN, MCP39F511A_BAUD);

void setup() {
    Serial.begin(115200);
    
    if (mcp39f511a.begin()) {
        Serial.println("MCP39F511A initialized");
    }
}

void loop() {
    if (mcp39f511a.readMeasurements()) {
        MCP39F511A_Measurements* data = mcp39f511a.getMeasurements();
        
        Serial.print("Voltage: ");
        Serial.print(data->rmsVoltage, 2);
        Serial.println(" V");
        
        Serial.print("Current: ");
        Serial.print(data->rmsCurrent, 4);
        Serial.println(" A");
        
        Serial.print("Power: ");
        Serial.print(data->activePower, 3);
        Serial.println(" W");
    }
    
    delay(2000);
}
```

## API Reference

### Constructor

```cpp
MCP39F511A_UART(HardwareSerial *serial, uint8_t rxPin, uint8_t txPin, uint32_t baudRate = 9600)
```

### Initialization

```cpp
bool begin()
void end()
```

### Measurement Functions

```cpp
bool readMeasurements()
MCP39F511A_Measurements* getMeasurements()
```

### Configuration Functions

```cpp
bool readConfig()
bool writeConfig()
MCP39F511A_Config* getConfig()
```

### Calibration Functions

```cpp
bool readCalibration()
bool writeCalibration()
MCP39F511A_Calibration* getCalibration()
```

### Low-level Functions

```cpp
bool readRegister(uint16_t address, uint8_t *data, uint8_t numBytes)
bool writeRegister(uint16_t address, uint8_t *data, uint8_t numBytes)
bool saveToFlash()
int16_t readInt16(uint16_t address)
uint16_t readUint16(uint16_t address)
uint32_t readUint32(uint16_t address)
```

## Data Structures

### MCP39F511A_Measurements

```cpp
typedef struct {
    float rmsVoltage;        // RMS Voltage (V)
    float rmsCurrent;        // RMS Current (A)
    float activePower;       // Active Power (W)
    float reactivePower;     // Reactive Power (VAR)
    float apparentPower;     // Apparent Power (VA)
    float frequency;         // Line Frequency (Hz)
    float powerFactor;       // Power Factor
    uint32_t timestamp;      // Timestamp of last reading
    bool valid;              // Data validity flag
} MCP39F511A_Measurements;
```

### MCP39F511A_Config

```cpp
typedef struct {
    uint32_t sysConfig;      // System Configuration Register (32-bit)
    uint32_t eventConfig;   // Event Configuration Register (32-bit)
    bool valid;              // Configuration validity flag
} MCP39F511A_Config;
```

### MCP39F511A_Calibration

```cpp
typedef struct {
    uint16_t calDelimiter;
    uint16_t gainCurrentRMS;
    uint16_t gainVoltageRMS;
    uint16_t gainActivePower;
    uint16_t gainReactivePower;
    int16_t offsetCurrentRMS;
    int16_t offsetActivePower;
    int16_t offsetReactivePower;
    uint16_t gainFrequency;
    int16_t phaseCompensation;
    bool valid;
} MCP39F511A_Calibration;
```

## Register Addresses

Common register addresses are defined as constants:

- `MCP39F511A_REG_VRMS` - RMS Voltage (0x0006)
- `MCP39F511A_REG_IRMS` - RMS Current (0x000E)
- `MCP39F511A_REG_POWER_ACTIVE` - Active Power (0x0012)
- `MCP39F511A_REG_POWER_REACTIVE` - Reactive Power (0x0016)
- `MCP39F511A_REG_POWER_APPARENT` - Apparent Power (0x001A)
- `MCP39F511A_REG_FREQUENCY` - Line Frequency (0x0008)
- `MCP39F511A_REG_POWER_FACTOR` - Power Factor (0x000C)
- `MCP39F511A_REG_SYS_CONFIG` - System Configuration (0x007A)
- `MCP39F511A_REG_EVENT_CONFIG` - Event Configuration (0x007E)

See the header file for a complete list of register addresses.

## Error Codes

- `MCP39F511A_OK` - Operation successful
- `MCP39F511A_ERROR_RECEIVED_LESS` - Received less data than expected
- `MCP39F511A_ERROR_CHECKSUM` - Checksum error
- `MCP39F511A_ERROR_NO_ACK` - No acknowledge received
- `MCP39F511A_ERROR_TIMEOUT` - Operation timeout

## Notes

- The library uses LSB-first byte order for multi-byte values (as per the original library)
- Voltage scaling is adjusted to divide by 10 (instead of 100) based on actual device behavior
- Frequency scaling divides by 1000 (as per library specification)
- All timing constants can be adjusted if needed for different hardware configurations

## License

Based on the original library by Futura Group srl. See original library files for license information.

## Credits

- Original library: Futura Group srl (www.Futurashop.it, www.open-electronics.org)
- Developer: Destro Matteo
- ESP32 adaptation: 2024
