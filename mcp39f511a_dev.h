/*
 * mcp39f511a_dev.h — MCP39F511A power-meter driver over ESP-IDF UART.
 *
 * Request/response protocol only (no RX event task). Transport is UART_NUM_2
 * via uart_write_bytes / uart_read_bytes with timeouts — not Arduino
 * HardwareSerial.
 *
 * Register map past the measurement block is project-owned and verified
 * against doc/MCP39F511A-Data-Sheet-20006044A.pdf (the old vendored
 * MCP39F511A_UART.h addresses for config/calibration did not match).
 */

#ifndef MCP39F511A_DEV_H
#define MCP39F511A_DEV_H

#include <stdint.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

// Protocol constants (datasheet Table 4-x)
#define MCP39F511A_ACK                  0x06
#define MCP39F511A_NACK                 0x15
#define MCP39F511A_CSFAIL               0x51
#define MCP39F511A_ERROR_TIMEOUT        0x04

#define MCP39F511A_CMD_SET_ADDR_PTR     0x41
#define MCP39F511A_CMD_REG_READ_N       0x4E
#define MCP39F511A_CMD_REG_WRITE_N      0x4D
#define MCP39F511A_CMD_SAVE_ENERGY_EEPROM 0x45  // Table 4-1 #11 — persist energy counters
#define MCP39F511A_CMD_SAVE_TO_FLASH    0x53
#define MCP39F511A_HEADER_BYTE          0xA5

#define MCP39F511A_TIMESLOT_US          100
#define MCP39F511A_RESPONSE_TIMEOUT_MS  500

// Measurement / energy registers — Table 6-1
#define MCP39F511A_REG_VRMS                   0x0006
#define MCP39F511A_REG_FREQUENCY              0x0008
#define MCP39F511A_REG_POWER_FACTOR           0x000C
#define MCP39F511A_REG_IRMS                   0x000E
#define MCP39F511A_REG_POWER_ACTIVE           0x0012
#define MCP39F511A_REG_POWER_REACTIVE         0x0016
#define MCP39F511A_REG_POWER_APPARENT         0x001A
#define MCP39F511A_REG_IMPORT_ACTIVE_ENERGY   0x001E
#define MCP39F511A_REG_EXPORT_ACTIVE_ENERGY   0x0026
#define MCP39F511A_REG_IMPORT_REACTIVE_ENERGY 0x002E
#define MCP39F511A_REG_EXPORT_REACTIVE_ENERGY 0x0036

// Config / calibration — datasheet addresses (not the buggy vendor header)
#define MCP39F511A_REG_CAL_DELIMITER          0x004E
#define MCP39F511A_REG_AC_GAIN_CURRENT_RMS    0x0050
#define MCP39F511A_REG_AC_GAIN_VOLTAGE_RMS    0x0052
#define MCP39F511A_REG_AC_GAIN_ACTIVE_PWR     0x0054
#define MCP39F511A_REG_AC_GAIN_REACTIVE_PWR   0x0056
#define MCP39F511A_REG_AC_OFFSET_CURRENT_RMS  0x005A
#define MCP39F511A_REG_AC_OFFSET_ACTIVE_PWR   0x005C
#define MCP39F511A_REG_AC_OFFSET_REACTIVE_PWR 0x005E
#define MCP39F511A_REG_GAIN_LINE_FREQUENCY    0x0060
#define MCP39F511A_REG_PHASE_COMPENSATION     0x0062
#define MCP39F511A_REG_VOLTAGE_DROP_COMP      0x0064
#define MCP39F511A_REG_INCAP_CURRENT_COMP     0x0066
#define MCP39F511A_REG_RANGE_VDROP_INCAP_COMP 0x0068
#define MCP39F511A_REG_DC_GAIN_CURRENT_RMS    0x006C
#define MCP39F511A_REG_DC_GAIN_VOLTAGE_RMS    0x006E
#define MCP39F511A_REG_DC_GAIN_ACTIVE_PWR     0x0070
#define MCP39F511A_REG_DC_OFFSET_CURRENT_RMS  0x0072
#define MCP39F511A_REG_DC_OFFSET_ACTIVE_PWR   0x0074
#define MCP39F511A_REG_OFFCAL_MSB             0x007A
#define MCP39F511A_REG_OFFCAL_CH0             0x007C
#define MCP39F511A_REG_OFFCAL_CH1             0x007E
#define MCP39F511A_REG_TEMP_POS_COMP_FREQ     0x0080
#define MCP39F511A_REG_TEMP_NEG_COMP_FREQ     0x0082
#define MCP39F511A_REG_TEMP_POS_COMP_CURRENT  0x0084
#define MCP39F511A_REG_TEMP_NEG_COMP_CURRENT  0x0086
#define MCP39F511A_REG_TEMP_POS_COMP_POWER    0x0088
#define MCP39F511A_REG_TEMP_NEG_COMP_POWER    0x008A
#define MCP39F511A_REG_SYS_CONFIG             0x0094
#define MCP39F511A_REG_EVENT_CONFIG           0x0098
#define MCP39F511A_REG_RANGE                  0x009C
#define MCP39F511A_REG_CAL_CURRENT            0x00A0
#define MCP39F511A_REG_CAL_VOLTAGE            0x00A4
#define MCP39F511A_REG_CAL_ACTIVE_PWR         0x00A6
#define MCP39F511A_REG_CAL_REACTIVE_PWR       0x00AA
#define MCP39F511A_REG_APPARENT_POWER_DIV     0x00BE
#define MCP39F511A_REG_ACCUM_INTERVAL         0x00C0
#define MCP39F511A_REG_PWM_PERIOD             0x00C2
#define MCP39F511A_REG_PWM_DUTY_CYCLE         0x00C4
#define MCP39F511A_REG_MINMAX_PTR_1           0x00C6
#define MCP39F511A_REG_MINMAX_PTR_2           0x00C8
#define MCP39F511A_REG_LINE_FREQ_REF          0x00CA
#define MCP39F511A_REG_THERMISTOR_VOLTAGE_CAL 0x00CC
#define MCP39F511A_REG_VOLTAGE_SAG_LIMIT      0x00CE
#define MCP39F511A_REG_VOLTAGE_SURGE_LIMIT    0x00D0
#define MCP39F511A_REG_OVER_CURRENT_LIMIT     0x00D2
#define MCP39F511A_REG_OVER_POWER_LIMIT       0x00D6
#define MCP39F511A_REG_OVERTEMP_LIMIT         0x00DA
#define MCP39F511A_REG_VOLTAGE_LOW_THRESHOLD  0x00DC
#define MCP39F511A_REG_VOLTAGE_HIGH_THRESHOLD 0x00DE
#define MCP39F511A_REG_NO_LOAD_THRESHOLD      0x00E0

// UART pins / port — historically Arduino Serial2 (UART2)
#define MCP39F511A_UART_NUM       2
#define MCP39F511A_UART_RX_GPIO   16
#define MCP39F511A_UART_TX_GPIO   17
#define MCP39F511A_UART_BAUD      9600

union Mcp39f511aSysConfig {
    uint32_t uDWord;
    struct {
        uint32_t ENRG_CNTRL      : 1;
        uint32_t PWM_CNTRL       : 1;
        uint32_t VREFEXT         : 1;
        uint32_t SHUTDOWN        : 2;
        uint32_t RESET           : 2;
        uint32_t TEMPCOMP        : 1;
        uint32_t SINGLE_WIRE     : 1;
        uint32_t _unimpl9        : 1;
        uint32_t ZCD_OUTPUT_DIS  : 1;
        uint32_t ZCD_PULS        : 1;
        uint32_t ZCD_INV         : 1;
        uint32_t UART            : 3;  // 100b = 9600
        uint32_t _unimpl16_23    : 8;
        uint32_t PGA_CH0         : 3;
        uint32_t PGA_CH1         : 3;
        uint32_t _unimpl30_31    : 2;
    };
};

union Mcp39f511aEventConfig {
    uint32_t uDWord;
    struct {
        uint32_t OVERCUR_TST    : 1;
        uint32_t OVERPOW_TST    : 1;
        uint32_t VSAG_TST       : 1;
        uint32_t VSUR_TST       : 1;
        uint32_t OVERCUR_LA     : 1;
        uint32_t OVERPOW_LA     : 1;
        uint32_t VSAG_LA        : 1;
        uint32_t VSUR_LA        : 1;
        uint32_t VSAG_CL        : 1;
        uint32_t VSUR_CL        : 1;
        uint32_t OVERPOW_CL     : 1;
        uint32_t OVERCUR_CL     : 1;
        uint32_t _unimpl12_13   : 2;
        uint32_t EVENT1_MC      : 1;
        uint32_t EVENT2_MC      : 1;
        uint32_t VSAG_PIN1      : 1;
        uint32_t VSURGE_PIN1    : 1;
        uint32_t OVERCUR_PIN1   : 1;
        uint32_t OVERPOW_PIN1   : 1;
        uint32_t VSAG_PIN2      : 1;
        uint32_t VSURGE_PIN2    : 1;
        uint32_t OVERCUR_PIN2   : 1;
        uint32_t OVERPOW_PIN2   : 1;
        uint32_t OVERTEMP_TST   : 1;
        uint32_t OVERTEMP_LA    : 1;
        uint32_t OVERTEMP_CL    : 1;
        uint32_t OVERTEMP_PIN1  : 1;
        uint32_t OVERTEMP_PIN2  : 1;
        uint32_t _unimpl29_31   : 3;
    };
};

struct Mcp39f511aConfig {
    Mcp39f511aSysConfig   sysConfig;
    Mcp39f511aEventConfig eventConfig;
    bool valid;
};

struct MCP39F511A_FullConfig {
    uint16_t acGainCurrentRms;
    uint16_t acGainVoltageRms;
    uint16_t acGainActivePower;
    uint16_t acGainReactivePower;
    int16_t  acOffsetCurrentRms;
    int16_t  acOffsetActivePower;
    int16_t  acOffsetReactivePower;
    uint16_t gainLineFrequency;
    int16_t  phaseCompensation;
    uint16_t voltageDropComp;
    uint16_t inCapCurrentComp;
    uint16_t rangeVdropInCapComp;
    uint16_t dcGainCurrentRms;
    uint16_t dcGainVoltageRms;
    uint16_t dcGainActivePower;
    int16_t  dcOffsetCurrentRms;
    int16_t  dcOffsetActivePower;
    uint16_t offcalMsb;
    uint16_t offcalCh0;
    uint16_t offcalCh1;
    uint16_t tempPosCompFreq;
    uint16_t tempNegCompFreq;
    uint16_t tempPosCompCurrent;
    uint16_t tempNegCompCurrent;
    uint16_t tempPosCompPower;
    uint16_t tempNegCompPower;
    Mcp39f511aSysConfig   sysConfig;
    Mcp39f511aEventConfig eventConfig;
    uint32_t range;
    uint32_t calibrationCurrent;
    uint16_t calibrationVoltage;
    uint32_t calibrationActivePower;
    uint32_t calibrationReactivePower;
    uint16_t apparentPowerDivisor;
    uint16_t accumulatorIntervalParameter;
    uint16_t pwmPeriod;
    uint16_t pwmDutyCycle;
    uint16_t minMaxPointer1;
    uint16_t minMaxPointer2;
    uint16_t lineFrequencyReference;
    uint16_t thermistorVoltageCal;
    uint16_t voltageSagLimit;
    uint16_t voltageSurgeLimit;
    uint32_t overCurrentLimit;
    uint32_t overPowerLimit;
    uint16_t overTemperatureLimit;
    uint16_t voltageLowThreshold;
    uint16_t voltageHighThreshold;
    uint16_t noLoadThreshold;
    bool valid;
};

struct Mcp39f511aEnergyCounters {
    uint64_t importActiveEnergy;
    uint64_t exportActiveEnergy;
    uint64_t importReactiveEnergy;
    uint64_t exportReactiveEnergy;
    bool valid;
};

struct Mcp39f511aMeasurements {
    float rmsVoltage;
    float rmsCurrent;
    float activePower;
    float reactivePower;
    float apparentPower;
    float frequency;
    float powerFactor;
    uint32_t timestamp;
    bool valid;
};

// Install ESP-IDF UART driver on UART2 / pins 16+17 / 9600. Call once from
// setup before any mcp39f511a.* traffic (boot dump or PowerMeterTask).
bool mcp39f511a_uart_begin(void);

class Mcp39f511aDev {
public:
    Mcp39f511aDev() = default;

    bool readMeasurements(void);
    Mcp39f511aMeasurements *getMeasurements(void) { return &measurements_; }

    bool readConfig(void);
    Mcp39f511aConfig *getConfig(void) { return &config_; }

    bool writeConfig(const Mcp39f511aConfig &cfg);
    bool saveToFlash(void);

    bool readFullConfig(void);
    MCP39F511A_FullConfig *getFullConfig(void) { return &fullConfig_; }
    bool writeFullConfig(const MCP39F511A_FullConfig &cfg);

    bool readEnergyCounters(void);
    Mcp39f511aEnergyCounters *getEnergyCounters(void) { return &energyCounters_; }
    bool resetEnergyCounters(void);

private:
    bool readRegister(uint16_t address, uint8_t *data, uint8_t numBytes);
    bool writeRegister(uint16_t address, const uint8_t *data, uint8_t numBytes);
    uint8_t sendCommand(const uint8_t *cmd, uint8_t cmdLen, uint8_t *response, uint8_t *responseLen,
                        uint32_t timeoutMs = MCP39F511A_RESPONSE_TIMEOUT_MS,
                        bool expectDataResponse = true);
    uint8_t calculateChecksum(const uint8_t *data, uint8_t len);
    void flushRx(void);
    bool readExact(uint8_t *buf, size_t len, uint32_t timeoutMs);

    bool readUint16(uint16_t address, uint16_t &out);
    bool readInt16(uint16_t address, int16_t &out);
    bool readUint32(uint16_t address, uint32_t &out);
    bool readInt32(uint16_t address, int32_t &out);
    bool readUint64(uint16_t address, uint64_t &out);

    bool writeUint16(uint16_t address, uint16_t value);
    bool writeInt16(uint16_t address, int16_t value);
    bool writeUint32(uint16_t address, uint32_t value);
    bool writeInt32(uint16_t address, int32_t value);

    Mcp39f511aMeasurements measurements_ = {};
    Mcp39f511aConfig config_ = {};
    MCP39F511A_FullConfig fullConfig_ = {};
    Mcp39f511aEnergyCounters energyCounters_ = {};
};

extern Mcp39f511aDev mcp39f511a;

#endif // MCP39F511A_DEV_H
