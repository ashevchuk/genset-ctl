/*********************************************************************
 *
 *       MCP39F511A UART Library for ESP32
 *
 *********************************************************************
 * FileName:        MCP39F511A_UART.h
 * Revision:        1.0.0
 * Date:            2024
 *
 * Dependencies:    Arduino.h
 *
 * Arduino Board:   ESP32
 *
 * Description:     Simplified UART-only library for MCP39F511A power monitoring IC
 *                  Based on original library by Futura Group srl
 *
 **********************************************************************/

#ifndef _MCP39F511A_UART_H_INCLUDED__
#define _MCP39F511A_UART_H_INCLUDED__

#include "Arduino.h"
#include "HardwareSerial.h"

//======================================================================
//  Response codes
#define MCP39F511A_ACK          0x06    //  Acknowledge
#define MCP39F511A_NACK         0x15    //  NO Acknowledge
#define MCP39F511A_CSFAIL       0x51    //  Checksum fail

//======================================================================
//  Error codes
#define MCP39F511A_OK                   0x00
#define MCP39F511A_ERROR_RECEIVED_LESS   0x01
#define MCP39F511A_ERROR_CHECKSUM        0x02
#define MCP39F511A_ERROR_NO_ACK         0x03
#define MCP39F511A_ERROR_TIMEOUT         0x04

//======================================================================
//  Command codes
#define MCP39F511A_CMD_SET_ADDR_PTR     0x41
#define MCP39F511A_CMD_REG_READ_N       0x4E
#define MCP39F511A_CMD_REG_WRITE_N      0x4D
#define MCP39F511A_CMD_SAVE_TO_FLASH    0x53
#define MCP39F511A_HEADER_BYTE          0xA5

//======================================================================
//  Register addresses (from datasheet Table 6-1)
#define MCP39F511A_REG_INSTR_PTR        0x0000
#define MCP39F511A_REG_SYSTEM_STATUS     0x0002
#define MCP39F511A_REG_SYSTEM_VERSION    0x0004
#define MCP39F511A_REG_VRMS             0x0006
#define MCP39F511A_REG_FREQUENCY        0x0008
#define MCP39F511A_REG_ANALOG_INPUT     0x000A
#define MCP39F511A_REG_POWER_FACTOR     0x000C
#define MCP39F511A_REG_IRMS             0x000E
#define MCP39F511A_REG_POWER_ACTIVE     0x0012
#define MCP39F511A_REG_POWER_REACTIVE   0x0016
#define MCP39F511A_REG_POWER_APPARENT   0x001A
#define MCP39F511A_REG_SYS_CONFIG       0x007A
#define MCP39F511A_REG_EVENT_CONFIG      0x007E

// Calibration registers (AC mode)
#define MCP39F511A_REG_CAL_DELIMITER    0x005E
#define MCP39F511A_REG_GAIN_CURRENT_RMS 0x0060
#define MCP39F511A_REG_GAIN_VOLTAGE_RMS 0x0062
#define MCP39F511A_REG_GAIN_ACTIVE_PWR  0x0064
#define MCP39F511A_REG_GAIN_REACTIVE_PWR 0x0066
#define MCP39F511A_REG_OFFSET_CURRENT_RMS 0x0068
#define MCP39F511A_REG_OFFSET_ACTIVE_PWR  0x006C
#define MCP39F511A_REG_OFFSET_REACTIVE_PWR 0x0070
#define MCP39F511A_REG_GAIN_FREQUENCY   0x0076
#define MCP39F511A_REG_PHASE_COMP       0x0076

//======================================================================
//  Data scaling constants
#define MCP39F511A_VOLTAGE_DEC      2   //  Voltage decimals (divide by 10^2 = 100)
#define MCP39F511A_CURRENT_DEC      3   //  Current decimals (divide by 10^3 = 1000)
#define MCP39F511A_FREQUENCY_DEC   3   //  Frequency decimals (divide by 10^3 = 1000)
#define MCP39F511A_POWER_DEC       2   //  Power decimals (divide by 10^2 = 100)

//======================================================================
//  Timing constants
#define MCP39F511A_TIMESLOT_US     100  //  Time slot between bytes (microseconds)
#define MCP39F511A_RESPONSE_TIMEOUT_MS 500  //  Response timeout (milliseconds)

//======================================================================
//  Flag structures (based on Cmd_MCP39F511.h)
typedef union {
    uint16_t uWord;
    struct {
        uint8_t Byte0;
        uint8_t Byte1;
    };
    struct {
        uint8_t Vsag        :1;  //  State of Voltage Sag Detection algorithm. This bit is latched and must be cleared
        uint8_t Vsurge      :1;  //  State of Voltage Surge Detection algorithm. This bit is latched and must be cleared
        uint8_t OverCur     :1;  //  State of the Overcurrent detection algorithm. An Overcurrent event has occurred in the system
        uint8_t OverPow     :1;  //  State of Overpower detection algorithm. An Overpower event has occurred in the system
        uint8_t Sign_PA     :1;  //  Sign of Active Power (import/export sign of active power)
        uint8_t Sign_PR     :1;  //  Sign of Reactive Power
        uint8_t OverTemp    :1;  //  State of the Overtemperature Detection Algorithm
        uint8_t Free1       :3;
        uint8_t Event1      :1;  //  State of Event1 Detection algorithm. This bit is latched and must be cleared
        uint8_t Event2      :1;  //  State of Event2 Detection algorithm. This bit is latched and must be cleared
        uint8_t Free2       :4;
    };
} MCP39F511A_SystemStatusFlag;

typedef union {
    uint16_t uWord;
    struct {
        uint8_t Byte0;
        uint8_t Byte1;
    };
    struct {
        uint8_t Year        :4;
        uint8_t Month       :4;
        uint8_t Day         :8;
    };
} MCP39F511A_SystemVersionFlag;

typedef union {
    long l;
    uint32_t uDWord;
    uint8_t Byte0;
    uint8_t Byte1;
    uint8_t Byte2;
    uint8_t Byte3;
    struct {
        uint16_t Word0;
        uint16_t Word1;
    };
    struct {
        uint8_t Free_1         :2;  //  Unimplemented
        uint8_t VrefExt        :1;  //  VREFEXT:        Internal Voltage Reference Shutdown Control
        uint8_t ShutDown       :2;  //  SHUTDOWN <1:0>: Shutdown mode setting for ADCs
        uint8_t Reset          :2;  //  RESET <1:0>:    Reset mode setting for ADCs
        uint8_t TempComp       :1;  //  TEMPCOMP:       Temperature-Compensation Enable bit
        uint8_t SingleWire     :1;  //  SINGLE_WIRE:    Single-Wire Enable bit
        uint8_t Free_2         :2;  //  Unimplemented
        uint8_t ZcdOutput_Dis  :1;  //  ZCD_OUTPUT_DIS: Disable the Zero Crossing output pin
        uint8_t ZcdPuls        :1;  //  ZCD_PULS:       Zero Crossing Detection Pulse mode
        uint8_t ZcdInv         :1;  //  ZCD_INV:        Zero Crossing Detection Output Inverse
        uint8_t UartBaud       :3;  //  UART<2:0>:      UART Baud Rate bits "000" -> 115200 Default
        uint8_t VrefCal        :8;  //  VREFCAL<n>:     Internal voltage reference temperature coefficient register value
        uint8_t PgaCh0         :3;  //  PGA_CH0 <2:0>:  PGA Setting for the voltage channel
        uint8_t PgaCh1         :3;  //  PGA_CH1 <2:0>:  PGA Setting for the current channel
        uint8_t Free_3         :2;  //  Unimplemented
    };
} MCP39F511A_SystemConfigurationFlag;

typedef union {
    long l;
    uint32_t uDWord;
    uint8_t Byte0;
    uint8_t Byte1;
    uint8_t Byte2;
    uint8_t Byte3;
    struct {
        uint16_t Word0;
        uint16_t Word1;
    };
    struct {
        uint8_t OverCurTst    :1;  //  OVERCUR_TST:   Test control of the Overcurrent event
        uint8_t OverPowTst    :1;  //  OVERPOW_TST:   Test control of the Overpower event
        uint8_t VsagTst       :1;  //  VSAG_TST:      Test control of the Voltage Sag event
        uint8_t VsurTst       :1;  //  VSUR_TST:      Test control of the Voltage Surge event
        uint8_t OverCurLa     :1;  //  OVERCUR_LA:    Latching control of the Overcurrent event
        uint8_t OverPowLa     :1;  //  OVERPOW_LA:    Latching control of the Overpower event
        uint8_t VsagLa        :1;  //  VSAG_LA:       Latching control of the Voltage Sag event
        uint8_t VsurLa        :1;  //  VSUR_LA:       Latching control of the Voltage Surge event
        uint8_t VsagCl        :1;  //  VSAG_CL:       Reset or clear bit for the Voltage Sag event
        uint8_t VsurCl        :1;  //  VSUR_CL:       Reset or clear bit for the Voltage Surge event
        uint8_t OverPowCl     :1;  //  OVERPOW_CL:    Reset or clear bit for the Overpower event
        uint8_t OverCurCl     :1;  //  OVERCUR_CL:    Reset or clear bit for the Overcurrent event
        uint8_t Free_1        :4;  //  Unimplemented
        uint8_t VsagPin1      :1;  //  VSAG_PIN1:     Event pin 1 operation for the Voltage Sag event
        uint8_t VsurgePin1    :1;  //  VSURGE_PIN1:   Event pin 1 operation for the Voltage Surge event
        uint8_t OverCurPin1   :1;  //  OVERCUR_PIN1:  Event pin 1 operation for the Overcurrent event
        uint8_t OverPowPin1   :1;  //  OVERPOW_PIN1:  Event pin 1 operation for the Overpower event
        uint8_t VsagPin2      :1;  //  VSAG_PIN2:     Event pin 2 operation for the Voltage Sag event
        uint8_t VsurgePin2    :1;  //  VSURGE_PIN2:   Event pin 2 operation for the Voltage Surge event
        uint8_t OverCurPin2   :1;  //  OVERCUR_PIN2:  Event pin 2 operation for the Overcurrent event
        uint8_t OverPowPin2   :1;  //  OVERPOW_PIN2:  Event pin 2 operation for the Overpower event
        uint8_t OverTempTest  :1;  //  OVERTEMP_TST:  Test control of the Overtemperature event
        uint8_t OverTempLa    :1;  //  OVERTEMP_LA:   Latching control of the Overtemperature event
        uint8_t OverTempCl    :1;  //  OVERTEMP_CL:   Reset or clear bit for the Overtemperature event
        uint8_t OverTempPin1  :1;  //  OVERTEMP_PIN1: Event pin 1 operation for the Overtemperature event
        uint8_t OverTempPin2  :1;  //  OVERTEMP_PIN2: Event pin 2 operation for the Overtemperature event
        uint8_t Free_2        :3;  //  Unimplemented
    };
} MCP39F511A_EventConfigurationFlag;

typedef union {
    uint16_t uWord;
    struct {
        uint8_t Byte0;
        uint8_t Byte1;
    };
    struct {
        uint8_t PowerFactorSign             :1;  //  Sign of Power Factor "1" negative sign; "0" positive sign
        uint8_t PowerFactorAverageSign      :1;  //  Sign of Power Factor Average "1" negative sign; "0" positive sign
        uint8_t ReadDataInProgress          :1;  //  If "1" reads data in progress
        uint8_t RestoreParametersFT1346M    :1;  //  If "1" Restore FT1346 Factory Parameters
        uint8_t RestoreParametersMCP39F511  :1;  //  If "1" Restore Microchip MCP39F511 Factory Parameters
        uint8_t Free                        :2;
        uint8_t ZCD_Error                   :1;  //  If "1" Zero Cross Detection failed
    };
} MCP39F511A_GenericFlag;

//======================================================================
//  Data structures
typedef struct {
    float rmsVoltage;        //  RMS Voltage (V)
    float rmsCurrent;        //  RMS Current (A)
    float activePower;       //  Active Power (W)
    float reactivePower;     //  Reactive Power (VAR)
    float apparentPower;     //  Apparent Power (VA)
    float frequency;        //  Line Frequency (Hz)
    float powerFactor;       //  Power Factor
    uint32_t timestamp;     //  Timestamp of last reading
    bool valid;              //  Data validity flag
} MCP39F511A_Measurements;

typedef struct {
    MCP39F511A_SystemConfigurationFlag sysConfig;     //  System Configuration Register (32-bit)
    MCP39F511A_EventConfigurationFlag eventConfig;    //  Event Configuration Register (32-bit)
    bool valid;                                        //  Configuration validity flag
} MCP39F511A_Config;

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

//======================================================================
//  Main class
class MCP39F511A_UART {
    public:
        // Constructor
        MCP39F511A_UART(HardwareSerial *serial, uint8_t rxPin, uint8_t txPin, uint32_t baudRate = 9600);
        
        // Initialization
        bool begin();
        void end();
        
        // Low-level communication
        uint8_t sendCommand(uint8_t *cmd, uint8_t cmdLen, uint8_t *response, uint8_t *responseLen, uint16_t timeout_ms = MCP39F511A_RESPONSE_TIMEOUT_MS);
        bool readRegister(uint16_t address, uint8_t *data, uint8_t numBytes);
        bool writeRegister(uint16_t address, uint8_t *data, uint8_t numBytes);
        bool saveToFlash();
        
        // Helper functions for reading different data types
        int16_t readInt16(uint16_t address);
        uint16_t readUint16(uint16_t address);
        uint32_t readUint32(uint16_t address);
        
        // Measurement functions
        bool readMeasurements();
        MCP39F511A_Measurements* getMeasurements();
        
        // Configuration functions
        bool readConfig();
        bool writeConfig();
        MCP39F511A_Config* getConfig();
        
        // Calibration functions
        bool readCalibration();
        bool writeCalibration();
        MCP39F511A_Calibration* getCalibration();
        
        // Utility functions
        uint8_t calculateChecksum(uint8_t *data, uint8_t len);
        
    private:
        HardwareSerial *_serial;
        uint8_t _rxPin;
        uint8_t _txPin;
        uint32_t _baudRate;
        
        MCP39F511A_Measurements _measurements;
        MCP39F511A_Config _config;
        MCP39F511A_Calibration _calibration;
        
        void clearSerialBuffer();
};

#endif // _MCP39F511A_UART_H_INCLUDED__
