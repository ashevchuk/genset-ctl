/*
 * MCP39F511A_UART - Basic Readings Example
 * 
 * This example demonstrates how to read power measurements from MCP39F511A
 * using the UART library for ESP32.
 * 
 * Hardware connections:
 * - MCP39F511A TX -> ESP32 GPIO 16 (Serial2 RX)
 * - MCP39F511A RX -> ESP32 GPIO 17 (Serial2 TX)
 * - MCP39F511A GND -> ESP32 GND
 * - MCP39F511A VDD -> 3.3V or 5V (check datasheet)
 * 
 * Note: Adjust Serial2 RX/TX pins according to your ESP32 board configuration.
 */

#include "MCP39F511A_UART.h"

// Define Serial2 pins for ESP32
#define MCP39F511A_RX_PIN  16
#define MCP39F511A_TX_PIN  17
#define MCP39F511A_BAUD    9600

// Create library instance
MCP39F511A_UART mcp39f511a(&Serial2, MCP39F511A_RX_PIN, MCP39F511A_TX_PIN, MCP39F511A_BAUD);

void setup() {
    // Initialize Serial for debug output
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("MCP39F511A UART Library - Basic Readings Example");
    Serial.println("=================================================");
    
    // Initialize MCP39F511A
    if (mcp39f511a.begin()) {
        Serial.println("MCP39F511A initialized successfully");
    } else {
        Serial.println("Failed to initialize MCP39F511A!");
        while(1) delay(1000);
    }
    
    // Read configuration at startup
    if (mcp39f511a.readConfig()) {
        Serial.println("Configuration read successfully");
        MCP39F511A_Config* config = mcp39f511a.getConfig();
        Serial.print("System Config: 0x");
        Serial.println(config->sysConfig, HEX);
        Serial.print("Event Config: 0x");
        Serial.println(config->eventConfig, HEX);
    } else {
        Serial.println("Failed to read configuration");
    }
    
    Serial.println("\nStarting measurements...\n");
}

void loop() {
    // Read measurements
    if (mcp39f511a.readMeasurements()) {
        MCP39F511A_Measurements* data = mcp39f511a.getMeasurements();
        
        Serial.println("=== Power Measurements ===");
        Serial.print("RMS Voltage:     ");
        Serial.print(data->rmsVoltage, 2);
        Serial.println(" V");
        
        Serial.print("RMS Current:     ");
        Serial.print(data->rmsCurrent, 4);
        Serial.println(" A");
        
        Serial.print("Active Power:    ");
        Serial.print(data->activePower, 3);
        Serial.println(" W");
        
        Serial.print("Reactive Power:  ");
        Serial.print(data->reactivePower, 3);
        Serial.println(" VAR");
        
        Serial.print("Apparent Power:  ");
        Serial.print(data->apparentPower, 3);
        Serial.println(" VA");
        
        Serial.print("Frequency:       ");
        Serial.print(data->frequency, 2);
        Serial.println(" Hz");
        
        Serial.print("Power Factor:    ");
        Serial.print(data->powerFactor, 4);
        Serial.println();
        
        Serial.println("==========================\n");
    } else {
        Serial.println("Failed to read measurements!");
    }
    
    delay(2000);  // Read every 2 seconds
}
