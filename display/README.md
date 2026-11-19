# LVGL Porting Example

The example demonstrates how to port LVGL(v8). And for RGB LCD, it can enable the avoid tearing fucntion.

## How to Use

use ESP32S3 Dev module board.
Set up as follow:
USB CDC On Boot: "Enabled" // Important for Serial communication
CPU Frequency: "240MHz (WiFi)"
Core Debug Level: "None"
USB DFU On Boot: "Disabled"
Erase All Flash Before Sketch Upload: "Disabled"
Events Run On: "Core 1"
Flash Mode: "QIO 80MHz"
Flash Size: "16MB (128Mb)"
JTAG Adapter: "Disabled"
Arduino Runs On: "Core 1"
USB Firmware MSC On Boot: "Disabled"
Partition Scheme: "8M with spiffs (3MB APP/1.5MB SPIFFS)"
PSRAM: "OPI PSRAM"
Upload Mode: "UART0 / Hardware CDC"
Upload Speed: "921600"
USB Mode: "Hardware CDC and JTAG"

3.5-inch LCD-TFT capacitive touch display module with housing
Supports WiFi and Bluetooth
Support lithium battery power supply
Capacitive touch
320 * 480 screen resolution
RGB 65K realistic colors
IPS glass panel
ESP32-S3 controller dual core CPU
The clock frequency can reach 240MHz
8M PSRAM
16M FLASH
Arduino LVGL development

To use this example, please firstly install the following dependent libraries:

- lvgl (>= v8.3.9, < v9)

Follow the steps below to configure:

1. For **ESP32_Display_Panel**:

    - Follow the [steps](../../README.md#configuring-drivers) to configure drivers if needed.
    - If using a supported development board, follow the [steps](../../README.md#using-supported-development-boards) to configure it.
    - If using a custom board, follow the [steps](../../README.md#using-custom-development-boards) to configure it.

2. For **lvgl**:

    - Follow the [steps](../../README.md#configuring-lvgl) to add *lv_conf.h* file and change the configurations.
    - Modify the macros in the [lvgl_port_v8.h](./lvgl_port_v8.h) file to configure the LVGL porting parameters.

3. Navigate to the `Tools` menu in the Arduino IDE to choose a ESP board and configure its parameters. For supported boards, please refter to [Configuring Supported Development Boards](../../README.md#configuring-supported-development-boards)
4. Verify and upload the example to your ESP board.

## Serial Output

```bash
...
LVGL porting example start
Initialize panel device
Initialize LVGL
Create UI
LVGL porting example end
IDLE loop
IDLE loop
...
```

## Troubleshooting

Please check the [FAQ](../../../../README.md#faq) first to see if the same question exists. If not, please create a [Github issue](https://github.com/esp-arduino-libs/ESP32_Display_Panel/issues). We will get back to you as soon as possible.
