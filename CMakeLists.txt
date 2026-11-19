# idf.py component wrapper for this project's own device drivers — read
# directly by the idf/ build via EXTRA_COMPONENT_DIRS, no copies. Invisible
# to arduino-builder (it only looks for .ino/.cpp/.h, ignores CMakeLists.txt),
# so this coexists with the existing Arduino build without touching it.
# esp32-gen-ctl-ai-rtos.ino itself is NOT listed here — .ino files can't be
# compiled directly by CMake (arduino-builder's ctags auto-prototyping has no
# idf.py equivalent) and are ported separately into idf/main/.
idf_component_register(
    SRCS
        "ads1115_dev.cpp"
        "bootloader_update.cpp"
        "buzzer.cpp"
        "config.cpp"
        "datetime.cpp"
        "encoder.cpp"
        "esp_now_link.cpp"
        "fram_dev.cpp"
        "fram_log.cpp"
        "gen_discovery.cpp"
        "i2c.cpp"
        "ina233_dev.cpp"
        "mcp39f511a_dev.cpp"
        "mcp9600_dev.cpp"
        "mqtt.cpp"
        "oled_dev.cpp"
        "oled_menu.cpp"
        "powermeter.cpp"
        "report.cpp"
        "rtc_dev.cpp"
        "shiftreg.cpp"
        "statemachine.cpp"
        "tacho.cpp"
        "update_mode.cpp"
        "web.cpp"
        "wifi_setup.cpp"
    INCLUDE_DIRS "."
    REQUIRES arduino-esp32 esp_driver_pcnt esp_driver_uart SSD1306Ascii ArduinoJson AsyncWiFiMulti Smoothed PubSubClient WebSockets WebServer WiFiManager app_update esp_partition wear_levelling espcoredump lwip spi_flash esp_wifi
)

# Same pure strictness delta already documented+suppressed for the main
# component in idf/main/CMakeLists.txt (see its own comment for the full
# rationale) — report.cpp inherited a handful of these %u/uint32_t call
# sites when ReportTask moved here from the main .cpp. Scoped to just this
# one file rather than the whole component, since none of the other files
# here trip it.
set_source_files_properties(report.cpp web.cpp PROPERTIES COMPILE_OPTIONS "-Wno-error=format=")
