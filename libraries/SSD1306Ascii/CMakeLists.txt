# Frozen local copy (was ~/Arduino/libraries/SSD1306Ascii) — see
# libraries/MCP39F511A_UART/CMakeLists.txt for why. Invisible to
# arduino-builder, coexists without touching the existing build.
idf_component_register(
    SRCS "src/SSD1306Ascii.cpp"
    INCLUDE_DIRS "src"
    REQUIRES arduino-esp32
)
