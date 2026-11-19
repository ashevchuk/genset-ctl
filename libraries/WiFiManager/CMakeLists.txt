cmake_minimum_required(VERSION 3.5)

idf_component_register(
                       SRCS "WiFiManager.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES arduino-esp32 WebServer
)

project(WiFiManager)

# Vendor code (frozen, not touched): a %d format specifier against a
# uint32_t argument — arduino-builder's default warning level lets this
# slide, idf's -Werror doesn't. Scoped to just this component.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=format=)
