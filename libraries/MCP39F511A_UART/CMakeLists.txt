# Project-local vendored library (arduino-builder finds it via LOCAL_LIB in
# compile.sh/compile_fast.sh) — this wrapper is invisible to arduino-builder
# (it only looks for .ino/.cpp/.h), coexists without touching the existing
# build.
idf_component_register(
    SRCS "MCP39F511A_UART.cpp"
    INCLUDE_DIRS "."
    REQUIRES arduino-esp32
)

# Vendor code (frozen, not touched) narrows a uint16_t register value to
# uint8_t in two byte-split writes — arduino-builder's default warning level
# lets this slide, idf's -Werror doesn't. Scoped to just this component, not
# a project-wide relaxation.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=narrowing)
