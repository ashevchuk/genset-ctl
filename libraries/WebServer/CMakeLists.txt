# Frozen local copy of arduino-esp32's own WebServer library (was
# idf/managed_components/espressif__arduino-esp32/libraries/WebServer) — see
# libraries/MCP39F511A_UART/CMakeLists.txt for why vendoring beats hand-
# editing managed_components/ directly (a component-manager reconfigure
# silently discards edits there; patch_arduino_esp32.sh exists precisely
# because NetworkClient.cpp couldn't be vendored the same way this could).
#
# Vendored specifically to add a WEBSERVER_HAS_HOOK mechanism (see
# WebServer.h/Parsing.cpp's own comments at the hook call site) that
# upstream only ever implemented for ESP8266/RP2040 — WebSocketsServer's own
# libraries/WebSockets/src/WebSockets4WebServer.h shim already knows how to
# use it, it was just gated off on ESP32 for lack of this. Frozen like every
# other library here — update by hand only when actually needed, not
# re-synced from managed_components/ automatically.
idf_component_register(
    SRCS
        "src/WebServer.cpp"
        "src/Parsing.cpp"
        "src/detail/mimetable.cpp"
        "src/middleware/AuthenticationMiddleware.cpp"
        "src/middleware/CorsMiddleware.cpp"
        "src/middleware/LoggingMiddleware.cpp"
        "src/middleware/MiddlewareChain.cpp"
    INCLUDE_DIRS "src"
    REQUIRES arduino-esp32
)
