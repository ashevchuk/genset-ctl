# Frozen local copy (was ~/Arduino/libraries/WebSockets) — see
# libraries/MCP39F511A_UART/CMakeLists.txt for why. Invisible to
# arduino-builder, coexists without touching the existing build.
# All src/*.cpp compiled unconditionally, same as arduino-builder already
# does for this library regardless of which specific headers a sketch
# includes (this project only uses WebSocketsServer, not the Client/SocketIO
# variants, but they build harmlessly alongside it).
idf_component_register(
    SRCS
        "src/WebSockets.cpp"
        "src/WebSocketsServer.cpp"
        "src/WebSocketsClient.cpp"
        "src/SocketIOclient.cpp"
    INCLUDE_DIRS "src"
    REQUIRES arduino-esp32
)

# WEBSOCKETS_TCP_TIMEOUT (src/WebSockets.h: `#ifndef WEBSOCKETS_TCP_TIMEOUT
# #define WEBSOCKETS_TCP_TIMEOUT (5000)`) is read only by this library's own
# src/*.cpp files above — a `#define` in a *consuming* .cpp (the main .cpp
# used to carry one, right before its own #include <WebSocketsServer.h>)
# only affects that file's own translation unit's preprocessor state, never
# this component's separately-compiled sources. Found live 2026-09-24: the
# project believed this was tightened to 1500ms since 2026-08-25 (a stalled
# WS client's blocking TCP write shouldn't be able to eat enough of
# ReportTask's 15s TWDT window to matter) — it was a silent no-op the whole
# time, every build since then actually ran on the library's unmodified
# 5000ms default. The define belongs here, where it can actually reach the
# code that reads it.
target_compile_definitions(${COMPONENT_LIB} PRIVATE WEBSOCKETS_TCP_TIMEOUT=1500)

# WEBSOCKETS_SERVER_CLIENT_MAX (src/WebSocketsServer.h: `#ifndef
# WEBSOCKETS_SERVER_CLIENT_MAX #define WEBSOCKETS_SERVER_CLIENT_MAX (5)`) —
# raised to 16, found live 2026-09-25 that 5 can genuinely run out (a single
# browser tab, over one long session with a few real network hiccups, ended
# up holding 4 simultaneous connections at once — see web.cpp's own
# enableHeartbeat() comment). Cost measured directly: 240 bytes of pure
# static RAM per slot (nm -S on wsServer's own BSS symbol, comparing a
# WEBSOCKETS_SERVER_CLIENT_MAX=5 vs =8 build), zero flash cost.
#
# Deliberately PUBLIC, unlike WEBSOCKETS_TCP_TIMEOUT above — that one is only
# ever read inside this library's own src/*.cpp function bodies (a plain
# runtime constant), but this one sizes `WSclient_t _clients[...]`, a member
# declared directly in the *header* (WebSocketsServer.h), which idf/main's
# own esp32-gen-ctl-ai-rtos.cpp (the global `WebSockets4WebServer wsServer`
# instance), web.cpp, and report.cpp all include too. PRIVATE would let this
# component's own .cpp files see one array size while every OTHER component
# that includes the same header and touches `wsServer` computes a different
# one — a real ODR violation (mismatched struct layout across translation
# units), not just a wrong constant. PUBLIC propagates it to every component
# that REQUIRES WebSockets, matching how idf_component_register's REQUIRES
# already works.
target_compile_definitions(${COMPONENT_LIB} PUBLIC WEBSOCKETS_SERVER_CLIENT_MAX=16)
