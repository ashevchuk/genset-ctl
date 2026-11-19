#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include <type_traits>
#include <utility>
#include <initializer_list>
#include "esp_log.h"

// Определяй DEBUG в .ino перед подключением Debug.h чтобы включить отладку.
// #define DEBUG
#ifndef DEBUG_ADD_TIMESTAMP
#define DEBUG_ADD_TIMESTAMP  true
#endif
#ifndef DEBUG_EOL
#define DEBUG_EOL "\r\n"
#endif

// SLOG_PRINT()/SLOG_PRINTLN()/SLOG_PRINTF() wrap every raw Serial.print()/
// println()/printf() call site in the firmware — set DEBUG_SERIAL_ENABLED
// to 0 to silence the USB serial console entirely (e.g. for a production
// build where nothing should ever assume a serial monitor is attached).
// Deliberately set *here*, not as a per-file #define before #include
// "debug.h": a #define in one .cpp is invisible to every other translation
// unit's own preprocessor state (the exact WIFI_POWER/WEBSOCKETS_TCP_TIMEOUT
// trap documented in CLAUDE.md/main-cpp-modularization project memory) — a
// single #ifndef guard in the shared header both files already include is
// the only way every SLOG_*() call site agrees on the same value.
#ifndef DEBUG_SERIAL_ENABLED
#define DEBUG_SERIAL_ENABLED 1
#endif

#if DEBUG_SERIAL_ENABLED
#define SLOG_PRINT(...)   Serial.print(__VA_ARGS__)
#define SLOG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define SLOG_PRINTF(...)  Serial.printf(__VA_ARGS__)
// No-op when DEBUG_SERIAL_ENABLED is on — ESP-IDF's own ESP_LOG* output
// (esp32-hal/Arduino-core included, since it's implemented on top of the
// same logging facility) stays at whatever level sdkconfig set, same as
// always.
#define SLOG_SILENCE_IDF_LOGS() do {} while (0)
#else
#define SLOG_PRINT(...)   do {} while (0)
#define SLOG_PRINTLN(...) do {} while (0)
#define SLOG_PRINTF(...)  do {} while (0)
// Same switch also kills ESP-IDF/Arduino-core's own ESP_LOG* output at
// runtime (esp_log_level_set() applies immediately, no sdkconfig/rebuild
// needed) — call this once, as early as possible in setup(), so the rest
// of boot (WiFi driver init, etc.) stays quiet too. This is a runtime call,
// not a Kconfig change: the bootloader banner and the handful of cpu_start
// lines printed before setup() ever runs are earlier than any C++ code can
// reach and are NOT silenced by this — only sdkconfig's own
// CONFIG_BOOTLOADER_LOG_LEVEL/CONFIG_LOG_DEFAULT_LEVEL could reach those,
// and that's a separate, heavier lever (requires `rm idf/sdkconfig` +
// rebuild, see CLAUDE.md's idf.py-build-details) deliberately not wired to
// this same flag.
#define SLOG_SILENCE_IDF_LOGS() esp_log_level_set("*", ESP_LOG_NONE)
#endif

#ifdef DEBUG

static inline void printOne(Print &out, const __FlashStringHelper *ifsh) { out.print(ifsh); }
static inline void printOne(Print &out, const char *s) { out.print(s); }
static inline void printOne(Print &out, const String &s) { out.print(s); }
static inline void printOne(Print &out, char c) { out.print(c); }
static inline void printOne(Print &out, bool b) { out.print(b ? "true" : "false"); }

// Обработка всех арифметических типов (int, float и т.д.)
template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value>::type>
static inline void printOne(Print &out, T num) {
  out.print(num);
}

// Универсальная обёртка, использующая forward
template<typename T>
static inline void printAny(Print &out, T &&value) {
  printOne(out, std::forward<T>(value));
}

// Основная variadic-функция debug, печатает в любой Print
template<typename... Args>
void debug(Print &out, Args&&... args) {
#if DEBUG_ADD_TIMESTAMP
  out.print('[');
  out.print(millis());
  out.print("] ");
#endif
  (void)std::initializer_list<int>{( (void)printAny(out, std::forward<Args>(args)), 0)...};
  out.print(DEBUG_EOL);
}

// Версия, использующая глобальный WiFiClient (скетч должен объявить `WiFiClient client;`)
template<typename... Args>
void debugIfClientConnected(Args&&... args) {
  extern WiFiClient client; // ссылка на глобальный клиент, объяви client в .ino
  if (client && client.connected()) {
    debug(static_cast<Print&>(client), std::forward<Args>(args)...);
  }
}

template<typename... Args>
void debugIfConnected(Args&&... args) {
    uint8_t i;
    //check UART for data
    //push UART data to all connected telnet clients
    for (i = 0; i < MAX_SRV_CLIENTS; i++) {
	if (serverClients[i] && serverClients[i].connected()) {
	    debug(static_cast<Print&>(serverClients[i]), std::forward<Args>(args)...);
	//        delay(1);
	}
    }
}

// Удобный макрос
#define SDEBUG(...) debugIfConnected(__VA_ARGS__)

// Отдельный макрос для отладки зарядки аккумулятора.
// Включается независимо: #define DEBUG_CHARGING перед #include "debug.h"
#ifdef DEBUG_CHARGING
#define SDEBUG_CHARGING(...) debugIfConnected(__VA_ARGS__)
#else
#define SDEBUG_CHARGING(...)
#endif

#else // DEBUG выключен

template<typename... Args> inline void debug(Print&, Args&&...) {}
template<typename... Args> inline void debugIfConnected(Args&&...) {}
#define SDEBUG(...)
#define SDEBUG_CHARGING(...)

#endif // DEBUG

#endif // DEBUG_H
