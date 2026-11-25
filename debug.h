#ifndef DEBUG_H
#define DEBUG_H

#include <stdio.h>
#include <inttypes.h>
#include <string>
#include <type_traits>
#include <utility>
#include <initializer_list>
#include "esp_log.h"
#include "uptime.h"
#include "debug_telnet.h"

// Define DEBUG (and optionally DEBUG_CHARGING) before including this header
// to enable SDEBUG()/SDEBUG_CHARGING() output on the telnet sink.
// #define DEBUG
#ifndef DEBUG_ADD_TIMESTAMP
#define DEBUG_ADD_TIMESTAMP  true
#endif
#ifndef DEBUG_EOL
#define DEBUG_EOL "\r\n"
#endif

// SLOG_PRINT()/SLOG_PRINTLN()/SLOG_PRINTF() are the firmware's console
// output (stdout = UART0, 115200, "\n" sent as "\r\n" — the same bytes the
// old Serial.print()/println()/printf() produced). SLOG_PRINT/PRINTLN only
// ever take a string literal. Set DEBUG_SERIAL_ENABLED to 0 to silence the
// USB serial console entirely (e.g. for a production
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
// SLOG_PRINT flushes: the old Serial.print() put partial lines (progress
// dots) on the wire immediately.
#define SLOG_PRINT(s)     do { fputs((s), stdout); fflush(stdout); } while (0)
#define SLOG_PRINTLN(...) slog_println(__VA_ARGS__)
#define SLOG_PRINTF(...)  printf(__VA_ARGS__)
static inline void slog_println(void) { putchar('\n'); }
static inline void slog_println(const char *s) { puts(s); }
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

// `v` with exactly `digits` decimals, as Arduino's String(float, digits) gave;
// used for SDEBUG() arguments.
static inline std::string dbg_fixed(double v, int digits) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%.*f", digits, v);
  return buf;
}

#ifdef DEBUG

// Same formatting as the Arduino Print overloads SDEBUG() used before:
// numbers in decimal, floating point with 2 decimals, bool as true/false.
static inline void debug_append(std::string &out, const char *s) { out += s; }
static inline void debug_append(std::string &out, char c) { out += c; }
static inline void debug_append(std::string &out, bool b) { out += b ? "true" : "false"; }
template <typename T, typename = decltype(std::declval<const T &>().c_str())>
static inline void debug_append(std::string &out, const T &s) { out += s.c_str(); }
template <typename T, typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, bool>::value &&
                                                  !std::is_same<T, char>::value, int>::type = 0>
static inline void debug_append(std::string &out, T num) {
  char buf[24];
  if (std::is_signed<T>::value) snprintf(buf, sizeof(buf), "%lld", (long long)num);
  else snprintf(buf, sizeof(buf), "%llu", (unsigned long long)num);
  out += buf;
}
template <typename T, typename std::enable_if<std::is_floating_point<T>::value, int>::type = 0>
static inline void debug_append(std::string &out, T num) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%.2f", (double)num);
  out += buf;
}

template<typename... Args>
void debugIfConnected(Args&&... args) {
  std::string line;
#if DEBUG_ADD_TIMESTAMP
  line += '[';
  debug_append(line, uptime_ms());
  line += "] ";
#endif
  (void)std::initializer_list<int>{((void)debug_append(line, std::forward<Args>(args)), 0)...};
  line += DEBUG_EOL;
  debug_telnet_send(line.data(), line.size());
}

#define SDEBUG(...) debugIfConnected(__VA_ARGS__)

// Separate switch for battery-charging debug output: #define DEBUG_CHARGING
// before #include "debug.h".
#ifdef DEBUG_CHARGING
#define SDEBUG_CHARGING(...) debugIfConnected(__VA_ARGS__)
#else
#define SDEBUG_CHARGING(...)
#endif

#else // DEBUG off

template<typename... Args> inline void debugIfConnected(Args&&...) {}
#define SDEBUG(...)
#define SDEBUG_CHARGING(...)

#endif // DEBUG

#endif // DEBUG_H
