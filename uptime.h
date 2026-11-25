#ifndef UPTIME_H
#define UPTIME_H

#include <stdint.h>
#include <esp_timer.h>

// Milliseconds since boot. Wraps at 2^32 (~49.7 days) exactly like Arduino's
// millis(), so existing unsigned-subtraction interval checks keep working.
static inline uint32_t uptime_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

#endif
