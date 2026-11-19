#pragma once

#include <Arduino.h>
#include <Smoothed.h>

// Guards every SmoothedSafe<T> instance (adc_1_0.../rpm_smoothed) — these are
// shared between I2CTask/PowerMeterTask (writers) and now also oled_menu.cpp's
// i2c_render_display() (reader), which is a separate translation unit.
//
// This MUST be `extern`, not `static`: a `static portMUX_TYPE` at namespace
// scope in a header gives every translation unit that includes it its own
// private copy, and two separate mutex instances guard nothing against each
// other — SmoothedSafe's whole cross-task locking guarantee would silently
// stop applying the moment a second .cpp file included this header. Defined
// once, non-static, in esp32-gen-ctl-ai-rtos.cpp.
extern portMUX_TYPE smoothed_mux;

// Thread-safe wrapper around Smoothed<T>. Values wrapped by this are shared
// across tasks (and translation units); doubles are 64-bit and tear on this
// 32-bit core, and Smoothed's own ring buffer state is not atomic.
template <typename T>
class SmoothedSafe : public Smoothed<T> {
 public:
  bool add(T value) {
    portENTER_CRITICAL(&smoothed_mux);
    bool r = Smoothed<T>::add(value);
    portEXIT_CRITICAL(&smoothed_mux);
    return r;
  }
  T get() {
    portENTER_CRITICAL(&smoothed_mux);
    T r = Smoothed<T>::get();
    portEXIT_CRITICAL(&smoothed_mux);
    return r;
  }
  T getLast() {
    portENTER_CRITICAL(&smoothed_mux);
    T r = Smoothed<T>::getLast();
    portEXIT_CRITICAL(&smoothed_mux);
    return r;
  }
};
