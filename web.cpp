#include "web.h"

#include <esp_task_wdt.h>

#include "rtos_config.h"
#include "gen_discovery.h"
#include "http_server.h"
#include "debug.h"

void WebTask(void *pvParameters) {
    (void)pvParameters;
    esp_task_wdt_add(NULL);  // self-register before first wdt_reset

    // Started here rather than in setup(): its handlers run in their own task
    // as soon as it starts and need the RTOS objects (xFlashI2CGuard, the
    // I2C job queue) that only exist by the time WebTask runs.
    if (!http_server_start()) {
        SLOG_PRINTLN("http_server: start failed");
    }

    for (;;) {
        gen_discovery_poll();
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
