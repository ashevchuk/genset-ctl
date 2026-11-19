# ESP-IDF's component-name resolution has no override hook (component.cmake:
# `set(component_name ${base_dir})`, unconditional) — a component's name is
# always its containing directory's basename. The project root (one level
# up from idf/, registered as its own component by ../CMakeLists.txt) is
# therefore only named "esp32-gen-ctl-ai-rtos" because that happens to be
# this checkout's directory name — REQUIRES-ing it by that literal string
# would silently break the moment the repo is cloned/moved anywhere else
# (found live 2026-09-25 testing exactly that: "Failed to resolve component
# 'esp32-gen-ctl-ai-rtos'... unknown name" from a relocated checkout).
# Computed here instead so REQUIRES always names whatever the root actually
# is, regardless of what the checkout directory is called.
get_filename_component(PROJECT_ROOT_ABS "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
get_filename_component(PROJECT_ROOT_COMPONENT_NAME "${PROJECT_ROOT_ABS}" NAME)

idf_component_register(
    SRCS "esp32-gen-ctl-ai-rtos.cpp"
    INCLUDE_DIRS "."
    REQUIRES arduino-esp32 ${PROJECT_ROOT_COMPONENT_NAME} ArduinoJson AsyncWiFiMulti PubSubClient WebSockets WebServer WiFiManager Smoothed
             esp_partition wear_levelling app_update esp_timer esp_wifi lwip
)

# arduino-builder's default warning level is more lenient than idf's
# -Wall -Wextra -Werror. These are pure strictness deltas, not correctness
# bugs — verified individually, not a blanket "make warnings go away":
#   -Wformat=                 uint32_t passed to a %u specifier (technically
#                              a different typedef from plain unsigned int on
#                              this ABI even though same width/value; correct
#                              at runtime either way)
#   -Wmissing-field-initializers  a couple of partial struct initializers
#                              (e.g. `= {0}`) — remaining fields still
#                              zero-initialize either way
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=format=
    -Wno-error=missing-field-initializers
)
