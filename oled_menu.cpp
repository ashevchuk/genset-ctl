/*
 * oled_menu.cpp — SSD1306 status/config UI driven by the rotary encoder.
 *
 * Extracted 2026-09-24 from esp32-gen-ctl-ai-rtos.cpp as the first real cut
 * of that ~8500-line file into separate modules (see CLAUDE.md). Moved
 * verbatim except for a handful of call sites that used to reach directly
 * into esp32-gen-ctl-ai-rtos.cpp's own file-static globals/functions —
 * those now go through small shims (fuel_level_from_adc(),
 * wifi_get_last_sta_creds(), shiftreg_write_buzzer() via buzzer.h) or
 * through types/externs promoted to rtos_config.h (SharedState_t/g_state,
 * LimitsConfig_t/g_limits_config, the SHIFT_REG/SHIFT_REG_IN enums,
 * state_name()) — see that header's own comments for why each one moved.
 *
 * Two-task split, unchanged from before the extraction:
 *   - DisplayTask (this file, bottom): encoder polling + menu navigation
 *     only, no I2C at all. Owns g_menu_input, mutates it, publishes a
 *     snapshot to xMenuRenderQueue.
 *   - i2c_render_display() (this file): called once per ~200ms from
 *     I2CTask (the sole OLED/I2C writer in the firmware). Owns g_menu (its
 *     own copy, refreshed from xMenuRenderQueue), draws it.
 * g_menu_input and g_menu are never touched by both tasks — see the
 * comment above their declarations below for how a consistent snapshot
 * crosses over.
 */

#include <stdint.h>
#include <stddef.h>
#include "uptime.h"
#include <ArduinoJson.h>
#include "wifi_net.h"
#include <esp_task_wdt.h>

#include "oled_menu.h"
#include "rtos_config.h"
#include "fram_log.h"
#include "oled_dev.h"
#include "encoder.h"
#include "smoothed_safe.h"
#include "buzzer.h"
#include "wifi_setup.h"
#include "debug.h"
#include "shared_defines.h" // RTOS_REG_ON/WIFI_HOSTNAME_DEF/sendOutputCommand()

// ---- externs into esp32-gen-ctl-ai-rtos.cpp's own globals -----------------
// Everything below is defined (non-static) in esp32-gen-ctl-ai-rtos.cpp;
// see that file's own comments at each definition for why it's exposed.

extern OledDev oled;

extern SmoothedSafe<double> adc_1_0;
extern SmoothedSafe<double> adc_1_1;
extern SmoothedSafe<double> adc_1_2;
extern SmoothedSafe<double> adc_1_3;
extern SmoothedSafe<double> adc_2_0;
extern SmoothedSafe<double> adc_2_1;

extern bool wifiModeIsAP;

// wifi_config_load()/wifi_config_save()/wifi_get_last_sta_creds() now come
// from wifi_setup.h (2026-09-24).
double fuel_level_from_adc(double adc_value);

// sendOutputCommand()/RTOS_REG_ON(REG)/WIFI_HOSTNAME_DEF now come from
// shared_defines.h (2026-09-25) — this file used to carry its own
// hand-written forward declaration and two macro duplicates instead of a
// real #include. WIFI_JSON_SIZE also used to be duplicated here, unused
// anywhere in this file — just dropped instead of centralized (config.h
// already has the real one, for the files that actually need it).

// ---- generic list/confirm screen engine's plain-data types ----------------
// (menu_list_render()/menu_list_process()/menu_confirm_render()/
// menu_confirm_process() and their callers all live further down in this
// same file now — these no longer need to be hoisted above anything.)

typedef struct {
    const char *label;      // static label, or NULL if label_fn is used
    void (*label_fn)(uint8_t idx, char *buf, size_t n);  // dynamic label, or NULL
    void (*on_select)(uint8_t idx);  // required — called on click (DisplayTask side)
    uint8_t idx;             // passed to label_fn/on_select — item identity
                              // (e.g. a FLOG_MAINT_* subtype), independent of
                              // the item's position in the array
} MenuListItem_t;

typedef struct {
    const MenuListItem_t *items;
    uint8_t count;
} MenuListDef_t;

typedef struct {
    void (*title_fn)(char *buf, size_t n);  // required — writes the question text
    void (*on_yes)(void);                    // side effect if confirmed; may be NULL
    void (*on_done)(void);                   // always runs after Yes or No — owns
                                              // the screen transition (mirrors
                                              // on_select's convention above)
} MenuConfirmDef_t;

// ============================================================================
// OLED Menu System
// ============================================================================
// Screen is OFF by default. Encoder button press wakes it up.
// 30 seconds of inactivity → screen off again.
// Navigation: rotate encoder = scroll, press button = select.
// ============================================================================

#define MENU_TIMEOUT_MS   30000   // 30 s inactivity → screen off
#define MENU_SSID_MAX     32
#define MENU_PASS_MAX     64
#define MENU_SCAN_MAX     16      // max scanned networks shown in list

// The menu's own copy of its scan results: restarting the multi-network
// connect right after the scan starts another scan that replaces the shared
// results, so reading them back later returned empty/other SSIDs. Written by
// DisplayTask before scan_done is published, read by the renderer after.
static char s_scan_ssids[MENU_SCAN_MAX][33];

// Character input alphabet; '<' = backspace, '>' = done (must be last two)
static const char MENU_CHARSET[] =
    " ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
    "0123456789-_.@!#$%^&*()+=<>";
#define MENU_CHARSET_LEN  ((int8_t)(sizeof(MENU_CHARSET) - 1))

typedef enum {
    MENU_OFF = 0,
    MENU_FUEL,           // fuel progress bar — shown while STATE_RUNNING
    MENU_MAIN,
    MENU_STATUS,
    MENU_GEN_CONFIRM,
    MENU_WIFI,
    MENU_WIFI_MODE,
    MENU_WIFI_SCAN,
    MENU_WIFI_SSID_INPUT,
    MENU_WIFI_PASS_INPUT,
    MENU_WIFI_SAVE,
    MENU_REBOOT_CONFIRM,
    MENU_MAINT,           // list of FLOG_MAINT_TYPES + "< Back"
    MENU_MAINT_CONFIRM,   // Yes/No before logging g_menu.maint_selected done
} MenuScreen_t;

typedef struct {
    MenuScreen_t  screen;
    uint32_t      last_activity;
    bool          display_on;

    // Shared by whichever generic list-select screen is currently shown
    // (MENU_MAIN, MENU_MAINT) — only one is ever visible at once, same
    // sharing convention confirm_cursor already used across the 3 confirm
    // screens. See MenuListDef_t/menu_list_process() below.
    int8_t        list_cursor;
    int8_t        confirm_cursor; // 0=Yes 1=No

    int8_t        status_page;   // 0-5

    int8_t        wifi_cursor;   // 0-3
    bool          wifi_mode_ap;
    char          wifi_ssid[MENU_SSID_MAX + 1];
    char          wifi_pass[MENU_PASS_MAX + 1];
    char         *wifi_input_buf;
    uint8_t       wifi_input_max;
    int8_t        wifi_char_idx;

    bool          scan_done;
    int8_t        scan_count;
    int8_t        scan_cursor;

    bool          gen_is_running;

    int8_t        maint_selected;  // FLOG_MAINT_* subtype chosen, pending confirm

    int32_t       enc_last_pos;
} MenuState_t;

// MenuState_t stays private to this file — setup_rtos() (main .cpp) needs
// its size to create xMenuRenderQueue but has no other business knowing its
// shape, hence this accessor instead of exposing the type itself.
size_t oled_menu_state_size(void) { return sizeof(MenuState_t); }

// g_menu_input is DisplayTask's private working copy — mutated only from
// menu_process_encoder()/menu_show()/menu_activity()/menu_init(). g_menu is
// I2CTask's private render-side copy — mutated only by i2c_render_display()
// pulling the latest snapshot off xMenuRenderQueue, read only by
// menu_render()/menu_render_*()/menu_oled_line()/menu_oled_item() below.
// The two are never touched by both tasks — see the queue publish at the
// end of menu_process_encoder() for how a consistent snapshot crosses over
// (in particular, why wifi_input_buf needs a pointer fixup there).
static MenuState_t g_menu_input;
static MenuState_t g_menu;

// Set by i2c_render_display() (I2CTask) the first time it shows the
// "Saving WiFi.../Restarting..." screen; cleared and acted on by
// menu_process_encoder() (DisplayTask). The one signal in this system that
// flows render→input instead of input→render — a plain volatile bool is
// enough (same simplicity level as ntp_sync_flag elsewhere in this file):
// single producer, single consumer, no data payload, and a one-tick delay
// in observing it is harmless (it's a "go reboot now" trigger, not
// timing-critical).
static volatile bool g_wifi_save_ready = false;

// ---- helpers ---------------------------------------------------------------

static void menu_activity() {
    g_menu_input.last_activity = uptime_ms();
    g_menu_input.display_on    = true;
}

static void menu_show(int s) {
    g_menu_input.screen = (MenuScreen_t)s;
    menu_activity();
}

// Write one display line (row 0-3).  Text is printf-formatted, padded to 21 chars.
static void menu_oled_line(uint8_t row, const char *fmt, ...) {
    char buf[22];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    size_t len = strlen(buf);
    while (len < 21) buf[len++] = ' ';
    buf[21] = '\0';
    oled.setCursor(0, row);
    oled.print(buf);
}

// Like menu_oled_line but prepends '>' or ' ' cursor indicator (uses 20 chars for text).
static void menu_oled_item(uint8_t row, bool sel, const char *fmt, ...) {
    char text[20];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);
    char buf[22];
    snprintf(buf, sizeof(buf), "%c %-18s", sel ? '>' : ' ', text);
    oled.setCursor(0, row);
    oled.print(buf);
}

// ---------------------------------------------------------------------------
// Generic "select list" and "Yes/No confirm" screen engine.
//
// A handful of screens (MENU_MAIN, MENU_MAINT, and the confirm dialogs) are
// nothing but "N items, pick one" or "Yes/No" — hand-rolling each one meant
// an item array + wrap arithmetic + windowed-scroll math + a switch mapping
// cursor position to an action, all duplicated per screen. That duplication
// is exactly what produced a real bug (a confirm screen's "return to main
// menu" landed on the wrong item because the item count changed in one
// place but a hardcoded return-index didn't). Table-driven instead: describe
// the items once as a static const array; the engine functions below do the
// cursor/window/dispatch arithmetic exactly once, for every such screen.
//
// Genuinely unique screens (WiFi text entry, network scan, the paged status
// screen, the fuel gauge) stay hand-written — they aren't "pick one of N",
// this engine wouldn't simplify them, just add a layer to route through.
// Both g_menu (I2CTask's render-side copy) and g_menu_input (DisplayTask's
// working copy) keep using the exact same split as every other screen —
// this is a rendering/authoring convenience, not a new ownership model.
// ---------------------------------------------------------------------------
// (MenuListItem_t/MenuListDef_t/MenuConfirmDef_t themselves are declared up
// near the top of the file, alongside the fram_log.h include — Arduino's
// ctags-based auto-prototype scanner inserts this file's function
// prototypes right after the #include block, before any type declared
// further down in the file exists yet, the same issue already hit with
// FLOG_MAINT_TYPES above.)

// I2CTask side — reads g_menu.list_cursor, writes the OLED.
static void menu_list_render(const MenuListDef_t *def) {
    int n = (int)def->count;
    int start = g_menu.list_cursor - 3;
    if (start < 0) start = 0;
    if (start + 4 > n) start = n - 4;
    if (start < 0) start = 0;
    for (int r = 0; r < 4; r++) {
        int i = start + r;
        if (i >= n) continue;
        const MenuListItem_t *it = &def->items[i];
        char buf[20];
        const char *text = it->label;
        if (it->label_fn) {
            it->label_fn(it->idx, buf, sizeof(buf));
            text = buf;
        }
        menu_oled_item(r, g_menu.list_cursor == i, "%s", text);
    }
}

// DisplayTask side — mutates g_menu_input.list_cursor; on_select runs at
// click time and owns the screen transition (calls menu_show() itself),
// same convention every hand-written screen below already follows.
static void menu_list_process(const MenuListDef_t *def, int delta, bool clicked) {
    int n = (int)def->count;
    if (delta != 0) {
        g_menu_input.list_cursor += (delta > 0) ? 1 : -1;
        if (g_menu_input.list_cursor < 0)  g_menu_input.list_cursor = (int8_t)(n - 1);
        if (g_menu_input.list_cursor >= n) g_menu_input.list_cursor = 0;
    }
    if (clicked) {
        const MenuListItem_t *it = &def->items[g_menu_input.list_cursor];
        it->on_select(it->idx);
    }
}

static void menu_confirm_render(const MenuConfirmDef_t *def) {
    char title[22];
    def->title_fn(title, sizeof(title));
    menu_oled_line(0, "%s", title);
    menu_oled_line(1, "");
    menu_oled_item(2, g_menu.confirm_cursor == 0, "Yes");
    menu_oled_item(3, g_menu.confirm_cursor == 1, "No");
}

static void menu_confirm_process(const MenuConfirmDef_t *def, int delta, bool clicked) {
    if (delta != 0) g_menu_input.confirm_cursor = (g_menu_input.confirm_cursor == 0) ? 1 : 0;
    if (clicked) {
        if (g_menu_input.confirm_cursor == 0 && def->on_yes) def->on_yes();
        def->on_done();
    }
}

// ---- render functions (called with I2C mutex held) -------------------------

// ---- MENU_MAIN: table-driven via the generic list engine above ------------

static bool mm_can_start(void) {
    state cur;
    uint32_t rpm;
    uint8_t inputs;
    portENTER_CRITICAL(&g_state_spinlock);
    cur    = g_state.current_state;
    rpm    = g_state.sensors.rpm;
    inputs = g_state.shift_reg_inputs;
    portEXIT_CRITICAL(&g_state_spinlock);
    return (cur == STATE_IDLE) && (rpm == 0) &&
           !(inputs & (1 << REG_IN_OUTPUT)) && !(inputs & (1 << REG_IN_GEN_SENSE));
}

static void mm_status_select(uint8_t idx) {
    (void)idx;
    g_menu_input.status_page = 0;
    menu_show(MENU_STATUS);
}

static void mm_generator_label(uint8_t idx, char *buf, size_t n) {
    (void)idx;
    snprintf(buf, n, "%s", mm_can_start() ? "Start Generator" : "Stop Generator");
}

static void mm_generator_select(uint8_t idx) {
    (void)idx;
    // gen_is_running=false → will send START; true → will send STOP
    g_menu_input.gen_is_running = !mm_can_start();
    g_menu_input.confirm_cursor = 1;  // default No
    menu_show(MENU_GEN_CONFIRM);
}

static void mm_wifi_select(uint8_t idx) {
    (void)idx;
    g_menu_input.wifi_mode_ap = wifiModeIsAP;
    g_menu_input.wifi_ssid[0] = '\0';
    g_menu_input.wifi_pass[0] = '\0';
    // Pre-fill from saved config
    JsonDocument doc;
    if (wifi_config_load(doc)) {
        if (g_menu_input.wifi_mode_ap) {
            strncpy(g_menu_input.wifi_ssid, doc["ap"]["ssid"]      | "", MENU_SSID_MAX);
            strncpy(g_menu_input.wifi_pass, doc["ap"]["password"]  | "", MENU_PASS_MAX);
        } else {
            JsonArrayConst nets = doc["sta"]["networks"];
            if (!nets.isNull() && nets.size() > 0) {
                strncpy(g_menu_input.wifi_ssid, nets[0]["ssid"]     | "", MENU_SSID_MAX);
                strncpy(g_menu_input.wifi_pass, nets[0]["password"] | "", MENU_PASS_MAX);
            }
        }
    }
    g_menu_input.wifi_ssid[MENU_SSID_MAX] = '\0';
    g_menu_input.wifi_pass[MENU_PASS_MAX] = '\0';
    g_menu_input.wifi_cursor = 0;
    menu_show(MENU_WIFI);
}

static void mm_battery_select(uint8_t idx) {
    (void)idx;
    RTOS_REG_ON(REG_BATT_OFF);  // direct action, no confirm
}

static bool mm_fuel_pump_on(void) {
    uint16_t outputs;
    portENTER_CRITICAL(&g_state_spinlock);
    outputs = g_state.shift_reg_outputs;
    portEXIT_CRITICAL(&g_state_spinlock);
    return outputs & (1 << REG_FUEL_PUMP);
}

static void mm_fuel_pump_label(uint8_t idx, char *buf, size_t n) {
    (void)idx;
    snprintf(buf, n, "%s", mm_fuel_pump_on() ? "Fuel pump: ON" : "Fuel pump: OFF");
}

// Toggle routed through xWebCommandQueue (not a direct register write) so
// it still goes through StateMachineTask's normal FUEL_PUMP validation
// (full-tank block).
static void mm_fuel_pump_select(uint8_t idx) {
    (void)idx;
    if (xWebCommandQueue != NULL) {
        WebCommand_t cmd = {};
        cmd.type   = WEB_CMD_REG_TOGGLE;
        cmd.reg    = REG_FUEL_PUMP;
        cmd.source = FLOG_SRC_PANEL;
        xQueueSend(xWebCommandQueue, &cmd, 0);
    }
}

static void mm_maintenance_select(uint8_t idx) {
    (void)idx;
    g_menu_input.list_cursor = 0;
    menu_show(MENU_MAINT);
}

static void mm_reboot_select(uint8_t idx) {
    (void)idx;
    g_menu_input.confirm_cursor = 1;  // default No
    menu_show(MENU_REBOOT_CONFIRM);
}

static void mm_exit_select(uint8_t idx) {
    (void)idx;
    state cur_s;
    portENTER_CRITICAL(&g_state_spinlock);
    cur_s = g_state.current_state;
    portEXIT_CRITICAL(&g_state_spinlock);
    if (cur_s == STATE_RUNNING) {
        menu_show(MENU_FUEL);
    } else {
        g_menu_input.screen     = MENU_OFF;
        g_menu_input.display_on = false;
    }
}

static const MenuListItem_t MENU_MAIN_ITEMS[] = {
    { "Status",         NULL,               mm_status_select,    0 },
    { NULL,              mm_generator_label, mm_generator_select, 0 },
    { "WiFi Settings",   NULL,               mm_wifi_select,      0 },
    { "Disable battery", NULL,               mm_battery_select,   0 },
    { NULL,              mm_fuel_pump_label, mm_fuel_pump_select, 0 },
    { "Maintenance",     NULL,               mm_maintenance_select, 0 },
    { "Reboot",          NULL,               mm_reboot_select,    0 },
    { "< Exit",          NULL,               mm_exit_select,      0 },
};
static const MenuListDef_t MENU_MAIN_DEF = { MENU_MAIN_ITEMS, 8 };

static void menu_render_main() {
    menu_list_render(&MENU_MAIN_DEF);
}

static void menu_render_status() {
    float rpm_f, freq_f, head_t, vrms, pw, pf, irms, pr;
    state cur;
    uint8_t inputs;
    uint64_t energy_in_mwh, energy_out_mwh;
    bool energy_valid;
    portENTER_CRITICAL(&g_state_spinlock);
    rpm_f         = (float)g_state.sensors.rpm;
    freq_f        = g_state.sensors.frequency;
    head_t        = g_state.sensors.head_temp;
    vrms          = g_state.power.rms_voltage;
    pw            = g_state.power.active_power;
    pf            = g_state.power.power_factor;
    irms          = g_state.power.rms_current;
    pr            = g_state.power.reactive_power;
    cur           = g_state.current_state;
    inputs        = g_state.shift_reg_inputs;
    energy_in_mwh  = g_state.power.import_active_energy_mwh;
    energy_out_mwh = g_state.power.export_active_energy_mwh;
    energy_valid   = g_state.power.energy_valid;
    portEXIT_CRITICAL(&g_state_spinlock);

    float fuel  = (float)fuel_level_from_adc(adc_2_0.get());
    float bat   = (float)adc_2_1.get();
    float oil_t = (float)adc_1_0.get();
    float air_t = (float)adc_1_1.get();
    float inv_t = (float)adc_1_2.get();
    float gen_t = (float)adc_1_3.get();

    switch (g_menu.status_page) {
        case 0:
            menu_oled_line(0, "-- Status 1/6 --");
            menu_oled_line(1, "Fuel: %.1f L", fuel);
            menu_oled_line(2, "Bat:  %.2f V", bat);
            menu_oled_line(3, "Head: %.1f C", head_t);
            break;
        case 1:
            menu_oled_line(0, "-- Status 2/6 --");
            menu_oled_line(1, "RPM: %.0f", rpm_f);
            menu_oled_line(2, "Hz:  %.1f", freq_f);
            menu_oled_line(3, "ROT=page BTN=back");
            break;
        case 2:
            menu_oled_line(0, "-- Status 3/6 --");
            menu_oled_line(1, "Oil:%.0f Air:%.0f", oil_t, air_t);
            menu_oled_line(2, "Inv:%.0f Gen:%.0f", inv_t, gen_t);
            menu_oled_line(3, "ROT=page BTN=back");
            break;
        case 3:
            menu_oled_line(0, "-- Status 4/6 --");
            menu_oled_line(1, "V:%.1f W:%.0f I:%.1f", vrms, pw, irms);
            menu_oled_line(2, "PF:%.2f Q:%.0f", pf, pr);
            menu_oled_line(3, "ROT=page BTN=back");
            break;
        case 4: {
            menu_oled_line(0, "-- Status 5/6 --");
            menu_oled_line(1, "State: %s", state_name(cur));
            // Safety-input trips take priority over RSSI on this row —
            // they're the more urgent thing to notice at a glance.
            char trips[22] = "";
            if (inputs & (1 << REG_IN_OIL))      strlcat(trips, "OIL,",   sizeof(trips));
            if (inputs & (1 << REG_IN_OVERHEAT)) strlcat(trips, "HEAT,",  sizeof(trips));
            if (inputs & (1 << REG_IN_OVERLOAD)) strlcat(trips, "OLOAD,", sizeof(trips));
            size_t tlen = strlen(trips);
            if (tlen > 0) trips[tlen - 1] = 0;   // strip trailing comma
            if (trips[0]) {
                menu_oled_line(2, "TRIP: %s", trips);
            } else {
                menu_oled_line(2, "Prot:OK RSSI:%d", (int)wifi_net_rssi());
            }
            menu_oled_line(3, "ROT=page BTN=back");
            break;
        }
        case 5: {
            menu_oled_line(0, "-- Status 6/6 --");
            double kwh_in  = energy_valid ? (double)energy_in_mwh  / 1000000.0 : 0.0;
            double kwh_out = energy_valid ? (double)energy_out_mwh / 1000000.0 : 0.0;
            menu_oled_line(1, "In:  %.1f kWh", kwh_in);
            menu_oled_line(2, "Out: %.1f kWh", kwh_out);
            menu_oled_line(3, "ROT=page BTN=back");
            break;
        }
    }
}

// ---- MENU_GEN_CONFIRM: table-driven via the generic confirm engine above --

static void gc_title(char *buf, size_t n) {
    snprintf(buf, n, "%s", g_menu.gen_is_running ? "Stop generator?" : "Start generator?");
}

static void gc_yes(void) {
    if (xWebCommandQueue != NULL) {
        WebCommand_t cmd = {};
        cmd.type   = g_menu_input.gen_is_running ? WEB_CMD_STOP : WEB_CMD_START;
        cmd.reg    = 0;
        cmd.source = FLOG_SRC_PANEL;
        xQueueSend(xWebCommandQueue, &cmd, 0);
        buzzerBeepOnce(BUZZER_LONG_BEEP_MS);
    }
}

static void gc_done(void) {
    g_menu_input.list_cursor = 1;
    menu_show(MENU_MAIN);
}

static const MenuConfirmDef_t GEN_CONFIRM_DEF = { gc_title, gc_yes, gc_done };

static void menu_render_gen_confirm() {
    menu_confirm_render(&GEN_CONFIRM_DEF);
}

static void menu_render_wifi() {
    // 5 items (0-4), show 4 at a time with scrolling
    const char *net_item = g_menu.wifi_mode_ap ? "AP Name" : "Select Network";
    char mode_str[20];
    snprintf(mode_str, sizeof(mode_str), "Mode: %s", g_menu.wifi_mode_ap ? "AP" : "STA");
    const char *items[5] = { mode_str, net_item, "Password", "Save & Apply", "< Back" };

    int start = g_menu.wifi_cursor - 3;
    if (start < 0) start = 0;
    if (start + 4 > 5) start = 1;  // last window: items 1-4
    for (int r = 0; r < 4; r++)
        menu_oled_item(r, g_menu.wifi_cursor == start + r, items[start + r]);
}

static void menu_render_wifi_mode() {
    menu_oled_line(0, "WiFi Mode:");
    menu_oled_item(1, !g_menu.wifi_mode_ap, "STA (Client)");
    menu_oled_item(2,  g_menu.wifi_mode_ap, "AP (Hotspot)");
    menu_oled_line(3, "ROT=toggle BTN=ok");
}

static void menu_render_wifi_scan() {
    if (!g_menu.scan_done) {
        int n = wifi_net_scan_result();
        // Should not happen (scan is now synchronous), but guard just in case
        if (n == WIFI_NET_SCAN_RUNNING) {
            menu_oled_line(0, "Scanning WiFi...");
            menu_oled_line(1, "");
            menu_oled_line(2, "Please wait");
            menu_oled_line(3, "");
            return;
        }
        g_menu.scan_count = (n > 0) ? ((n > MENU_SCAN_MAX) ? MENU_SCAN_MAX : (int8_t)n) : 0;
        g_menu.scan_done  = true;
    }
    if (g_menu.scan_count == 0) {
        menu_oled_line(0, "No networks found");
        menu_oled_line(1, "BTN=back");
        menu_oled_line(2, "");
        menu_oled_line(3, "");
        return;
    }
    // Virtual list: indices 0..scan_count-1 = networks, scan_count = Cancel
    int total = g_menu.scan_count + 1;  // +1 for Cancel
    int start = g_menu.scan_cursor - 1;
    if (start < 0) start = 0;
    if (start + 3 > total) start = total - 3;
    if (start < 0) start = 0;
    menu_oled_line(0, "Select network:");
    for (int r = 0; r < 3; r++) {
        int idx = start + r;
        if (idx >= total) { menu_oled_line(r + 1, ""); continue; }
        bool sel = (idx == g_menu.scan_cursor);
        if (idx < g_menu.scan_count)
            menu_oled_item(r + 1, sel, "%.18s", s_scan_ssids[idx]);
        else
            menu_oled_item(r + 1, sel, "[ Cancel ]");
    }
}

static void menu_render_text_input(const char *title) {
    menu_oled_line(0, title);
    uint8_t len = (uint8_t)strlen(g_menu.wifi_input_buf);
    const char *disp = g_menu.wifi_input_buf;
    if (len > 18) disp = g_menu.wifi_input_buf + len - 18;
    menu_oled_line(1, "%-18s", disp);
    char cc = MENU_CHARSET[(uint8_t)g_menu.wifi_char_idx];
    if      (cc == '<') menu_oled_line(2, "[< DEL ]");
    else if (cc == '>') menu_oled_line(2, "[> DONE]");
    else                menu_oled_line(2, "[ %c ]", cc);
    menu_oled_line(3, "ROT=char BTN=sel");
}

static void menu_render_wifi_save() {
    menu_oled_line(0, "Saving WiFi...");
    menu_oled_line(1, "");
    menu_oled_line(2, "Restarting...");
    menu_oled_line(3, "");
    g_wifi_save_ready = true;
}

// Draw a 2-row (16 px) progress bar on pages row and row+1.
// Uses raw SSD1306 RAM writes: 0xFF = filled, border pixels on empty portion.
#define FUEL_MAX_LITERS  8.65f
static void menu_draw_progress_bar(uint8_t row, float pct) {
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    // Layout: col0=margin, col1=border, col2..125=bar(124px), col126=border, col127=margin
    const int BAR_COLS = 124;
    int filled = (int)(pct / 100.0f * BAR_COLS + 0.5f);
    if (filled > BAR_COLS) filled = BAR_COLS;
    for (int page = (int)row; page <= (int)row + 1; page++) {
        oled.setCursor(0, page);
        oled.ssd1306WriteRam(0x00);   // left margin
        oled.ssd1306WriteRam(0xFF);   // left border
        for (int c = 0; c < BAR_COLS; c++) {
            if (c < filled) {
                oled.ssd1306WriteRam(0xFF);
            } else {
                // only top/bottom edge pixel of the empty zone
                oled.ssd1306WriteRam(page == (int)row ? 0x01 : 0x80);
            }
        }
        oled.ssd1306WriteRam(0xFF);   // right border
        oled.ssd1306WriteRam(0x00);   // right margin
    }
}

static void menu_render_fuel() {
    float fuel_l = (float)fuel_level_from_adc(adc_2_0.get());
    if (fuel_l < 0.0f) fuel_l = 0.0f;
    float pct = fuel_l / FUEL_MAX_LITERS * 100.0f;
    if (pct > 100.0f) pct = 100.0f;

    uint32_t rpm;
    portENTER_CRITICAL(&g_state_spinlock);
    rpm = g_state.sensors.rpm;
    portEXIT_CRITICAL(&g_state_spinlock);

    menu_oled_line(0, "Fuel:%.1fL %.0f%% R:%lu", fuel_l, pct, (unsigned long)rpm);
    menu_draw_progress_bar(1, pct);
    menu_oled_line(3, "BTN=menu");
}

// ---- MENU_REBOOT_CONFIRM: table-driven via the generic confirm engine -----

static void rc_title(char *buf, size_t n) {
    snprintf(buf, n, "Reboot controller?");
}

static void rc_yes(void) {
    esp_restart();  // does not return
}

static void rc_done(void) {
    g_menu_input.list_cursor = 6;  // "Reboot" itself, see MENU_MAIN_ITEMS
    menu_show(MENU_MAIN);
}

static const MenuConfirmDef_t REBOOT_CONFIRM_DEF = { rc_title, rc_yes, rc_done };

static void menu_render_reboot_confirm() {
    menu_confirm_render(&REBOOT_CONFIRM_DEF);
}

// ---- MENU_MAINT: table-driven via the generic list engine above -----------
// hours_since/due read straight from the cached FramMetrics
// (fram_metrics_get() — no I2C transaction, safe to call from this render
// pass same as ReportTask's identical MAINT_DUE computation).

static_assert(FLOG_MAINT_TYPES == 6,
    "MENU_MAINT_ITEMS below has one entry per FLOG_MAINT_* subtype — add/remove a row to match");

static void mt_label(uint8_t idx, char *buf, size_t n) {
    FramMetrics fm = fram_metrics_get();
    double since_h = fram_maint_hours_since(&fm, idx);
    bool due = g_limits_config.maint_interval_hours[idx] > 0.0f &&
               since_h >= (double)g_limits_config.maint_interval_hours[idx];
    snprintf(buf, n, "%.12s %.0fh%s", MAINT_TYPE_LABELS[idx], since_h, due ? " DUE" : "");
}

static void mt_select(uint8_t idx) {
    g_menu_input.maint_selected = idx;
    g_menu_input.confirm_cursor = 1;  // default No
    menu_show(MENU_MAINT_CONFIRM);
}

static void mt_back_select(uint8_t idx) {
    (void)idx;
    g_menu_input.list_cursor = 5;  // "Maintenance" item in MENU_MAIN_ITEMS
    menu_show(MENU_MAIN);
}

static const MenuListItem_t MENU_MAINT_ITEMS[] = {
    { NULL,     mt_label, mt_select, 0 },
    { NULL,     mt_label, mt_select, 1 },
    { NULL,     mt_label, mt_select, 2 },
    { NULL,     mt_label, mt_select, 3 },
    { NULL,     mt_label, mt_select, 4 },
    { NULL,     mt_label, mt_select, 5 },
    { "< Back", NULL,     mt_back_select, 0 },
};
static const MenuListDef_t MENU_MAINT_DEF = { MENU_MAINT_ITEMS, FLOG_MAINT_TYPES + 1 };

static void menu_render_maint() {
    menu_list_render(&MENU_MAINT_DEF);
}

// ---- MENU_MAINT_CONFIRM: table-driven via the generic confirm engine ------

static void mc_title(char *buf, size_t n) {
    snprintf(buf, n, "Record %.12s?", MAINT_TYPE_LABELS[g_menu.maint_selected]);
}

static void mc_yes(void) {
    fram_log_maintenance((uint8_t)g_menu_input.maint_selected, "panel");
    buzzerBeepOnce(BUZZER_SHORT_BEEP_MS);
}

static void mc_done(void) {
    // Back to the list (not MENU_MAIN) — lets the operator log several
    // types in one pass without re-navigating each time.
    menu_show(MENU_MAINT);
}

static const MenuConfirmDef_t MAINT_CONFIRM_DEF = { mc_title, mc_yes, mc_done };

static void menu_render_maint_confirm() {
    menu_confirm_render(&MAINT_CONFIRM_DEF);
}

// Pure render — reads g_menu only (I2CTask's own copy, refreshed from
// xMenuRenderQueue just before this is called). The screen-timeout/auto-
// fuel-display decision used to live here but belongs with the
// navigation-state owner instead — see menu_process_encoder().
static void menu_render() {
    oled.beginFrame();  // reset the writeDisplay() failure latch for this pass

    if (!g_menu.display_on) {
        oled.ssd1306WriteCmd(0xAE);  // display OFF
        return;
    }

    oled.ssd1306WriteCmd(0xAF);  // display ON

    switch (g_menu.screen) {
        case MENU_OFF:              oled.clear();                            break;
        case MENU_FUEL:             menu_render_fuel();                      break;
        case MENU_MAIN:             menu_render_main();                      break;
        case MENU_STATUS:           menu_render_status();                    break;
        case MENU_GEN_CONFIRM:      menu_render_gen_confirm();               break;
        case MENU_WIFI:             menu_render_wifi();                      break;
        case MENU_WIFI_MODE:        menu_render_wifi_mode();                 break;
        case MENU_WIFI_SCAN:        menu_render_wifi_scan();                 break;
        case MENU_WIFI_SSID_INPUT:  menu_render_text_input("AP Name:");      break;
        case MENU_WIFI_PASS_INPUT:  menu_render_text_input("Password:");     break;
        case MENU_WIFI_SAVE:        menu_render_wifi_save();                 break;
        case MENU_REBOOT_CONFIRM:   menu_render_reboot_confirm();            break;
        case MENU_MAINT:            menu_render_maint();                     break;
        case MENU_MAINT_CONFIRM:    menu_render_maint_confirm();             break;
    }
}

// Called once per ~200ms from I2CTask (its sole owner). Pulls the latest
// menu snapshot published by menu_process_encoder() (DisplayTask) off
// xMenuRenderQueue into g_menu, then renders it — see the MenuState_t
// split comment above g_menu_input's declaration.
void i2c_render_display(void) {
    if (xMenuRenderQueue != NULL) {
        xQueuePeek(xMenuRenderQueue, &g_menu, 0);  // non-destructive: always render the latest
    }
    menu_render();
}

// ---- WiFi save helper -------------------------------------------------------

static void menu_do_wifi_save() {
    JsonDocument doc;
    doc["mode"]     = g_menu.wifi_mode_ap ? "ap" : "sta";
    doc["hostname"] = WIFI_HOSTNAME_DEF;
    if (g_menu.wifi_mode_ap) {
        JsonObject ap = doc["ap"].to<JsonObject>();
        ap["ssid"]     = g_menu.wifi_ssid;
        ap["password"] = g_menu.wifi_pass;
        ap["channel"]  = 1;
        ap["hidden"]   = false;
        ap["max_conn"] = 4;
        ap["ip"]       = "192.168.4.1";
        ap["gw"]       = "192.168.4.1";
        ap["sn"]       = "255.255.255.0";
    } else {
        JsonObject sta    = doc["sta"].to<JsonObject>();
        sta["dhcp"]       = true;
        JsonArray  nets   = sta["networks"].to<JsonArray>();
        JsonObject net    = nets.add<JsonObject>();
        net["ssid"]       = g_menu.wifi_ssid;
        net["password"]   = g_menu.wifi_pass;
    }
    wifi_config_save(doc);
}

// ---- Encoder input processing (no I2C mutex needed) ------------------------

// Publishes a self-contained snapshot of g_menu_input to xMenuRenderQueue —
// I2CTask picks up the latest one before each render pass. The wifi_input_buf
// pointer is fixed up to target the snapshot's own wifi_ssid/wifi_pass array
// instead of g_menu_input's, since a plain struct copy would otherwise leave
// it pointing at a buffer DisplayTask keeps mutating one character at a time
// during SSID/password entry.
static void menu_publish_snapshot() {
    MenuState_t snap = g_menu_input;
    if (g_menu_input.wifi_input_buf == g_menu_input.wifi_ssid) snap.wifi_input_buf = snap.wifi_ssid;
    else if (g_menu_input.wifi_input_buf == g_menu_input.wifi_pass) snap.wifi_input_buf = snap.wifi_pass;
    if (xMenuRenderQueue != NULL) QUEUE_OVERWRITE(xMenuRenderQueue, &snap);
}

static void menu_process_encoder() {
    // If the "Saving WiFi..." screen was just shown by i2c_render_display(),
    // do the actual save+reboot now. This is the one signal that flows
    // render→input instead of input→render (see g_wifi_save_ready).
    if (g_wifi_save_ready) {
        g_wifi_save_ready = false;
        menu_do_wifi_save();
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
        return;
    }

    int32_t pos   = encoder_get_position();
    int32_t delta = pos - g_menu_input.enc_last_pos;
    g_menu_input.enc_last_pos = pos;
    bool clicked  = encoder_button_clicked();

    if (delta  != 0) menu_activity();
    if (clicked)     menu_activity();

    // Auto-manage fuel screen based on generator state
    {
        state cur_s;
        portENTER_CRITICAL(&g_state_spinlock);
        cur_s = g_state.current_state;
        portEXIT_CRITICAL(&g_state_spinlock);
        if (cur_s == STATE_RUNNING) {
            // Auto-show fuel display when screen is off or blank
            if (!g_menu_input.display_on || g_menu_input.screen == MENU_OFF) {
                g_menu_input.display_on = true;
                g_menu_input.screen     = MENU_FUEL;
            }
        } else {
            // Generator stopped — leave fuel screen
            if (g_menu_input.screen == MENU_FUEL) {
                g_menu_input.screen     = MENU_OFF;
                g_menu_input.display_on = false;
            }
        }
    }

    // Timeout check (MENU_FUEL is persistent — no timeout while running).
    // Used to live in menu_render() — moved here since it's a navigation
    // decision, not a rendering one, and belongs with the state owner.
    if (g_menu_input.screen != MENU_OFF && g_menu_input.screen != MENU_FUEL &&
        (uptime_ms() - g_menu_input.last_activity) > MENU_TIMEOUT_MS) {
        state cur_s;
        portENTER_CRITICAL(&g_state_spinlock);
        cur_s = g_state.current_state;
        portEXIT_CRITICAL(&g_state_spinlock);
        if (cur_s == STATE_RUNNING) {
            g_menu_input.screen = MENU_FUEL;   // return to fuel display
        } else {
            g_menu_input.screen     = MENU_OFF;
            g_menu_input.display_on = false;
        }
    }

    // Screen is off → only button wakes it (to main menu)
    if (!g_menu_input.display_on || g_menu_input.screen == MENU_OFF) {
        if (clicked) {
            g_menu_input.display_on  = true;
            g_menu_input.list_cursor = 0;
            menu_show(MENU_MAIN);
        }
        menu_publish_snapshot();
        return;
    }

    if (delta == 0 && !clicked) { menu_publish_snapshot(); return; }  // nothing to process

    switch (g_menu_input.screen) {

        // ------------------------------------------------------------------
        case MENU_FUEL:
            if (clicked) {
                g_menu_input.list_cursor = 0;
                menu_show(MENU_MAIN);
            }
            break;

        // ------------------------------------------------------------------
        case MENU_MAIN:
            menu_list_process(&MENU_MAIN_DEF, delta, clicked);
            break;

        // ------------------------------------------------------------------
        case MENU_STATUS:
            if (delta != 0) {
                g_menu_input.status_page += (delta > 0) ? 1 : -1;
                if (g_menu_input.status_page < 0) g_menu_input.status_page = 5;
                if (g_menu_input.status_page > 5) g_menu_input.status_page = 0;
            }
            if (clicked) menu_show(MENU_MAIN);
            break;

        // ------------------------------------------------------------------
        case MENU_GEN_CONFIRM:
            menu_confirm_process(&GEN_CONFIRM_DEF, delta, clicked);
            break;

        // ------------------------------------------------------------------
        case MENU_WIFI:
            if (delta != 0) {
                g_menu_input.wifi_cursor += (delta > 0) ? 1 : -1;
                if (g_menu_input.wifi_cursor < 0) g_menu_input.wifi_cursor = 4;
                if (g_menu_input.wifi_cursor > 4) g_menu_input.wifi_cursor = 0;
            }
            if (clicked) {
                switch (g_menu_input.wifi_cursor) {
                    case 0:
                        menu_show(MENU_WIFI_MODE);
                        break;
                    case 1:
                        if (!g_menu_input.wifi_mode_ap) {
                            g_menu_input.scan_done   = false;
                            g_menu_input.scan_count  = 0;
                            g_menu_input.scan_cursor = 0;
                            menu_show(MENU_WIFI_SCAN);
                            // No more direct OLED write here to show
                            // "Scanning..." before the blocking call below —
                            // that used to be an unprotected I2C write
                            // outside the mutex. Now that i2c_render_display()
                            // runs independently on its own ~200ms cadence in
                            // I2CTask, publishing the MENU_WIFI_SCAN screen
                            // state (right below) is enough: the next render
                            // pass picks it up and shows "Scanning..." on its
                            // own, with zero I2C touched from this task.
                            menu_publish_snapshot();
                            // Pause the multi-network connect, do sync scan, then restore
                            wifi_net_multi_clear();  // status Idle: its scan-done handling is skipped
                            vTaskDelay(pdMS_TO_TICKS(150));
                            esp_task_wdt_reset();  // scan is usually ~2-3s but not hard-bounded
                            int n = wifi_net_scan_blocking();  // ~2-3s
                            esp_task_wdt_reset();
                            g_menu_input.scan_count = (n > 0) ? ((n > MENU_SCAN_MAX) ? MENU_SCAN_MAX : (int8_t)n) : 0;
                            for (int i = 0; i < g_menu_input.scan_count; i++) {
                                wifi_net_scan_ssid(i, s_scan_ssids[i], sizeof(s_scan_ssids[i]));
                            }
                            g_menu_input.scan_done  = true;
                            // Restore AP list and restart async connection
                            {
                                char sta_ssid[33], sta_pass[65];
                                wifi_get_last_sta_creds(sta_ssid, sizeof(sta_ssid), sta_pass, sizeof(sta_pass));
                                if (strlen(sta_ssid) > 0) wifi_net_multi_add(sta_ssid, sta_pass);
                            }
                            wifi_net_multi_start();
                        } else {
                            g_menu_input.wifi_input_buf = g_menu_input.wifi_ssid;
                            g_menu_input.wifi_input_max = MENU_SSID_MAX;
                            g_menu_input.wifi_char_idx  = 0;
                            menu_show(MENU_WIFI_SSID_INPUT);
                        }
                        break;
                    case 2:
                        g_menu_input.wifi_input_buf = g_menu_input.wifi_pass;
                        g_menu_input.wifi_input_max = MENU_PASS_MAX;
                        g_menu_input.wifi_char_idx  = 0;
                        menu_show(MENU_WIFI_PASS_INPUT);
                        break;
                    case 3:
                        g_wifi_save_ready = false;
                        menu_show(MENU_WIFI_SAVE);
                        break;
                    case 4:  // Back — discard changes
                        g_menu_input.list_cursor = 2;  // "WiFi Settings" in MENU_MAIN_ITEMS
                        menu_show(MENU_MAIN);
                        break;
                }
            }
            break;

        // ------------------------------------------------------------------
        case MENU_WIFI_MODE:
            if (delta != 0) g_menu_input.wifi_mode_ap = !g_menu_input.wifi_mode_ap;
            if (clicked)    menu_show(MENU_WIFI);
            break;

        // ------------------------------------------------------------------
        case MENU_WIFI_SCAN:
            if (!g_menu_input.scan_done) break;  // still scanning
            if (g_menu_input.scan_count == 0) {
                if (clicked) menu_show(MENU_WIFI);
                break;
            }
            {
                int total_scan = g_menu_input.scan_count + 1;  // +1 for Cancel
                if (delta != 0) {
                    g_menu_input.scan_cursor += (delta > 0) ? 1 : -1;
                    if (g_menu_input.scan_cursor < 0)              g_menu_input.scan_cursor = (int8_t)(total_scan - 1);
                    if (g_menu_input.scan_cursor >= total_scan)    g_menu_input.scan_cursor = 0;
                }
                if (clicked) {
                    if (g_menu_input.scan_cursor == g_menu_input.scan_count) {
                        // Cancel selected
                        wifi_net_scan_clear();
                        menu_show(MENU_WIFI);
                    } else {
                        strlcpy(g_menu_input.wifi_ssid, s_scan_ssids[g_menu_input.scan_cursor], MENU_SSID_MAX + 1);
                        wifi_net_scan_clear();
                        menu_show(MENU_WIFI);
                    }
                }
            }
            break;

        // ------------------------------------------------------------------
        case MENU_WIFI_SSID_INPUT:
        case MENU_WIFI_PASS_INPUT:
            if (delta != 0) {
                g_menu_input.wifi_char_idx += (delta > 0) ? 1 : -1;
                if (g_menu_input.wifi_char_idx < 0)              g_menu_input.wifi_char_idx = MENU_CHARSET_LEN - 1;
                if (g_menu_input.wifi_char_idx >= MENU_CHARSET_LEN) g_menu_input.wifi_char_idx = 0;
            }
            if (clicked) {
                char cc  = MENU_CHARSET[(uint8_t)g_menu_input.wifi_char_idx];
                uint8_t len = (uint8_t)strlen(g_menu_input.wifi_input_buf);
                if (cc == '>') {
                    menu_show(MENU_WIFI);
                } else if (cc == '<') {
                    if (len > 0) g_menu_input.wifi_input_buf[len - 1] = '\0';
                } else if (len < g_menu_input.wifi_input_max) {
                    g_menu_input.wifi_input_buf[len]     = cc;
                    g_menu_input.wifi_input_buf[len + 1] = '\0';
                }
            }
            break;

        // ------------------------------------------------------------------
        case MENU_REBOOT_CONFIRM:
            menu_confirm_process(&REBOOT_CONFIRM_DEF, delta, clicked);
            break;

        // ------------------------------------------------------------------
        case MENU_MAINT:
            menu_list_process(&MENU_MAINT_DEF, delta, clicked);
            break;

        // ------------------------------------------------------------------
        case MENU_MAINT_CONFIRM:
            // mc_done() (see MAINT_CONFIRM_DEF) sends us back to MENU_MAINT,
            // not MENU_MAIN — lets the operator log several types in one
            // pass without re-navigating each time.
            menu_confirm_process(&MAINT_CONFIRM_DEF, delta, clicked);
            break;

        default: break;
    }

    menu_publish_snapshot();
}

static void menu_init() {
    memset(&g_menu_input, 0, sizeof(g_menu_input));
    g_menu_input.screen      = MENU_OFF;
    g_menu_input.display_on  = false;
    g_menu_input.wifi_mode_ap = wifiModeIsAP;
    g_menu_input.enc_last_pos = encoder_get_position();
    menu_publish_snapshot();  // seed xMenuRenderQueue so I2CTask's first render tick has something
}

// ----------------------------------------------------------------------------
// DisplayTask: OLED menu + display
// ----------------------------------------------------------------------------
// Encoder polling + menu navigation only — no I2C at all. OLED rendering
// moved to I2CTask's i2c_render_display(), decoupled via xMenuRenderQueue
// (see MenuState_t/g_menu_input above).
void DisplayTask(void *pvParameters) {
    (void)pvParameters;
    esp_task_wdt_add(NULL);  // self-register before first wdt_reset
    TickType_t xLastWakeTime = xTaskGetTickCount();

    menu_init();
    SLOG_PRINTLN("DisplayTask started");

    for (;;) {
        // Process encoder every tick (50 ms) — fast response
        menu_process_encoder();

        // Feed watchdog
        esp_task_wdt_reset();

        // Wait for next tick
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PERIOD_DISPLAY_MS));
    }
}
