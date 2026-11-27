/*
 * ui.cpp — LVGL v8 UI for Generator Control Panel
 * Display: 480×320 (landscape, after 90° rotation of 320×480 physical)
 */
#include "ui.h"
#include "esp_now_client.h"
#include "udp_discovery.h"
#include "config.h"
#include <WiFi.h>
#include <stdio.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Palette
// ---------------------------------------------------------------------------
#define COL_BG        lv_color_hex(0x0d1117)
#define COL_CARD      lv_color_hex(0x161b22)
#define COL_BORDER    lv_color_hex(0x30363d)
#define COL_TXT       lv_color_hex(0xe6edf3)
#define COL_TXT2      lv_color_hex(0x8b949e)
#define COL_GREEN     lv_color_hex(0x3fb950)
#define COL_RED       lv_color_hex(0xf85149)
#define COL_ORANGE    lv_color_hex(0xd29922)
#define COL_BLUE      lv_color_hex(0x388bfd)
#define COL_ACCENT    lv_color_hex(0x1f6feb)

// ---------------------------------------------------------------------------
// Dimensions
// ---------------------------------------------------------------------------
#define TOPBAR_H   30
#define TABBAR_H   38
#define CONTENT_H  (UI_H - TOPBAR_H - TABBAR_H)   // 252

// ---------------------------------------------------------------------------
// Widget handles (top bar)
// ---------------------------------------------------------------------------
static lv_obj_t *wifi_bar[4];  // signal-strength bars, top-bar left edge, ascending height
static lv_obj_t *lbl_ws;
static lv_obj_t *lbl_state;
static lv_obj_t *lbl_panel_bat;  // this panel's Li-ion, hidden until a cell is detected

// ---------------------------------------------------------------------------
// Dashboard tab
// ---------------------------------------------------------------------------
static lv_obj_t *lbl_gen_state;
static lv_obj_t *lbl_rpm;
static lv_obj_t *lbl_bat;
static lv_obj_t *lbl_volt;
static lv_obj_t *lbl_power;
static lv_obj_t *lbl_temp;
static lv_obj_t *lbl_fuel;
static lv_obj_t *arc_rpm;
static lv_obj_t *arc_bat;
static lv_obj_t *arc_volt;
static lv_obj_t *arc_power;
static lv_obj_t *arc_temp;
static lv_obj_t *arc_fuel;

#define CHART_POINTS 60   // 60 samples × ~1s = 1 min history

static lv_obj_t          *chart_rpm,  *chart_volt, *chart_curr, *chart_pwr;
static lv_obj_t          *chart_head, *chart_amb,  *chart_oil,  *chart_inv, *chart_alt;
static lv_chart_series_t *ser_rpm,    *ser_volt,   *ser_curr,   *ser_pwr;
static lv_chart_series_t *ser_head,   *ser_amb,    *ser_oil,    *ser_inv,   *ser_alt;
// Current-value labels (top-right corner of each chart container)
static lv_obj_t *cv_rpm, *cv_volt, *cv_curr, *cv_pwr;
static lv_obj_t *cv_head, *cv_amb, *cv_oil, *cv_inv, *cv_alt;

// ---------------------------------------------------------------------------
// Controls tab
// ---------------------------------------------------------------------------
static lv_obj_t *btn_start;
static lv_obj_t *btn_retry;
static lv_obj_t *btn_stop_ctrl;   // STOP button (needs WS lock)
static lv_obj_t *sw_choke;
static lv_obj_t *sw_fuel;
static lv_obj_t *sw_eco;
static lv_obj_t *sw_load;
static lv_obj_t *sw_charger;

static lv_obj_t *ind_overload;
static lv_obj_t *ind_oil;
static lv_obj_t *ind_output;
static lv_obj_t *ind_overheat;
static lv_obj_t *ind_gen;
static lv_obj_t *ind_bat;

// Overdue-maintenance bar — hidden while maint_due_mask == 0
static lv_obj_t *maint_bar;
static lv_obj_t *maint_bar_lbl;
static lv_obj_t *maint_bar_btn;
static uint8_t   s_maint_mask = 0;      // last mask seen (change-detect for the bar text)
static lv_obj_t *maint_mbox = nullptr;  // type-picker popup, one at a time

// ---------------------------------------------------------------------------
// Power tab
// ---------------------------------------------------------------------------
static lv_obj_t *lbl_vrms;
static lv_obj_t *lbl_irms;
static lv_obj_t *lbl_pw;
static lv_obj_t *lbl_pa;
static lv_obj_t *lbl_pr;
static lv_obj_t *lbl_pf;
static lv_obj_t *lbl_hz;
static lv_obj_t *lbl_htemp;
static lv_obj_t *lbl_fuel2;
static lv_obj_t *lbl_uptime;
static lv_obj_t *lbl_rssi;
static lv_obj_t *lbl_bat_v;
static lv_obj_t *lbl_core_v;
static lv_obj_t *lbl_oil_t;
static lv_obj_t *lbl_amb_t;
static lv_obj_t *lbl_inv_t;
static lv_obj_t *lbl_alt_t;

// ---------------------------------------------------------------------------
// Journal tab
// ---------------------------------------------------------------------------
static lv_obj_t *lbl_j_metrics;
static lv_obj_t *lbl_j_status;
static lv_obj_t *list_journal;
#define TAB_JOURNAL_IDX 3

// ---------------------------------------------------------------------------
// Config tab
// ---------------------------------------------------------------------------
static lv_obj_t *list_networks;
static lv_obj_t *ta_ssid;
static lv_obj_t *ta_host;
static lv_obj_t *dd_gen;   // Config tab: ESP-NOW generator picker
static lv_obj_t *ta_pass;
static lv_obj_t *kb_popup;       // keyboard shown over screen
static lv_obj_t *kb_ta_active;   // which textarea is being edited
static lv_obj_t *kb_preview_label;  // echoes ta text above the keyboard, since the keyboard covers the field
static lv_timer_t *kb_preview_timer;  // polls kb_preview_refresh() so cursor moves (no VALUE_CHANGED event) still show

// Connect callback
static void (*connect_cb)(const char *, const char *, const char *) = nullptr;

static bool s_ws_connected = false;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static lv_style_t sty_card, sty_btn_green, sty_btn_red, sty_btn_blue, sty_btn_orange;
static lv_style_t sty_topbar, sty_switch_on;

static void style_init(void) {
    // Card style
    lv_style_init(&sty_card);
    lv_style_set_bg_color(&sty_card, COL_CARD);
    lv_style_set_bg_opa(&sty_card, LV_OPA_COVER);
    lv_style_set_border_color(&sty_card, COL_BORDER);
    lv_style_set_border_width(&sty_card, 1);
    lv_style_set_radius(&sty_card, 6);
    lv_style_set_pad_all(&sty_card, 6);

    // Button styles
    lv_style_init(&sty_btn_green);
    lv_style_set_bg_color(&sty_btn_green, COL_GREEN);
    lv_style_set_bg_opa(&sty_btn_green, LV_OPA_COVER);
    lv_style_set_text_color(&sty_btn_green, lv_color_hex(0x000000));
    lv_style_set_radius(&sty_btn_green, 6);

    lv_style_init(&sty_btn_red);
    lv_style_set_bg_color(&sty_btn_red, COL_RED);
    lv_style_set_bg_opa(&sty_btn_red, LV_OPA_COVER);
    lv_style_set_text_color(&sty_btn_red, COL_TXT);
    lv_style_set_radius(&sty_btn_red, 6);

    lv_style_init(&sty_btn_blue);
    lv_style_set_bg_color(&sty_btn_blue, COL_ACCENT);
    lv_style_set_bg_opa(&sty_btn_blue, LV_OPA_COVER);
    lv_style_set_text_color(&sty_btn_blue, COL_TXT);
    lv_style_set_radius(&sty_btn_blue, 6);

    // Same color as journal.h's JSEV_MAINT — maintenance stays orange everywhere.
    lv_style_init(&sty_btn_orange);
    lv_style_set_bg_color(&sty_btn_orange, COL_ORANGE);
    lv_style_set_bg_opa(&sty_btn_orange, LV_OPA_COVER);
    lv_style_set_text_color(&sty_btn_orange, lv_color_hex(0x000000));
    lv_style_set_radius(&sty_btn_orange, 6);

    // Top bar
    lv_style_init(&sty_topbar);
    lv_style_set_bg_color(&sty_topbar, lv_color_hex(0x010409));
    lv_style_set_bg_opa(&sty_topbar, LV_OPA_COVER);
    lv_style_set_border_width(&sty_topbar, 0);
    lv_style_set_radius(&sty_topbar, 0);
    lv_style_set_pad_all(&sty_topbar, 0);
}

static lv_obj_t *make_card(lv_obj_t *parent, int x, int y, int w, int h) {
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_size(obj, w, h);
    lv_obj_add_style(obj, &sty_card, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    return obj;
}

// Value card with a compact ring gauge (lv_arc) in place of the plain
// bottom-mid value label — footprint (x,y,w,h) is unchanged from the
// label-only version, only the card's own content changes.
static lv_obj_t *make_val_card(lv_obj_t *parent, int x, int y, int w, int h,
                                const char *title, int rng_min, int rng_max,
                                lv_obj_t **out_label, lv_obj_t **out_arc) {
    lv_obj_t *card = make_card(parent, x, y, w, h);

    lv_obj_t *ttl = lv_label_create(card);
    lv_label_set_text(ttl, title);
    lv_obj_set_style_text_color(ttl, COL_TXT2, 0);
    lv_obj_set_style_text_font(ttl, &lv_font_montserrat_12, 0);
    lv_obj_align(ttl, LV_ALIGN_TOP_LEFT, 0, 0);

    // Ring gauge fills the space below the title, sized to the card's
    // available height so it stays square regardless of card width.
    const int arc_d = h - 12 - 14;   // card height minus pad/border minus title clearance
    lv_obj_t *arc = lv_arc_create(card);
    lv_obj_set_size(arc, arc_d, arc_d);
    lv_obj_align(arc, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_arc_set_bg_angles(arc, 135, 45);   // 270° sweep, gap at the bottom
    lv_arc_set_rotation(arc, 0);
    lv_arc_set_range(arc, rng_min, rng_max);
    lv_arc_set_value(arc, rng_min);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_color(arc, COL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 6, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, COL_BLUE, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 6, LV_PART_INDICATOR);

    lv_obj_t *lbl = lv_label_create(arc);
    lv_label_set_text(lbl, "--");
    lv_obj_set_style_text_color(lbl, COL_TXT, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    lv_obj_center(lbl);

    *out_label = lbl;
    if (out_arc) *out_arc = arc;
    return card;
}

// ---------------------------------------------------------------------------
// Keyboard popup
// ---------------------------------------------------------------------------
// Mirrors the textarea's current text + cursor position into the preview label,
// since the on-screen keyboard covers the field being edited (a plain '|' marks
// the cursor there, since the label itself has no caret rendering). Password
// fields are echoed as asterisks to preserve the masking the field itself
// already applies. Polled by a timer rather than hooked to LV_EVENT_VALUE_CHANGED
// because left/right arrow presses move the cursor without changing the text
// value, so no such event fires for them.
static void kb_preview_refresh(lv_obj_t *ta) {
    if (!kb_preview_label) return;
    const char *txt = lv_textarea_get_text(ta);
    size_t len = strlen(txt);
    uint32_t cur = lv_textarea_get_cursor_pos(ta);
    if (cur > len) cur = (uint32_t)len;
    bool pass = lv_textarea_get_password_mode(ta);

    char buf[80];
    size_t bi = 0;
    for (size_t i = 0; i <= len; i++) {
        if (i == cur && bi < sizeof(buf) - 1) buf[bi++] = '|';
        if (i == len || bi >= sizeof(buf) - 1) break;
        buf[bi++] = pass ? '*' : txt[i];
    }
    buf[bi] = 0;
    lv_label_set_text(kb_preview_label, buf);
}

static void kb_preview_timer_cb(lv_timer_t *t) {
    (void)t;
    if (kb_ta_active) kb_preview_refresh(kb_ta_active);
}

static void kb_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *kb = (lv_obj_t *)lv_event_get_target(e);
    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        if (kb_preview_timer) {
            lv_timer_del(kb_preview_timer);
            kb_preview_timer = nullptr;
        }
        lv_obj_del(kb_popup);
        kb_popup = nullptr;
        kb_ta_active = nullptr;
        kb_preview_label = nullptr;
    }
}

static void show_keyboard(lv_obj_t *ta) {
    if (kb_popup) lv_obj_del(kb_popup);

    kb_ta_active = ta;

    // Semi-transparent overlay
    kb_popup = lv_obj_create(lv_scr_act());
    lv_obj_set_size(kb_popup, UI_W, UI_H);
    lv_obj_align(kb_popup, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_bg_color(kb_popup, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(kb_popup, LV_OPA_70, 0);
    lv_obj_clear_flag(kb_popup, LV_OBJ_FLAG_SCROLLABLE);

    // Preview bar: the keyboard below covers the real field, so echo what's
    // being typed here instead.
    lv_obj_t *preview = lv_obj_create(kb_popup);
    lv_obj_set_size(preview, UI_W - 8, 32);
    lv_obj_align(preview, LV_ALIGN_TOP_MID, 0, 4);
    lv_obj_set_style_bg_color(preview, COL_CARD, 0);
    lv_obj_set_style_bg_opa(preview, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(preview, COL_ACCENT, 0);
    lv_obj_set_style_border_width(preview, 1, 0);
    lv_obj_set_style_radius(preview, 4, 0);
    lv_obj_set_style_pad_all(preview, 4, 0);
    lv_obj_clear_flag(preview, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *ttl = lv_label_create(preview);
    const char *field_title = (const char *)lv_obj_get_user_data(ta);
    if (!field_title || !field_title[0]) field_title = lv_textarea_get_placeholder_text(ta);
    lv_label_set_text_fmt(ttl, "%s:", (field_title && field_title[0]) ? field_title : "value");
    lv_obj_set_style_text_color(ttl, COL_TXT2, 0);
    lv_obj_set_style_text_font(ttl, &lv_font_montserrat_14, 0);
    lv_obj_align(ttl, LV_ALIGN_LEFT_MID, 4, 0);

    kb_preview_label = lv_label_create(preview);
    lv_obj_set_style_text_color(kb_preview_label, COL_TXT, 0);
    lv_obj_set_style_text_font(kb_preview_label, &lv_font_montserrat_14, 0);
    lv_label_set_long_mode(kb_preview_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(kb_preview_label, UI_W - 140);
    lv_obj_align(kb_preview_label, LV_ALIGN_RIGHT_MID, -4, 0);
    kb_preview_refresh(ta);
    kb_preview_timer = lv_timer_create(kb_preview_timer_cb, 150, nullptr);

    lv_obj_t *kb = lv_keyboard_create(kb_popup);
    lv_keyboard_set_textarea(kb, ta);
    lv_obj_set_size(kb, UI_W, 180);
    lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_add_event_cb(kb, kb_event_cb, LV_EVENT_ALL, nullptr);

    // Dark theme
    lv_obj_set_style_bg_color(kb, COL_CARD,   LV_PART_MAIN);
    lv_obj_set_style_bg_opa(kb,  LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(kb, COL_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(kb, 1,          LV_PART_MAIN);
    lv_obj_set_style_radius(kb, 0,                LV_PART_MAIN);
    lv_obj_set_style_pad_all(kb, 4,               LV_PART_MAIN);
    lv_obj_set_style_pad_gap(kb, 3,               LV_PART_MAIN);

    lv_obj_set_style_bg_color(kb,    lv_color_hex(0x21262d), LV_PART_ITEMS);
    lv_obj_set_style_bg_opa(kb,      LV_OPA_COVER,           LV_PART_ITEMS);
    lv_obj_set_style_text_color(kb,  COL_TXT,                LV_PART_ITEMS);
    lv_obj_set_style_text_font(kb,   &lv_font_montserrat_14, LV_PART_ITEMS);
    lv_obj_set_style_border_color(kb, COL_BORDER,            LV_PART_ITEMS);
    lv_obj_set_style_border_width(kb, 1,                     LV_PART_ITEMS);
    lv_obj_set_style_radius(kb,      4,                      LV_PART_ITEMS);
    lv_obj_set_style_shadow_width(kb, 0,                     LV_PART_ITEMS);

    lv_obj_set_style_bg_color(kb,   COL_ACCENT,              LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_text_color(kb, lv_color_hex(0xffffff),  LV_PART_ITEMS | LV_STATE_PRESSED);

    lv_obj_set_style_bg_color(kb,   COL_ACCENT,              LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(kb, lv_color_hex(0xffffff),  LV_PART_ITEMS | LV_STATE_CHECKED);
}

static void ta_click_cb(lv_event_t *e) {
    lv_obj_t *ta = (lv_obj_t *)lv_event_get_target(e);
    show_keyboard(ta);
}

// ---------------------------------------------------------------------------
// Top bar
// ---------------------------------------------------------------------------
static void build_topbar(lv_obj_t *screen) {
    lv_obj_t *bar = lv_obj_create(screen);
    lv_obj_set_size(bar, UI_W, TOPBAR_H);
    lv_obj_align(bar, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_add_style(bar, &sty_topbar, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

    // Signal-strength bars only — a raw dBm number takes a beat to parse
    // into "is this good or bad"; four ascending bars, lit left-to-right by
    // strength, read at a glance like a phone's signal icon (added
    // 2026-08-25, replacing the dBm figure here — RSSI in dBm is still
    // available on the Status tab for anyone who wants the number). The
    // "WiFi" text label that used to sit to the left of these was dropped
    // the same day — the bars alone are self-explanatory in that top-bar
    // position, and it's the first thing on the left edge now.
    static const int wifi_bar_h[4] = {5, 9, 13, 17};
    int wbx = 6;
    for (int i = 0; i < 4; i++) {
        wifi_bar[i] = lv_obj_create(bar);
        lv_obj_clear_flag(wifi_bar[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_border_width(wifi_bar[i], 0, 0);
        lv_obj_set_style_radius(wifi_bar[i], 1, 0);
        lv_obj_set_style_bg_opa(wifi_bar[i], LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(wifi_bar[i], lv_color_hex(0x3a3a3a), 0);
        lv_obj_set_size(wifi_bar[i], 5, wifi_bar_h[i]);
        // Bottom-align bars of different heights: shift each down from the
        // shared vertical-center anchor by half the height difference from
        // the tallest bar, so all four share one bottom edge.
        lv_obj_align(wifi_bar[i], LV_ALIGN_LEFT_MID, wbx + i * 7,
                     (wifi_bar_h[3] - wifi_bar_h[i]) / 2);
    }

    // WS label
    lbl_ws = lv_label_create(bar);
    lv_label_set_text(lbl_ws, "GEN: --");
    lv_obj_set_style_text_color(lbl_ws, COL_TXT2, 0);
    lv_obj_set_style_text_font(lbl_ws, &lv_font_montserrat_12, 0);
    lv_obj_align(lbl_ws, LV_ALIGN_LEFT_MID, 110, 0);

    // State label
    lbl_state = lv_label_create(bar);
    lv_label_set_text(lbl_state, "GEN PANEL");
    lv_obj_set_style_text_color(lbl_state, COL_ACCENT, 0);
    lv_obj_set_style_text_font(lbl_state, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl_state, LV_ALIGN_RIGHT_MID, -6, 0);

    // Own-panel battery. Pinned to the right edge when a cell is connected;
    // the state label then sits just to its left. Hidden otherwise, and the
    // state label takes the edge. Generator "Bat Volt" on the Power tab is
    // a different battery.
    lbl_panel_bat = lv_label_create(bar);
    lv_label_set_text(lbl_panel_bat, LV_SYMBOL_BATTERY_EMPTY);
    lv_obj_set_style_text_color(lbl_panel_bat, COL_TXT2, 0);
    lv_obj_set_style_text_font(lbl_panel_bat, &lv_font_montserrat_14, 0);
    lv_obj_add_flag(lbl_panel_bat, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(lbl_panel_bat, LV_ALIGN_RIGHT_MID, -4, 0);
}

// ---------------------------------------------------------------------------
// Chart helper
// ---------------------------------------------------------------------------
static lv_chart_series_t *make_chart_cont(lv_obj_t *parent,
    int x, int y, int w, int h,
    const char *title, lv_color_t color,
    int y_min, int y_max,
    lv_obj_t **out_chart,
    lv_obj_t **out_val_lbl = nullptr)
{
    // 32px left gutter for Y-axis tick labels
    const int AXIS_W = 32;

    lv_obj_t *cont = make_card(parent, x, y, w, h);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_left(cont, 6 + AXIS_W, 0);

    lv_obj_t *lbl = lv_label_create(cont);
    lv_label_set_text(lbl, title);
    lv_obj_set_style_text_color(lbl, COL_TXT2, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
    lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_t *val_lbl = lv_label_create(cont);
    lv_label_set_text(val_lbl, "--");
    lv_obj_set_style_text_color(val_lbl, color, 0);
    lv_obj_set_style_text_font(val_lbl, &lv_font_montserrat_10, 0);
    lv_obj_align(val_lbl, LV_ALIGN_TOP_RIGHT, -4, 0);
    if (out_val_lbl) *out_val_lbl = val_lbl;

    // available chart width = w - left(6+AXIS_W) - right(6) = w - 12 - AXIS_W
    // title 12px + 2px gap → chart uses remaining height
    int cw = w - 12 - AXIS_W;
    int ch = h - 12 - 14;

    lv_obj_t *chart = lv_chart_create(cont);
    lv_obj_set_size(chart, cw, ch);
    lv_obj_align(chart, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, CHART_POINTS);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, y_min, y_max);
    lv_chart_set_div_line_count(chart, 3, 5);

    // Y-axis tick marks with value labels
    lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_Y,
                           5, 3,        // major/minor tick length
                           3, 1,        // 3 major (min, mid, max), 1 minor between
                           true,        // labels enabled
                           AXIS_W);     // draw_size

    lv_obj_set_style_bg_color(chart, COL_BG, 0);
    lv_obj_set_style_bg_opa(chart, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(chart, COL_BORDER, 0);
    lv_obj_set_style_border_width(chart, 1, 0);
    lv_obj_set_style_line_color(chart, lv_color_hex(0x21262d), LV_PART_MAIN);
    lv_obj_set_style_line_width(chart, 2, LV_PART_ITEMS);
    lv_obj_set_style_text_color(chart, COL_TXT2, LV_PART_TICKS);
    lv_obj_set_style_text_font(chart, &lv_font_montserrat_8, LV_PART_TICKS);
    // Hide data-point dots
    lv_obj_set_style_width(chart,  0, LV_PART_INDICATOR);
    lv_obj_set_style_height(chart, 0, LV_PART_INDICATOR);

    lv_chart_series_t *ser = lv_chart_add_series(chart, color,
                                                  LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_all_value(chart, ser, LV_CHART_POINT_NONE);

    if (out_chart) *out_chart = chart;
    return ser;
}

// ---------------------------------------------------------------------------
// Dashboard tab
// ---------------------------------------------------------------------------
static void build_dashboard(lv_obj_t *tab) {
    lv_obj_set_style_bg_color(tab, COL_BG, 0);
    lv_obj_set_style_bg_opa(tab, LV_OPA_COVER, 0);
    // Keep scrolling enabled so charts are reachable below
    lv_obj_set_style_pad_all(tab, 0, 0);
    // Hide scrollbar
    lv_obj_set_scrollbar_mode(tab, LV_SCROLLBAR_MODE_OFF);

    // Big state label in the top area
    lbl_gen_state = lv_label_create(tab);
    lv_label_set_text(lbl_gen_state, "WAITING");
    lv_obj_set_style_text_color(lbl_gen_state, COL_TXT2, 0);
    lv_obj_set_style_text_font(lbl_gen_state, &lv_font_montserrat_36, 0);
    lv_obj_align(lbl_gen_state, LV_ALIGN_TOP_MID, 0, 4);

    // Row 1 — RPM | BAT | VOLT
    // 3 cards × cw + 2 × gap + x0 = 156*3 + 2*4 + 2 = 478 px (fills 480px screen)
    // Ranges mirror the chart y-min/y-max used further down for the same
    // metrics (kept in sync manually — see the chart section below).
    // BAT/VOLT/HEAD TEMP are scaled ×10 in ui_update() for 0.1 resolution.
    // BAT's 9.0-14.5V range mirrors data/dashboard.json's ADC1_1 gauge on
    // the web dashboard (a 12V lead-acid system).
    int row1_y = 50, cw = 156, ch = 96, gap = 4, x0 = 2;
    make_val_card(tab, x0,              row1_y, cw, ch, "RPM",   0,    4000, &lbl_rpm,   &arc_rpm);
    make_val_card(tab, x0 + cw + gap,   row1_y, cw, ch, "BAT",   90,   145,  &lbl_bat,   &arc_bat);
    make_val_card(tab, x0 + (cw+gap)*2, row1_y, cw, ch, "VOLT",  1500, 2600, &lbl_volt,  &arc_volt);

    // Row 2 — POWER | TEMP | FUEL
    int row2_y = row1_y + ch + gap;
    make_val_card(tab, x0,              row2_y, cw, ch, "POWER", 0,    4000, &lbl_power, &arc_power);
    make_val_card(tab, x0 + cw + gap,   row2_y, cw, ch, "TEMP",  0,    1500, &lbl_temp,   &arc_temp);
    make_val_card(tab, x0 + (cw+gap)*2, row2_y, cw, ch, "FUEL",  0,    90,   &lbl_fuel,   &arc_fuel);

    // -----------------------------------------------------------------------
    // Charts (scrollable area below cards) — one per row, full width
    // -----------------------------------------------------------------------
    const int cx0   = 2;
    const int cfw   = 476;   // full-width chart container
    const int cht_h = 180;   // container height
    const int cgap  = 4;
    int cy = 258;   // 12px gap after cards (cards end ~246)

    ser_rpm  = make_chart_cont(tab, cx0, cy, cfw, cht_h,
                "RPM", lv_color_hex(0x388bfd), 0, 4000, &chart_rpm, &cv_rpm);
    cy += cht_h + cgap;

    ser_volt = make_chart_cont(tab, cx0, cy, cfw, cht_h,
                "Voltage (V)", lv_color_hex(0xd29922), 150, 260, &chart_volt, &cv_volt);
    cy += cht_h + cgap;

    ser_curr = make_chart_cont(tab, cx0, cy, cfw, cht_h,
                "Current (A)", lv_color_hex(0xe36209), 0, 20, &chart_curr, &cv_curr);
    cy += cht_h + cgap;

    ser_pwr  = make_chart_cont(tab, cx0, cy, cfw, cht_h,
                "Power (W)", lv_color_hex(0xf85149), 0, 4000, &chart_pwr, &cv_pwr);
    cy += cht_h + cgap;

    ser_head = make_chart_cont(tab, cx0, cy, cfw, cht_h,
                "Head Temp (C)", lv_color_hex(0xff7b72), 0, 150, &chart_head, &cv_head);
    cy += cht_h + cgap;

    ser_amb  = make_chart_cont(tab, cx0, cy, cfw, cht_h,
                "Ambient (C)", lv_color_hex(0x79c0ff), -20, 60, &chart_amb, &cv_amb);
    cy += cht_h + cgap;

    ser_oil  = make_chart_cont(tab, cx0, cy, cfw, cht_h,
                "Oil Temp (C)", lv_color_hex(0xffa657), 0, 150, &chart_oil, &cv_oil);
    cy += cht_h + cgap;

    ser_inv  = make_chart_cont(tab, cx0, cy, cfw, cht_h,
                "Inverter (C)", lv_color_hex(0xd2a8ff), 0, 100, &chart_inv, &cv_inv);
    cy += cht_h + cgap;

    ser_alt  = make_chart_cont(tab, cx0, cy, cfw, cht_h,
                "Alternator (C)", lv_color_hex(0x3fb950), 0, 150, &chart_alt, &cv_alt);
}

// ---------------------------------------------------------------------------
// Controls tab
// ---------------------------------------------------------------------------
static void btn_start_cb(lv_event_t *) { ws_send("start"); }
static void btn_retry_cb(lv_event_t *) { ws_send("start_retry"); }
static void btn_stop_cb(lv_event_t *)  { ws_send("stop"); }

static void sw_toggle_cb(lv_event_t *e) {
    uint8_t reg = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    ws_send_toggle(reg);
}

static lv_obj_t *make_switch_row(lv_obj_t *parent, int x, int y,
                                  const char *label, uint8_t reg) {
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_pos(cont, x, y);
    lv_obj_set_size(cont, 140, 32);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(cont, 0, 0);

    lv_obj_t *lbl = lv_label_create(cont);
    lv_label_set_text(lbl, label);
    lv_obj_set_style_text_color(lbl, COL_TXT, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *sw = lv_switch_create(cont);
    lv_obj_align(sw, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(sw, sw_toggle_cb, LV_EVENT_VALUE_CHANGED,
                        (void *)(uintptr_t)reg);
    return sw;
}

static lv_obj_t *make_indicator(lv_obj_t *parent, int x, int y, const char *label) {
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_pos(cont, x, y);
    lv_obj_set_size(cont, 70, 36);
    lv_obj_add_style(cont, &sty_card, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *dot = lv_obj_create(cont);
    lv_obj_set_size(dot, 10, 10);
    lv_obj_set_style_radius(dot, 5, 0);
    lv_obj_set_style_bg_color(dot, COL_TXT2, 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(dot, 0, 0);
    lv_obj_align(dot, LV_ALIGN_TOP_LEFT, 2, 2);

    lv_obj_t *lbl = lv_label_create(cont);
    lv_label_set_text(lbl, label);
    lv_obj_set_style_text_color(lbl, COL_TXT2, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
    lv_obj_align(lbl, LV_ALIGN_BOTTOM_MID, 0, 0);

    // Return the dot so caller can color it
    return dot;
}

// ---------------------------------------------------------------------------
// Overdue-maintenance bar (Controls tab) — Record button
// ---------------------------------------------------------------------------

// Fires when the user taps a type (or Cancel) in the picker popup.
static void maint_mbox_event_cb(lv_event_t *e) {
    lv_obj_t *mbox = lv_event_get_current_target(e);
    const char *txt = lv_msgbox_get_active_btn_text(mbox);  // NULL if the "x" close was tapped
    if (txt) {
        for (uint8_t i = 0; i < MAINT_TYPES; i++) {
            if (strcmp(txt, MAINT_TYPE_LABELS[i]) == 0) {
                journal_request_maintenance(i);
                break;
            }
        }
        // No match ⇒ "Cancel" was tapped — nothing to do.
    }
    lv_msgbox_close(mbox);
    // maint_mbox itself is cleared by the LV_EVENT_DELETE handler below, not
    // here — this callback only runs for a matrix-button tap (Cancel or a
    // type), never for the built-in "x" close button (that one calls
    // lv_msgbox_close() directly inside LVGL, bypassing this VALUE_CHANGED
    // handler entirely). Clearing it only here left `maint_mbox` stuck
    // non-null forever after closing via "x", silently blocking every
    // future Record tap (the guard in maint_record_btn_cb()) — found live
    // 2026-08-25. LV_EVENT_DELETE fires for every close path alike, so hook
    // the object's actual lifecycle instead of guessing which button closed it.
}

// Runs for ANY reason maint_mbox gets deleted (a matrix button tap that
// calls lv_msgbox_close() above, or the built-in "x" close button, which
// deletes the msgbox directly without going through maint_mbox_event_cb at
// all) — the one place that's guaranteed to run in every case.
static void maint_mbox_delete_cb(lv_event_t *) {
    maint_mbox = nullptr;
}

// Tapping Record opens a picker limited to whichever types are actually
// overdue right now (s_maint_mask, refreshed by ui_update() from the WS
// report) — never all six, since ticking off something not due yet is
// just noise on the FRAM journal. Each type gets its own row ("\n" between
// entries) instead of packing them into one row of the button matrix —
// with up to 6 multi-word labels plus "Cancel", one row overlapped into an
// unreadable mess on real hardware (found live 2026-08-25).
static void maint_record_btn_cb(lv_event_t *) {
    if (maint_mbox || s_maint_mask == 0) return;
    // Up to 6 types, each preceded by a "\n" row-break except the first,
    // plus a "\n" + "Cancel" + "" terminator.
    static const char *btn_txts[MAINT_TYPES * 2 + 2];
    int n = 0, rows = 1;  // +1 for the Cancel row
    for (uint8_t i = 0; i < MAINT_TYPES; i++) {
        if (!(s_maint_mask & (1u << i))) continue;
        if (n > 0) btn_txts[n++] = "\n";
        btn_txts[n++] = MAINT_TYPE_LABELS[i];
        rows++;
    }
    btn_txts[n++] = "\n";
    btn_txts[n++] = "Cancel";
    btn_txts[n] = "";
    maint_mbox = lv_msgbox_create(nullptr, "Record maintenance",
                                   "Which item was completed?", btn_txts, true);
    lv_obj_set_width(maint_mbox, 300);
    // lv_msgbox_create() sizes its button matrix for exactly ONE row
    // (line_height + LV_DPI_DEF/10), with no awareness that "\n" in the map
    // produced more rows — every row after the first had nowhere to go but
    // to overlap the one above it, rendering as the garbled stacked text
    // seen live 2026-08-25. Resize the matrix to fit the real row count
    // using the same per-row height LVGL's own code computes internally.
    lv_obj_t *btnm = lv_msgbox_get_btns(maint_mbox);
    const lv_font_t *btn_font = lv_obj_get_style_text_font(btnm, LV_PART_ITEMS);
    lv_coord_t row_h = lv_font_get_line_height(btn_font) + LV_DPI_DEF / 10;
    lv_obj_set_height(btnm, row_h * rows);
    lv_obj_center(maint_mbox);
    lv_obj_add_event_cb(maint_mbox, maint_mbox_event_cb, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_add_event_cb(maint_mbox, maint_mbox_delete_cb, LV_EVENT_DELETE, nullptr);
}

static void build_controls(lv_obj_t *tab) {
    lv_obj_set_style_bg_color(tab, COL_BG, 0);
    lv_obj_set_style_bg_opa(tab, LV_OPA_COVER, 0);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);

    // --- Action buttons ---
    int bw = 140, bh = 40, gap = 5, bx0 = 5, by0 = 6;

    btn_start = lv_btn_create(tab);
    lv_obj_set_pos(btn_start, bx0, by0);
    lv_obj_set_size(btn_start, bw, bh);
    lv_obj_add_style(btn_start, &sty_btn_green, 0);
    lv_obj_add_event_cb(btn_start, btn_start_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *l = lv_label_create(btn_start);
    lv_label_set_text(l, "START");
    lv_obj_center(l);

    btn_retry = lv_btn_create(tab);
    lv_obj_set_pos(btn_retry, bx0 + bw + gap, by0);
    lv_obj_set_size(btn_retry, bw, bh);
    lv_obj_add_style(btn_retry, &sty_btn_blue, 0);
    lv_obj_add_event_cb(btn_retry, btn_retry_cb, LV_EVENT_CLICKED, nullptr);
    l = lv_label_create(btn_retry);
    lv_label_set_text(l, "START RETRY");
    lv_obj_center(l);

    btn_stop_ctrl = lv_btn_create(tab);
    lv_obj_set_pos(btn_stop_ctrl, bx0 + (bw + gap) * 2, by0);
    lv_obj_set_size(btn_stop_ctrl, bw, bh);
    lv_obj_add_style(btn_stop_ctrl, &sty_btn_red, 0);
    lv_obj_add_event_cb(btn_stop_ctrl, btn_stop_cb, LV_EVENT_CLICKED, nullptr);
    l = lv_label_create(btn_stop_ctrl);
    lv_label_set_text(l, "STOP");
    lv_obj_center(l);

    // --- Switches ---
    int sy0 = by0 + bh + 8;
    sw_choke   = make_switch_row(tab,   5, sy0,      "Choke",    REG_CHOKE);
    sw_fuel    = make_switch_row(tab, 155, sy0,      "Fuel Pump",REG_FUEL_PUMP);
    sw_eco     = make_switch_row(tab, 305, sy0,      "ECO Mode", REG_ECO_MODE);
    sy0 += 40;
    sw_load    = make_switch_row(tab,   5, sy0,      "Load",     REG_LOAD_RELAY);
    sw_charger = make_switch_row(tab, 155, sy0,      "Charger",  REG_CHARGER);

    // All controls start disabled until WS connects
    lv_obj_add_state(btn_start,    LV_STATE_DISABLED);
    lv_obj_add_state(btn_retry,    LV_STATE_DISABLED);
    lv_obj_add_state(btn_stop_ctrl,LV_STATE_DISABLED);
    lv_obj_add_state(sw_choke,     LV_STATE_DISABLED);
    lv_obj_add_state(sw_fuel,      LV_STATE_DISABLED);
    lv_obj_add_state(sw_eco,       LV_STATE_DISABLED);
    lv_obj_add_state(sw_load,      LV_STATE_DISABLED);
    lv_obj_add_state(sw_charger,   LV_STATE_DISABLED);

    // --- Indicators: OVLD, OIL, OVHEAT, OUT, GEN, BAT ---
    int iy0 = sy0 + 44;
    int ix = 4, istep = 74;
    ind_overload = make_indicator(tab, ix,              iy0, "OVLD");
    ind_oil      = make_indicator(tab, ix + istep,      iy0, "OIL");
    ind_overheat = make_indicator(tab, ix + istep * 2,  iy0, "OVHEAT");
    ind_output   = make_indicator(tab, ix + istep * 3,  iy0, "OUT");
    ind_gen      = make_indicator(tab, ix + istep * 4,  iy0, "GEN");
    ind_bat      = make_indicator(tab, ix + istep * 5,  iy0, "BAT");

    // --- Overdue-maintenance bar — hidden until ui_update() sees a nonzero mask ---
    // Height/gap tightened 2026-08-25: at the original 60px (+8px gap from the
    // indicator row), the bar's bottom edge (242) left only 10px of margin
    // against CONTENT_H (252) before running into the tabview's own bottom
    // tab-bar strip — on real hardware this read as the container "barely not
    // fitting" and the right-aligned Record button (near the bar's bottom-
    // right) not responding to taps at all, consistent with the tab bar
    // eating touches in that overlap. Shrunk to leave a real margin.
    int my0 = iy0 + 36 + 4;
    // Width trimmed 2026-08-25: at UI_W-8 (472px, x=4) the right edge sat
    // only 4px from the screen's own edge — reported live as the container
    // "barely not fitting" widthwise. UI_W-24 (x=8 each side) leaves 12px
    // of real margin on both edges instead.
    const int maint_bar_pad = 6, maint_bar_h = 52, maint_btn_w = 90, maint_btn_gap = 10;
    const int maint_bar_w = UI_W - 24;
    maint_bar = lv_obj_create(tab);
    lv_obj_set_pos(maint_bar, 8, my0);
    lv_obj_set_size(maint_bar, maint_bar_w, maint_bar_h);
    lv_obj_set_style_bg_color(maint_bar, lv_color_hex(0x3d2a0a), 0);
    lv_obj_set_style_bg_opa(maint_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(maint_bar, COL_ORANGE, 0);
    lv_obj_set_style_border_width(maint_bar, 1, 0);
    lv_obj_set_style_radius(maint_bar, 6, 0);
    lv_obj_set_style_pad_all(maint_bar, maint_bar_pad, 0);
    lv_obj_clear_flag(maint_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(maint_bar, LV_OBJ_FLAG_HIDDEN);

    maint_bar_lbl = lv_label_create(maint_bar);
    lv_label_set_text(maint_bar_lbl, "");
    lv_obj_set_style_text_color(maint_bar_lbl, COL_ORANGE, 0);
    lv_obj_set_style_text_font(maint_bar_lbl, &lv_font_montserrat_14, 0);
    lv_label_set_long_mode(maint_bar_lbl, LV_LABEL_LONG_WRAP);
    // Width computed from the container's own geometry (content width minus
    // the button + a gap), not a hardcoded guess — a fixed 340px here used
    // to run right up against the Record button once four-plus type names
    // were listed ("Oil change, Spark plug, Air filter, Fuel filter"),
    // reported live as the bar "not fitting width-wise" 2026-08-25.
    lv_obj_set_width(maint_bar_lbl, maint_bar_w - 2 * maint_bar_pad - maint_btn_w - maint_btn_gap);
    lv_obj_align(maint_bar_lbl, LV_ALIGN_LEFT_MID, 0, 0);

    maint_bar_btn = lv_btn_create(maint_bar);
    lv_obj_set_size(maint_bar_btn, maint_btn_w, 36);
    lv_obj_align(maint_bar_btn, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_style(maint_bar_btn, &sty_btn_orange, 0);
    lv_obj_add_event_cb(maint_bar_btn, maint_record_btn_cb, LV_EVENT_CLICKED, nullptr);
    l = lv_label_create(maint_bar_btn);
    lv_label_set_text(l, "Record");
    lv_obj_center(l);
}

// ---------------------------------------------------------------------------
// Power tab
// ---------------------------------------------------------------------------
static lv_obj_t *make_prow(lv_obj_t *parent, int x, int y, const char *label,
                             lv_obj_t **out) {
    lv_obj_t *lbl_t = lv_label_create(parent);
    lv_label_set_text(lbl_t, label);
    lv_obj_set_style_text_color(lbl_t, COL_TXT2, 0);
    lv_obj_set_style_text_font(lbl_t, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(lbl_t, x, y);

    lv_obj_t *lbl_v = lv_label_create(parent);
    lv_label_set_text(lbl_v, "--");
    lv_obj_set_style_text_color(lbl_v, COL_TXT, 0);
    lv_obj_set_style_text_font(lbl_v, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(lbl_v, x + 110, y);

    *out = lbl_v;
    return lbl_v;
}

static void build_power(lv_obj_t *tab) {
    lv_obj_set_style_bg_color(tab, COL_BG, 0);
    lv_obj_set_style_bg_opa(tab, LV_OPA_COVER, 0);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);

    int col1x = 8, col2x = 248, row_h = 26, y0 = 4;

    make_prow(tab, col1x, y0,            "Voltage (V):",    &lbl_vrms);
    make_prow(tab, col1x, y0 + row_h,    "Current (A):",   &lbl_irms);
    make_prow(tab, col1x, y0 + row_h*2,  "Power (W):",     &lbl_pw);
    make_prow(tab, col1x, y0 + row_h*3,  "Apparent (VA):", &lbl_pa);
    make_prow(tab, col1x, y0 + row_h*4,  "Reactive(VAR):", &lbl_pr);
    make_prow(tab, col1x, y0 + row_h*5,  "Power Factor:",  &lbl_pf);
    make_prow(tab, col1x, y0 + row_h*6,  "Bat Volt (V):",  &lbl_bat_v);
    make_prow(tab, col1x, y0 + row_h*7,  "Core Volt (V):", &lbl_core_v);

    make_prow(tab, col2x, y0,            "Frequency:",      &lbl_hz);
    make_prow(tab, col2x, y0 + row_h,    "Head Temp (C):",  &lbl_htemp);
    make_prow(tab, col2x, y0 + row_h*2,  "Fuel (L):",       &lbl_fuel2);
    make_prow(tab, col2x, y0 + row_h*3,  "Oil Temp (C):",   &lbl_oil_t);
    make_prow(tab, col2x, y0 + row_h*4,  "Ambient (C):",    &lbl_amb_t);
    make_prow(tab, col2x, y0 + row_h*5,  "Inverter (C):",   &lbl_inv_t);
    make_prow(tab, col2x, y0 + row_h*6,  "Alternator(C):",  &lbl_alt_t);
    make_prow(tab, col2x, y0 + row_h*7,  "Uptime:",         &lbl_uptime);
    make_prow(tab, col2x, y0 + row_h*8,  "RSSI:",           &lbl_rssi);
}

// ---------------------------------------------------------------------------
// Journal tab
// ---------------------------------------------------------------------------
static void btn_j_refresh_cb(lv_event_t *) {
    ui_journal_set_status("loading...");
    journal_request_refresh();
}

static void build_journal(lv_obj_t *tab) {
    lv_obj_set_style_bg_color(tab, COL_BG, 0);
    lv_obj_set_style_bg_opa(tab, LV_OPA_COVER, 0);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(tab, 0, 0);

    // Header: metrics summary + status line on the left, Refresh on the right
    lbl_j_metrics = lv_label_create(tab);
    lv_label_set_text(lbl_j_metrics, "--");
    lv_label_set_long_mode(lbl_j_metrics, LV_LABEL_LONG_DOT);
    lv_obj_set_size(lbl_j_metrics, 380, 16);
    lv_obj_set_pos(lbl_j_metrics, 6, 4);
    lv_obj_set_style_text_color(lbl_j_metrics, COL_TXT, 0);
    lv_obj_set_style_text_font(lbl_j_metrics, &lv_font_montserrat_12, 0);

    lbl_j_status = lv_label_create(tab);
    lv_label_set_text(lbl_j_status, "open tab or tap Refresh to load");
    lv_label_set_long_mode(lbl_j_status, LV_LABEL_LONG_DOT);
    lv_obj_set_size(lbl_j_status, 380, 14);
    lv_obj_set_pos(lbl_j_status, 6, 22);
    lv_obj_set_style_text_color(lbl_j_status, COL_TXT2, 0);
    lv_obj_set_style_text_font(lbl_j_status, &lv_font_montserrat_10, 0);

    lv_obj_t *btn = lv_btn_create(tab);
    lv_obj_set_pos(btn, UI_W - 88, 4);
    lv_obj_set_size(btn, 84, 30);
    lv_obj_add_style(btn, &sty_btn_blue, 0);
    lv_obj_add_event_cb(btn, btn_j_refresh_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *bl = lv_label_create(btn);
    lv_label_set_text(bl, "Refresh");
    lv_obj_set_style_text_font(bl, &lv_font_montserrat_12, 0);
    lv_obj_center(bl);

    // Scrollable event list
    list_journal = lv_obj_create(tab);
    lv_obj_set_pos(list_journal, 2, 40);
    lv_obj_set_size(list_journal, UI_W - 4, CONTENT_H - 42);
    lv_obj_add_style(list_journal, &sty_card, 0);
    lv_obj_set_style_pad_all(list_journal, 4, 0);
    lv_obj_set_flex_flow(list_journal, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(list_journal, 3, 0);
    lv_obj_set_scrollbar_mode(list_journal, LV_SCROLLBAR_MODE_AUTO);
}

static lv_color_t journal_sev_color(uint8_t sev) {
    switch (sev) {
        case JSEV_START:  return COL_GREEN;
        case JSEV_STOP:   return COL_TXT;
        case JSEV_ALARM:  return COL_RED;
        case JSEV_CHARGE: return COL_BLUE;
        case JSEV_MAINT:  return COL_ORANGE;
        default:          return COL_TXT2;   // JSEV_SYS
    }
}

void ui_journal_show(const JournalEntry *entries, int count,
                     const char *metrics_line) {
    if (metrics_line && metrics_line[0])
        lv_label_set_text(lbl_j_metrics, metrics_line);
    lv_label_set_text(lbl_j_status,
                      count ? "" : "journal is empty");

    lv_obj_clean(list_journal);
    char buf[128];
    for (int i = 0; i < count; i++) {
        const JournalEntry *e = &entries[i];
        snprintf(buf, sizeof(buf), "%-4s %-8s %s", e->age, e->type, e->text);
        lv_obj_t *lbl = lv_label_create(list_journal);
        lv_label_set_text(lbl, buf);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
        lv_obj_set_width(lbl, LV_PCT(100));
        lv_obj_set_style_text_color(lbl, journal_sev_color(e->sev), 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
    }
    lv_obj_scroll_to_y(list_journal, 0, LV_ANIM_OFF);
}

void ui_journal_set_status(const char *msg) {
    lv_label_set_text(lbl_j_status, msg);
}

// ---------------------------------------------------------------------------
// Config tab
// ---------------------------------------------------------------------------
static void net_list_cb(lv_event_t *e) {
    lv_obj_t *item = (lv_obj_t *)lv_event_get_target(e);
    const char *txt = lv_list_get_btn_text(list_networks, item);
    // Extract SSID (before any trailing space/rssi info) into the editable field
    char ssid[64];
    strncpy(ssid, txt, sizeof(ssid) - 1);
    ssid[sizeof(ssid)-1] = 0;
    char *sp = strchr(ssid, ' ');
    if (sp) *sp = 0;
    lv_textarea_set_text(ta_ssid, ssid);
}

static void btn_scan_cb(lv_event_t *) {
    WiFi.scanNetworks(true /*async*/);
    // Rides the same button: a full WiFi scan already visits every channel,
    // so any ESP-NOW beacon heard along the way gets its channel recorded
    // for free (esp_now_client.cpp's on_beacon()) — and this broadcasts a
    // one-shot LAN discovery request (udp_discovery.h) at the same time.
    udp_discovery_request();
    lv_obj_clean(list_networks);
    lv_obj_t *lbl = lv_label_create(list_networks);
    lv_label_set_text(lbl, "Scanning...");
    lv_obj_set_style_text_color(lbl, COL_TXT2, 0);
}

static void btn_cfg_connect_cb(lv_event_t *) {
    if (!connect_cb) return;

    const char *host = lv_textarea_get_text(ta_host);
    const char *pass = lv_textarea_get_text(ta_pass);
    const char *ssid = lv_textarea_get_text(ta_ssid);
    connect_cb(ssid, pass, host);
}

// Snapshot behind dd_gen's options: option 0 is "(none)", options
// 1..s_dd_count are s_dd_gens (ESP-NOW), the rest are s_dd_lan (LAN UDP
// discovery). Rebuilt only while the dropdown is closed.
#define DD_GEN_MAX 8
#define DD_LAN_MAX 8
static EspNowGenInfo s_dd_gens[DD_GEN_MAX];
static int           s_dd_count = 0;
static UdpGenInfo    s_dd_lan[DD_LAN_MAX];
static int           s_dd_lan_count = 0;

// A LAN pick has no radio join step — just hand the IP back to the caller
// (display.ino's loop(), via ui_take_lan_pick()) like a typed/learned host.
static char s_lan_pick_ip[16] = "";
static bool s_lan_pick_pending = false;

static void dd_gen_cb(lv_event_t *) {
    uint16_t idx = lv_dropdown_get_selected(dd_gen);
    if (idx == 0) {
        espnow_select_generator(nullptr, "");
    } else if (idx - 1 < (uint16_t)s_dd_count) {
        if (!s_dd_gens[idx - 1].selected)
            espnow_select_generator(s_dd_gens[idx - 1].mac, s_dd_gens[idx - 1].name);
    } else {
        int lan_idx = idx - 1 - s_dd_count;
        if (lan_idx < s_dd_lan_count) {
            strlcpy(s_lan_pick_ip, s_dd_lan[lan_idx].ip, sizeof(s_lan_pick_ip));
            s_lan_pick_pending = true;
        }
    }
}

static lv_obj_t *make_ta(lv_obj_t *parent, int x, int y, int w, int h,
                           const char *placeholder, const char *init,
                           const char *title = nullptr) {
    lv_obj_t *ta = lv_textarea_create(parent);
    lv_obj_set_pos(ta, x, y);
    lv_obj_set_size(ta, w, h);
    lv_textarea_set_one_line(ta, true);
    lv_textarea_set_placeholder_text(ta, placeholder);
    if (init && init[0]) lv_textarea_set_text(ta, init);
    lv_obj_set_style_bg_color(ta, COL_CARD, 0);
    lv_obj_set_style_text_color(ta, COL_TXT, 0);
    lv_obj_set_style_border_color(ta, COL_BORDER, 0);
    // Field's display name for the on-screen-keyboard preview bar — falls back to
    // the placeholder only if no title was given (placeholder can be a sample
    // value like "192.168.1.100" rather than a real label).
    lv_obj_set_user_data(ta, (void *)(title ? title : placeholder));
    lv_obj_add_event_cb(ta, ta_click_cb, LV_EVENT_CLICKED, nullptr);
    return ta;
}

static void build_config(lv_obj_t *tab, const char *init_ssid,
                          const char *init_pass, const char *init_host) {
    lv_obj_set_style_bg_color(tab, COL_BG, 0);
    lv_obj_set_style_bg_opa(tab, LV_OPA_COVER, 0);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);
    // Remove tab content padding so coordinates are exact
    lv_obj_set_style_pad_all(tab, 0, 0);

    // Available height inside tab content area
    const int avail_h = CONTENT_H - 4;   // 248
    const int btn_h   = 30;
    const int margin  = 4;               // gap between list and scan button

    // --- Left: WiFi list ---
    lv_obj_t *list_cont = make_card(tab, 2, 2, 210, avail_h);
    lv_obj_clear_flag(list_cont, LV_OBJ_FLAG_SCROLLABLE);
    // Remove card padding so we control positions manually
    lv_obj_set_style_pad_all(list_cont, 4, 0);

    lv_obj_t *lbl_h = lv_label_create(list_cont);
    lv_label_set_text(lbl_h, "WiFi Networks");
    lv_obj_set_style_text_color(lbl_h, COL_TXT, 0);
    lv_obj_set_style_text_font(lbl_h, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(lbl_h, 0, 0);

    // Scan button at bottom — placed before list so height is known
    lv_obj_t *btn_scan = lv_btn_create(list_cont);
    lv_obj_set_size(btn_scan, 100, btn_h);
    // Position: bottom of inner area (avail_h - 2*pad4 - btn_h = 248-8-30=210)
    lv_obj_set_pos(btn_scan, (202 - 100) / 2, avail_h - 2 * 4 - btn_h);
    lv_obj_add_style(btn_scan, &sty_btn_blue, 0);
    lv_obj_add_event_cb(btn_scan, btn_scan_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *sl = lv_label_create(btn_scan);
    lv_label_set_text(sl, "Scan");
    lv_obj_center(sl);

    // List fills space between header and button
    int list_top = 20;
    int list_h   = (avail_h - 2 * 4) - list_top - btn_h - margin;  // 248-8-20-30-4=186
    list_networks = lv_list_create(list_cont);
    lv_obj_set_pos(list_networks, 0, list_top);
    lv_obj_set_size(list_networks, 202, list_h);
    lv_obj_set_style_bg_color(list_networks, COL_BG, 0);
    lv_obj_set_style_border_width(list_networks, 0, 0);
    lv_obj_set_style_pad_all(list_networks, 2, 0);

    // --- Right: settings ---
    lv_obj_t *cfg_cont = make_card(tab, 216, 2, 260, avail_h);
    lv_obj_clear_flag(cfg_cont, LV_OBJ_FLAG_SCROLLABLE);

    auto mkl = [&](lv_obj_t *par, int x, int y, const char *txt) {
        lv_obj_t *l = lv_label_create(par);
        lv_label_set_text(l, txt);
        lv_obj_set_style_text_color(l, COL_TXT2, 0);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_12, 0);
        lv_obj_set_pos(l, x, y);
    };

    // card pad_all=6 → inner content height = avail_h - 12 = 236px
    // Layout budget: (14+32+2)*4 + 4 + 36 = 192+4+36 = 232 < 236 ✓
    const int rh   = 32;   // textarea height
    const int rgap = 2;    // gap between rows
    const int lh   = 14;   // label row height
    int ry = 0;

    // WiFi SSID — editable: tap a scanned network to fill it, or type manually
    mkl(cfg_cont, 0, ry, "WiFi SSID:");
    ta_ssid = make_ta(cfg_cont, 0, ry + lh, 248, rh, "network name",
                      (init_ssid && init_ssid[0]) ? init_ssid : "", "WiFi SSID");

    ry += lh + rh + rgap;
    mkl(cfg_cont, 0, ry, "WiFi Password:");
    ta_pass = make_ta(cfg_cont, 0, ry + lh, 248, rh, "password",
                      (init_pass && init_pass[0]) ? init_pass : "", "WiFi Password");
    lv_textarea_set_password_mode(ta_pass, true);

    ry += lh + rh + rgap;
    mkl(cfg_cont, 0, ry, "Generator Host:");
    ta_host = make_ta(cfg_cont, 0, ry + lh, 248, rh, "192.168.1.100", init_host, "Generator Host");

    // Picking a generator here also sets Generator Host: its IP arrives in
    // its encrypted ESP-NOW state (display.ino).
    ry += lh + rh + rgap;
    mkl(cfg_cont, 0, ry, "Generator (ESP-NOW):");
    dd_gen = lv_dropdown_create(cfg_cont);
    lv_obj_set_pos(dd_gen, 0, ry + lh);
    lv_obj_set_size(dd_gen, 248, rh);
    lv_dropdown_set_options(dd_gen, "(none)");
    lv_obj_set_style_bg_color(dd_gen, COL_CARD, 0);
    lv_obj_set_style_text_color(dd_gen, COL_TXT, 0);
    lv_obj_set_style_border_color(dd_gen, COL_BORDER, 0);
    lv_obj_add_event_cb(dd_gen, dd_gen_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    ry += lh + rh + rgap + 4;
    lv_obj_t *btn_conn = lv_btn_create(cfg_cont);
    lv_obj_set_pos(btn_conn, 0, ry);
    lv_obj_set_size(btn_conn, 248, 36);
    lv_obj_add_style(btn_conn, &sty_btn_green, 0);
    lv_obj_add_event_cb(btn_conn, btn_cfg_connect_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *cl = lv_label_create(btn_conn);
    lv_label_set_text(cl, "Save & Connect");
    lv_obj_center(cl);
}

// ---------------------------------------------------------------------------
// Indicator helper
// ---------------------------------------------------------------------------
static void set_indicator(lv_obj_t *dot, bool active, lv_color_t col) {
    lv_obj_set_style_bg_color(dot, active ? col : COL_TXT2, 0);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void ui_init(void) {
    style_init();

    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, COL_BG, 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    build_topbar(screen);

    // Tab view — tabs at bottom
    lv_obj_t *tv = lv_tabview_create(screen, LV_DIR_BOTTOM, TABBAR_H);
    lv_obj_set_pos(tv, 0, TOPBAR_H);
    lv_obj_set_size(tv, UI_W, UI_H - TOPBAR_H);
    lv_obj_set_style_bg_color(tv, COL_BG, 0);
    lv_obj_set_style_bg_opa(tv, LV_OPA_COVER, 0);

    // Tab button bar style
    lv_obj_t *tab_btns = lv_tabview_get_tab_btns(tv);
    lv_obj_set_style_bg_color(tab_btns, lv_color_hex(0x010409), 0);
    lv_obj_set_style_text_color(tab_btns, COL_TXT2, 0);
    lv_obj_set_style_text_color(tab_btns, COL_ACCENT, LV_STATE_CHECKED);
    lv_obj_set_style_border_side(tab_btns, LV_BORDER_SIDE_TOP, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_border_color(tab_btns, COL_ACCENT, LV_PART_ITEMS | LV_STATE_CHECKED);

    lv_obj_t *t0 = lv_tabview_add_tab(tv, "Dashboard");
    lv_obj_t *t1 = lv_tabview_add_tab(tv, "Controls");
    lv_obj_t *t2 = lv_tabview_add_tab(tv, "Status");
    lv_obj_t *t3 = lv_tabview_add_tab(tv, "Journal");
    lv_obj_t *t4 = lv_tabview_add_tab(tv, "Config");

    build_dashboard(t0);
    build_controls(t1);
    build_power(t2);
    build_journal(t3);
    build_config(t4, g_config.wifi_ssid, g_config.wifi_pass, g_config.gen_host);

    // Auto-load the journal when its tab is opened
    lv_obj_add_event_cb(tv, [](lv_event_t *e) {
        lv_obj_t *tabview = (lv_obj_t *)lv_event_get_target(e);
        if (lv_tabview_get_tab_act(tabview) == TAB_JOURNAL_IDX) {
            ui_journal_set_status("loading...");
            journal_request_refresh();
        }
    }, LV_EVENT_VALUE_CHANGED, nullptr);
}

// Map the controller's state_name() onto a top-bar / dashboard label.
// Without value.STATE (controller not yet updated) fall back to the old
// run/stop pair, which only knows idle vs everything else.
static void gen_state_label(const GenData &d, const char **text, lv_color_t *col) {
    static char cooldown_buf[20];
    const char *s = d.state;
    if (s[0] == '\0') {
        if (d.running)      { *text = "RUNNING";  *col = COL_GREEN; }
        else if (d.stopped) { *text = "STOPPED";  *col = COL_RED; }
        else                { *text = "STANDBY";  *col = COL_ORANGE; }
        return;
    }
    if (!strcmp(s, "running")) {
        *text = "RUNNING"; *col = COL_GREEN;
    } else if (!strcmp(s, "cooldown")) {
        if (d.cooldown_left) {
            snprintf(cooldown_buf, sizeof(cooldown_buf), "COOLING %us", (unsigned)d.cooldown_left);
            *text = cooldown_buf;
        } else {
            *text = "COOLING";
        }
        *col = COL_BLUE;
    } else if (!strcmp(s, "starting") || !strcmp(s, "starting_retry")) {
        *text = "STARTING"; *col = COL_ORANGE;
    } else if (!strcmp(s, "started")) {
        *text = "STARTED"; *col = COL_ORANGE;
    } else if (!strcmp(s, "stopping") || !strcmp(s, "stopping_retry")) {
        *text = "STOPPING"; *col = COL_ORANGE;
    } else if (!strcmp(s, "stopped_retry")) {
        *text = "RETRY"; *col = COL_ORANGE;
    } else if (!strcmp(s, "error")) {
        *text = "ERROR"; *col = COL_RED;
    } else if (!strcmp(s, "idle") || !strcmp(s, "stopped")) {
        *text = "STOPPED"; *col = COL_RED;
    } else {
        *text = "STANDBY"; *col = COL_ORANGE;
    }
}

void ui_update(const GenData &d) {
    char buf[32];

    // Top bar state
    const char *state_str;
    lv_color_t  state_col;
    gen_state_label(d, &state_str, &state_col);
    lv_label_set_text(lbl_state, state_str);
    lv_obj_set_style_text_color(lbl_state, state_col, 0);

    // Dashboard
    lv_label_set_text(lbl_gen_state, state_str);
    lv_obj_set_style_text_color(lbl_gen_state, state_col, 0);

    snprintf(buf, sizeof(buf), "%.0f", d.rpm);
    lv_label_set_text(lbl_rpm, buf);
    lv_arc_set_value(arc_rpm, (int16_t)(d.rpm + 0.5f));

    snprintf(buf, sizeof(buf), "%.1f V", d.bat_voltage);
    lv_label_set_text(lbl_bat, buf);
    lv_arc_set_value(arc_bat, (int16_t)(d.bat_voltage * 10.0f + 0.5f));

    snprintf(buf, sizeof(buf), "%.1f V", d.vrms);
    lv_label_set_text(lbl_volt, buf);
    lv_arc_set_value(arc_volt, (int16_t)(d.vrms * 10.0f + 0.5f));

    snprintf(buf, sizeof(buf), "%.0f W", d.power_active);
    lv_label_set_text(lbl_power, buf);
    lv_arc_set_value(arc_power, (int16_t)(d.power_active + 0.5f));

    snprintf(buf, sizeof(buf), "%.1f °C", d.head_temp);
    lv_label_set_text(lbl_temp, buf);
    lv_arc_set_value(arc_temp, (int16_t)(d.head_temp * 10.0f + 0.5f));

    snprintf(buf, sizeof(buf), "%.1f L", d.fuel_level);
    lv_label_set_text(lbl_fuel, buf);
    lv_arc_set_value(arc_fuel, (int16_t)(d.fuel_level * 10.0f + 0.5f));

    // Chart current-value labels — update every call
    snprintf(buf, sizeof(buf), "%.0f",   d.rpm);           lv_label_set_text(cv_rpm,  buf);
    snprintf(buf, sizeof(buf), "%.1f V", d.vrms);          lv_label_set_text(cv_volt, buf);
    snprintf(buf, sizeof(buf), "%.2f A", d.irms);          lv_label_set_text(cv_curr, buf);
    snprintf(buf, sizeof(buf), "%.0f W", d.power_active);  lv_label_set_text(cv_pwr,  buf);
    snprintf(buf, sizeof(buf), "%.1f°",  d.head_temp);     lv_label_set_text(cv_head, buf);
    snprintf(buf, sizeof(buf), "%.1f°",  d.ambient_temp);  lv_label_set_text(cv_amb,  buf);
    snprintf(buf, sizeof(buf), "%.1f°",  d.oil_temp);      lv_label_set_text(cv_oil,  buf);
    snprintf(buf, sizeof(buf), "%.1f°",  d.inverter_temp); lv_label_set_text(cv_inv,  buf);
    snprintf(buf, sizeof(buf), "%.1f°",  d.alternator_temp);lv_label_set_text(cv_alt, buf);

    // Charts — update every ~10 s (every 33rd call at 300 ms interval)
    // 60 points × 10 s = 10 min history
    static uint8_t chart_tick = 0;
    if (++chart_tick >= 33) {
        chart_tick = 0;
        lv_chart_set_next_value(chart_rpm,  ser_rpm,  (lv_coord_t)d.rpm);
        lv_chart_set_next_value(chart_volt, ser_volt, (lv_coord_t)d.vrms);
        lv_chart_set_next_value(chart_curr, ser_curr, (lv_coord_t)d.irms);
        lv_chart_set_next_value(chart_pwr,  ser_pwr,  (lv_coord_t)d.power_active);
        lv_chart_set_next_value(chart_head, ser_head, (lv_coord_t)d.head_temp);
        lv_chart_set_next_value(chart_amb,  ser_amb,  (lv_coord_t)d.ambient_temp);
        lv_chart_set_next_value(chart_oil,  ser_oil,  (lv_coord_t)d.oil_temp);
        lv_chart_set_next_value(chart_inv,  ser_inv,  (lv_coord_t)d.inverter_temp);
        lv_chart_set_next_value(chart_alt,  ser_alt,  (lv_coord_t)d.alternator_temp);
    }

    // Block start/retry when generator is running (only relevant when WS
    // connected). During cooldown the controller takes Start as "back to
    // running", so the buttons stay usable there.
    if (s_ws_connected) {
        if (d.running && strcmp(d.state, "cooldown") != 0) {
            lv_obj_add_state(btn_start, LV_STATE_DISABLED);
            lv_obj_add_state(btn_retry, LV_STATE_DISABLED);
        } else {
            lv_obj_clear_state(btn_start, LV_STATE_DISABLED);
            lv_obj_clear_state(btn_retry, LV_STATE_DISABLED);
        }
    }

    // Controls — sync switch states (don't trigger event)
    auto sync_sw = [](lv_obj_t *sw, bool on) {
        if (on) lv_obj_add_state(sw, LV_STATE_CHECKED);
        else    lv_obj_clear_state(sw, LV_STATE_CHECKED);
    };
    sync_sw(sw_choke,   d.choke);
    sync_sw(sw_fuel,    d.fuel_pump);
    sync_sw(sw_eco,     d.eco_mode);
    sync_sw(sw_load,    d.load_relay);
    sync_sw(sw_charger, d.charger);

    // Indicators
    set_indicator(ind_overload, d.overload,  COL_RED);
    set_indicator(ind_oil,      d.oil,       COL_RED);
    set_indicator(ind_output,   d.output_ok, COL_GREEN);
    set_indicator(ind_overheat, d.overheat,  COL_RED);
    set_indicator(ind_gen,      d.gen_sense, COL_GREEN);
    set_indicator(ind_bat,      d.bat_sense, COL_GREEN);

    // Overdue-maintenance bar — text only rebuilt on an actual mask change,
    // not every 300 ms tick. Bar hides itself automatically once a Record
    // tap clears the corresponding bit on the controller (next WS report,
    // ~1 s later) — no separate "recorded OK" toast needed.
    if (d.maint_due_mask != s_maint_mask) {
        s_maint_mask = d.maint_due_mask;
        if (s_maint_mask == 0) {
            lv_obj_add_flag(maint_bar, LV_OBJ_FLAG_HIDDEN);
        } else {
            char list_buf[100] = "";
            int list_len = 0;
            for (uint8_t i = 0; i < MAINT_TYPES; i++) {
                if (!(s_maint_mask & (1u << i))) continue;
                int a = snprintf(list_buf + list_len, sizeof(list_buf) - list_len,
                                  "%s%s", list_len ? ", " : "", MAINT_TYPE_LABELS[i]);
                if (a > 0) list_len += a;
                if ((size_t)list_len >= sizeof(list_buf)) break;
            }
            char full[140];
            snprintf(full, sizeof(full), "Maintenance overdue:\n%s", list_buf);
            lv_label_set_text(maint_bar_lbl, full);
            lv_obj_clear_flag(maint_bar, LV_OBJ_FLAG_HIDDEN);
        }
    }

    // Power tab
    snprintf(buf, sizeof(buf), "%.2f V", d.vrms);        lv_label_set_text(lbl_vrms, buf);
    snprintf(buf, sizeof(buf), "%.3f A", d.irms);        lv_label_set_text(lbl_irms, buf);
    snprintf(buf, sizeof(buf), "%.1f W", d.power_active);lv_label_set_text(lbl_pw, buf);
    snprintf(buf, sizeof(buf), "%.1f VA",d.power_apparent);lv_label_set_text(lbl_pa, buf);
    snprintf(buf, sizeof(buf), "%.1f VAR",d.power_reactive);lv_label_set_text(lbl_pr, buf);
    snprintf(buf, sizeof(buf), "%.3f",   d.power_factor); lv_label_set_text(lbl_pf, buf);
    snprintf(buf, sizeof(buf), "%.2f Hz",  d.freq);           lv_label_set_text(lbl_hz,    buf);
    snprintf(buf, sizeof(buf), "%.1f",     d.head_temp);      lv_label_set_text(lbl_htemp, buf);
    snprintf(buf, sizeof(buf), "%.1f L",   d.fuel_level);     lv_label_set_text(lbl_fuel2, buf);
    snprintf(buf, sizeof(buf), "%.1f",     d.oil_temp);       lv_label_set_text(lbl_oil_t, buf);
    snprintf(buf, sizeof(buf), "%.1f",     d.ambient_temp);   lv_label_set_text(lbl_amb_t, buf);
    snprintf(buf, sizeof(buf), "%.1f",     d.inverter_temp);  lv_label_set_text(lbl_inv_t, buf);
    snprintf(buf, sizeof(buf), "%.1f",     d.alternator_temp);lv_label_set_text(lbl_alt_t, buf);
    snprintf(buf, sizeof(buf), "%.2f V",   d.bat_voltage);    lv_label_set_text(lbl_bat_v, buf);
    snprintf(buf, sizeof(buf), "%.2f V",   d.core_voltage);   lv_label_set_text(lbl_core_v,buf);
    if (d.rssi) snprintf(buf, sizeof(buf), "%ddBm", d.rssi);
    else        strlcpy(buf, "--", sizeof(buf));
    lv_label_set_text(lbl_rssi, buf);

    uint32_t h = d.uptime / 3600, m = (d.uptime % 3600) / 60, s = d.uptime % 60;
    snprintf(buf, sizeof(buf), "%02dh %02dm %02ds", (int)h, (int)m, (int)s);
    lv_label_set_text(lbl_uptime, buf);
}

// ---------------------------------------------------------------------------
// OTA overlay
// ---------------------------------------------------------------------------
static lv_obj_t *ota_overlay = nullptr;
static lv_obj_t *ota_bar     = nullptr;
static lv_obj_t *ota_lbl     = nullptr;

void ui_ota_show(void) {
    if (ota_overlay) return;

    ota_overlay = lv_obj_create(lv_scr_act());
    lv_obj_set_size(ota_overlay, UI_W, UI_H);
    lv_obj_align(ota_overlay, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_bg_color(ota_overlay, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(ota_overlay, LV_OPA_80, 0);
    lv_obj_set_style_border_width(ota_overlay, 0, 0);
    lv_obj_clear_flag(ota_overlay, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(ota_overlay);
    lv_label_set_text(title, "Firmware Update");
    lv_obj_set_style_text_color(title, COL_TXT, 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -40);

    ota_bar = lv_bar_create(ota_overlay);
    lv_obj_set_size(ota_bar, 360, 20);
    lv_bar_set_range(ota_bar, 0, 100);
    lv_bar_set_value(ota_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(ota_bar, COL_BORDER, 0);
    lv_obj_set_style_bg_color(ota_bar, COL_ACCENT, LV_PART_INDICATOR);
    lv_obj_align(ota_bar, LV_ALIGN_CENTER, 0, 0);

    ota_lbl = lv_label_create(ota_overlay);
    lv_label_set_text(ota_lbl, "0%");
    lv_obj_set_style_text_color(ota_lbl, COL_TXT2, 0);
    lv_obj_set_style_text_font(ota_lbl, &lv_font_montserrat_16, 0);
    lv_obj_align(ota_lbl, LV_ALIGN_CENTER, 0, 32);
}

void ui_ota_set_progress(int percent) {
    if (!ota_bar || !ota_lbl) return;
    lv_bar_set_value(ota_bar, percent, LV_ANIM_OFF);
    char buf[16];
    snprintf(buf, sizeof(buf), "%d%%", percent);
    lv_label_set_text(ota_lbl, buf);
}

void ui_ota_hide(void) {
    if (!ota_overlay) return;
    lv_obj_del(ota_overlay);
    ota_overlay = nullptr;
    ota_bar     = nullptr;
    ota_lbl     = nullptr;
}

// 0 = empty … 4 = full. 30 mV hysteresis so the glyph does not flicker
// when the filtered voltage sits on a boundary. Green starts at 3.70 V,
// which falls inside the "2 bars" bucket, so color is tracked separately.
static int panel_bat_level(float volts, int prev) {
    static const float edge[] = {3.45f, 3.65f, 3.80f, 4.00f};
    int level = 0;
    for (int i = 0; i < 4; i++) {
        if (volts >= edge[i]) level = i + 1;
    }
    if (prev >= 0 && level != prev) {
        float boundary = (level > prev) ? edge[level - 1] : edge[prev - 1];
        float d = volts - boundary;
        if (d < 0) d = -d;
        if (d < 0.03f) return prev;
    }
    return level;
}

void ui_set_panel_battery(bool present, float volts) {
    static bool shown = false;
    static int level = -1;
    static int color_key = -1;

    if (!present) {
        if (shown) {
            lv_obj_add_flag(lbl_panel_bat, LV_OBJ_FLAG_HIDDEN);
            lv_obj_align(lbl_state, LV_ALIGN_RIGHT_MID, -6, 0);
            shown = false;
            level = -1;
            color_key = -1;
        }
        return;
    }

    int next = panel_bat_level(volts, level);
    int next_color = (volts >= 3.70f) ? 2 : (volts >= 3.45f) ? 1 : 0;
    if (next != level || next_color != color_key || !shown) {
        static const char *syms[] = {
            LV_SYMBOL_BATTERY_EMPTY,
            LV_SYMBOL_BATTERY_1,
            LV_SYMBOL_BATTERY_2,
            LV_SYMBOL_BATTERY_3,
            LV_SYMBOL_BATTERY_FULL,
        };
        lv_color_t col = (next_color == 2) ? COL_GREEN
                       : (next_color == 1) ? COL_ORANGE
                                           : COL_RED;
        lv_label_set_text(lbl_panel_bat, syms[next]);
        lv_obj_set_style_text_color(lbl_panel_bat, col, 0);
        level = next;
        color_key = next_color;
    }
    if (!shown) {
        lv_obj_clear_flag(lbl_panel_bat, LV_OBJ_FLAG_HIDDEN);
        shown = true;
    }
    // Icon stays on the right edge. State text width changes
    // (RUNNING / STARTING / STOPPED / …), so re-anchor it every tick.
    lv_obj_align(lbl_panel_bat, LV_ALIGN_RIGHT_MID, -4, 0);
    lv_obj_align_to(lbl_state, lbl_panel_bat, LV_ALIGN_OUT_LEFT_MID, -6, 0);
}

// dBm → 0-4 bars. Thresholds are the usual rule-of-thumb Android/ESP32
// cutoffs (not a spec, just a widely-used convention) — not meant to be
// precise, just to sort "great/usable/marginal/bad" into a glance-able bar
// count instead of a number that needs context to interpret.
static void set_signal_bars(int level, lv_color_t lit_col) {
    for (int i = 0; i < 4; i++) {
        lv_obj_set_style_bg_color(wifi_bar[i],
            (i < level) ? lit_col : lv_color_hex(0x3a3a3a), 0);
    }
}

static int wifi_rssi_to_level(int rssi) {
    if (rssi >= -55) return 4;
    if (rssi >= -67) return 3;
    if (rssi >= -75) return 2;
    if (rssi >= -85) return 1;
    return 0;
}

void ui_set_wifi_status(wifi_ui_state_t st, int rssi) {
    lv_color_t lit_col;
    int level;
    switch (st) {
        case WIFI_ST_CONNECTED:
            level   = wifi_rssi_to_level(rssi);
            lit_col = COL_GREEN;
            break;
        case WIFI_ST_CONNECTING:
            level   = 1;  // one lit bar as a lightweight "searching" cue
            lit_col = COL_ORANGE;
            break;
        default:
            level   = 0;
            lit_col = COL_RED;
            break;
    }
    set_signal_bars(level, lit_col);
}

void ui_set_link_status(link_ui_state_t link, int espnow_rssi) {
    bool connected = link != LINK_UI_NONE;
    s_ws_connected = connected;
    // Same text for both links; the colour (and the bars) tell them apart.
    lv_label_set_text(lbl_ws, connected ? "GEN: CONNECTED" : "GEN: --");
    lv_obj_set_style_text_color(lbl_ws, link == LINK_UI_WS     ? COL_GREEN :
                                        link == LINK_UI_ESPNOW ? COL_BLUE  : COL_TXT2, 0);
    // One signal indicator: while ESP-NOW carries the generator data the
    // WiFi bars show that link instead (blue — orange already means "WiFi
    // connecting"). Overrides what ui_set_wifi_status() just drew.
    if (link == LINK_UI_ESPNOW) set_signal_bars(wifi_rssi_to_level(espnow_rssi), COL_BLUE);

    // Lock all controls with no link to the generator; either link unlocks them
    auto set_dis = [](lv_obj_t *o, bool dis) {
        if (dis) lv_obj_add_state(o, LV_STATE_DISABLED);
        else     lv_obj_clear_state(o, LV_STATE_DISABLED);
    };
    bool dis = !connected;
    set_dis(btn_start,     dis);
    set_dis(btn_retry,     dis);
    set_dis(btn_stop_ctrl, dis);
    set_dis(sw_choke,      dis);
    set_dis(sw_fuel,       dis);
    set_dis(sw_eco,        dis);
    set_dis(sw_load,       dis);
    set_dis(sw_charger,    dis);
}

void ui_set_gen_host(const char *host) {
    lv_textarea_set_text(ta_host, host);
}

void ui_gen_list_refresh(void) {
    static uint32_t last_ms = 0;
    static char     last_opts[(DD_GEN_MAX + DD_LAN_MAX) * 48 + 16] = "";
    if (millis() - last_ms < 2000 || lv_dropdown_is_open(dd_gen)) return;
    last_ms = millis();

    lv_obj_clear_state(dd_gen, LV_STATE_DISABLED);
    s_dd_count     = espnow_client_enabled() ? espnow_list_generators(s_dd_gens, DD_GEN_MAX) : 0;
    s_dd_lan_count = udp_discovery_list(s_dd_lan, DD_LAN_MAX);

    char opts[sizeof(last_opts)];
    uint16_t sel = 0;
    size_t n = strlcpy(opts, "(none)", sizeof(opts));
    for (int i = 0; i < s_dd_count && n < sizeof(opts); i++) {
        const EspNowGenInfo &g = s_dd_gens[i];
        char label[56];
        const char *name = g.name[0] ? g.name : "?";
        if (g.seen) snprintf(label, sizeof(label), "\n%s  %d dBm [ESP-NOW]", name, g.rssi);
        else        snprintf(label, sizeof(label), "\n%s  (not seen) [ESP-NOW]", name);
        n = strlcat(opts, label, sizeof(opts));
        if (g.selected) sel = i + 1;
    }
    for (int i = 0; i < s_dd_lan_count && n < sizeof(opts); i++) {
        const UdpGenInfo &g = s_dd_lan[i];
        char label[56];
        snprintf(label, sizeof(label), "\n%s  %s [LAN]", g.name[0] ? g.name : "?", g.ip);
        n = strlcat(opts, label, sizeof(opts));
    }
    if (strcmp(opts, last_opts) != 0) {
        strlcpy(last_opts, opts, sizeof(last_opts));
        lv_dropdown_set_options(dd_gen, opts);
    }
    lv_dropdown_set_selected(dd_gen, sel);
}

bool ui_take_lan_pick(char *buf, size_t len) {
    if (!s_lan_pick_pending) return false;
    s_lan_pick_pending = false;
    strlcpy(buf, s_lan_pick_ip, len);
    return true;
}

void ui_wifi_scan_results(int count) {
    lv_obj_clean(list_networks);
    if (count <= 0) {
        lv_obj_t *lbl = lv_label_create(list_networks);
        lv_label_set_text(lbl, "No networks");
        lv_obj_set_style_text_color(lbl, COL_TXT2, 0);
        return;
    }
    char buf[80];
    for (int i = 0; i < count; i++) {
        snprintf(buf, sizeof(buf), "%s (%ddBm)",
                 WiFi.SSID(i).c_str(), WiFi.RSSI(i));
        lv_obj_t *item = lv_list_add_btn(list_networks, nullptr, buf);
        lv_obj_set_style_bg_color(item, COL_CARD, 0);
        lv_obj_set_style_text_color(item, COL_TXT, 0);
        lv_obj_set_style_text_font(item, &lv_font_montserrat_12, 0);
        lv_obj_add_event_cb(item, net_list_cb, LV_EVENT_CLICKED, nullptr);
    }
}

void ui_on_connect_request(void (*cb)(const char *, const char *,
                                      const char *)) {
    connect_cb = cb;
}
