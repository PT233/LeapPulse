#include "peripherals.h"

#include <stdalign.h>
#include <string.h>

#include "board_port.h"
#include "lvgl.h"

#define OLED_ADDR 0x3c
#define OLED_CMD 0x00
#define OLED_DATA 0x40
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_PAGES (OLED_HEIGHT / 8)
#define LVGL_PALETTE_BYTES 8
#define FRAME_PERIOD_MS 33u
#define OLED_RETRY_MS 250u

#define DRV_ADDR 0x5a
#define DRV_MODE 0x01
#define DRV_LIBRARY 0x03
#define DRV_SEQUENCE 0x04
#define DRV_GO 0x0c
#define DRV_CLAMP 0x17
#define DRV_FEEDBACK 0x1a
#define DRV_CONTROL3 0x1d
#define DRV_STANDBY 0x40
#define DRV_INTERNAL_TRIGGER 0x00
#define DRV_LRA_LIBRARY 6
#define DRV_STRONG_CLICK 1
#define DRV_WAIT_50_MS 0x85
#define DRV_CHECK_DELAY_MS 300u
#define DRV_RECHECK_MS 50u

#define ENCODER_DEBOUNCE_MS 20u
#define ENCODER_STEPS 4

static alignas(4) uint8_t lv_buffer[OLED_WIDTH * OLED_HEIGHT / 8 + LVGL_PALETTE_BYTES];
static uint8_t oled_target[OLED_PAGES][OLED_WIDTH];
static uint8_t oled_sent[OLED_PAGES][OLED_WIDTH];
static uint8_t oled_valid_pages;
static uint8_t oled_page_cursor;
static uint32_t oled_last_failure;
static bool oled_retry;
static bool oled_powered;
static lv_display_t *display;
static lv_obj_t *count_label;
static lv_obj_t *bpm_label;
static lv_obj_t *mode_label;
static lv_obj_t *goal_label;

static int previous_ab = -1;
static int encoder_steps;
static int encoder_delta;
static bool encoder_detent_seen;
static uint32_t encoder_last_detent;

enum haptic_event {
    HAPTIC_NONE,
    HAPTIC_GOAL,
    HAPTIC_ALERT,
};

static bool haptic_ready;
static bool haptic_active;
static enum haptic_event haptic_pending;
static uint32_t haptic_last_check;
static uint32_t haptic_check_delay;

static bool oled_init(void)
{
    /* Internal charge pump, horizontal addressing and 128x64 glass. */
    static const uint8_t setup[] = {
        0xae, 0xd5, 0x80, 0xa8, 63, 0xd3, 0, 0x40,
        0x8d, 0x14, 0x20, 0, 0xa1, 0xc8, 0xda, 0x12,
        0x81, 0x7f, 0xd9, 0xf1, 0xdb, 0x40, 0xa4, 0xa6, 0x2e,
    };
    return lp_board_i2c_write(OLED_ADDR, OLED_CMD, setup, sizeof(setup)) == 0;
}

static void oled_flush(lv_display_t *panel, const lv_area_t *area, uint8_t *pixels)
{
    (void)area; /* FULL mode always supplies the whole screen. */
    pixels += LVGL_PALETTE_BYTES;

    for (unsigned page = 0; page < OLED_PAGES; ++page) {
        for (unsigned col = 0; col < OLED_WIDTH; ++col) {
            uint8_t column = 0;
            for (unsigned bit = 0; bit < 8; ++bit) {
                unsigned row = page * 8 + bit;
                uint8_t pixel = pixels[row * (OLED_WIDTH / 8) + col / 8];
                column |= ((pixel >> (7 - col % 8)) & 1u) << bit;
            }
            oled_target[page][col] = column;
        }
    }
    lv_display_flush_ready(panel);
}

static void oled_send_page(uint32_t now)
{
    if (oled_retry && now - oled_last_failure < OLED_RETRY_MS) {
        return;
    }
    oled_retry = false;

    if (!oled_powered && oled_valid_pages == UINT8_MAX) {
        const uint8_t on = 0xaf;
        if (lp_board_i2c_write(OLED_ADDR, OLED_CMD, &on, 1)) {
            oled_last_failure = now;
            oled_retry = true;
            return;
        }
        oled_powered = true;
    }

    for (unsigned scan = 0; scan < OLED_PAGES; ++scan) {
        unsigned page = oled_page_cursor;
        oled_page_cursor = (page + 1) % OLED_PAGES;
        uint8_t *target = oled_target[page];
        uint8_t *sent = oled_sent[page];
        unsigned first = 0;
        unsigned last = OLED_WIDTH - 1;
        uint8_t valid_bit = 1u << page;
        if (oled_valid_pages & valid_bit) {
            while (first < OLED_WIDTH && target[first] == sent[first]) {
                ++first;
            }
            if (first == OLED_WIDTH) {
                continue;
            }
            while (last > first && target[last] == sent[last]) {
                --last;
            }
        }

        uint8_t address[] = {0x21, first, last, 0x22, page, page};
        size_t size = last - first + 1;
        if (lp_board_i2c_write(OLED_ADDR, OLED_CMD, address, sizeof(address)) ||
            lp_board_i2c_write(OLED_ADDR, OLED_DATA, target + first, size)) {
            oled_valid_pages &= (uint8_t)~valid_bit;
            oled_last_failure = now;
            oled_retry = true;
        } else {
            memcpy(sent + first, target + first, size);
            oled_valid_pages |= valid_bit;
        }
        return;
    }
}

static lv_obj_t *label_at(lv_obj_t *parent, const char *text, const lv_font_t *font, int x, int y)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_pos(label, x, y);
    lv_label_set_text(label, text);
    return label;
}

static void ui_init(void)
{
    lv_init();
    lv_tick_set_cb(lp_board_ms);
    display = lv_display_create(OLED_WIDTH, OLED_HEIGHT);
    if (!display) {
        return;
    }
    lv_display_set_color_format(display, LV_COLOR_FORMAT_I1);
    lv_display_set_buffers(display, lv_buffer, NULL, sizeof(lv_buffer), LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(display, oled_flush);
    lv_obj_t *screen = lv_display_get_screen_active(display);
    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_set_scrollable(screen, false);

    label_at(screen, "JUMPS", &lv_font_montserrat_14, 4, 22);
    label_at(screen, "BPM", &lv_font_montserrat_14, 76, 22);
    count_label = label_at(screen, "000", &lv_font_montserrat_20, 4, 39);
    bpm_label = label_at(screen, "---", &lv_font_montserrat_20, 76, 39);
    goal_label = label_at(screen, "", &lv_font_montserrat_14, 4, 2);

    lv_obj_t *icon = lv_obj_create(screen);
    lv_obj_remove_style_all(icon);
    lv_obj_set_size(icon, 16, 16);
    lv_obj_set_pos(icon, 108, 2);
    lv_obj_set_scrollable(icon, false);
    lv_obj_set_style_border_width(icon, 1, 0);
    lv_obj_set_style_border_color(icon, lv_color_white(), 0);
    mode_label = label_at(icon, "F", &lv_font_montserrat_14, 0, 0);
    lv_obj_center(mode_label);

    lv_refr_now(display);
}

static void ui_update(uint32_t now, uint32_t jumps, uint16_t bpm, bool valid, uint8_t mode,
                      bool complete)
{
    static const char *const mode_icons[] = {"F", "T", "C"};
    static unsigned displayed_count;
    static unsigned displayed_bpm;
    static bool displayed_valid;
    static bool displayed_complete;
    static uint8_t displayed_mode;
    static uint32_t last_frame;

    if (!display) {
        return;
    }
    unsigned count = jumps > 999 ? 999u : (unsigned)jumps;
    unsigned pulse = valid ? (bpm > 999 ? 999u : (unsigned)bpm) : 0;
    if (now - last_frame >= FRAME_PERIOD_MS &&
        (count != displayed_count || pulse != displayed_bpm || valid != displayed_valid ||
         mode != displayed_mode || complete != displayed_complete)) {
        lv_label_set_text_fmt(count_label, "%03u", count);
        if (valid) {
            lv_label_set_text_fmt(bpm_label, "%03u", pulse);
        } else {
            lv_label_set_text(bpm_label, "---");
        }
        lv_label_set_text(mode_label, mode < 3 ? mode_icons[mode] : "?");
        lv_label_set_text(goal_label, complete ? "GOAL" : "");
        lv_refr_now(display);
        displayed_count = count;
        displayed_bpm = pulse;
        displayed_valid = valid;
        displayed_mode = mode;
        displayed_complete = complete;
        last_frame = now;
    }
    oled_send_page(now);
}

static bool haptic_write(uint8_t reg, uint8_t value)
{
    return lp_board_i2c_write(DRV_ADDR, reg, &value, 1) == 0;
}

static bool haptic_init(void)
{
    uint8_t feedback, control;
    if (!haptic_write(DRV_MODE, DRV_INTERNAL_TRIGGER) ||
        lp_board_i2c_read(DRV_ADDR, DRV_FEEDBACK, &feedback, 1) ||
        lp_board_i2c_read(DRV_ADDR, DRV_CONTROL3, &control, 1)) {
        return false;
    }
    /* LRA with closed-loop resonant tracking. Bound the overdrive clamp. */
    return haptic_write(DRV_FEEDBACK, feedback | 0x80) &&
           haptic_write(DRV_CONTROL3, control & 0xfe) &&
           haptic_write(DRV_CLAMP, 0x60) &&
           haptic_write(DRV_LIBRARY, DRV_LRA_LIBRARY) &&
           haptic_write(DRV_MODE, DRV_STANDBY);
}

static void haptic_update(uint32_t now)
{
    if (!haptic_ready) {
        return;
    }
    if (haptic_active) {
        if (now - haptic_last_check < haptic_check_delay) {
            return;
        }
        uint8_t go;
        haptic_last_check = now;
        haptic_check_delay = DRV_RECHECK_MS;
        if (lp_board_i2c_read(DRV_ADDR, DRV_GO, &go, 1) || (go & 1u) ||
            !haptic_write(DRV_MODE, DRV_STANDBY)) {
            return;
        }
        haptic_active = false;
    }

    if (haptic_pending != HAPTIC_NONE) {
        static const uint8_t goal[] = {DRV_STRONG_CLICK, 0};
        static const uint8_t alert[] = {DRV_STRONG_CLICK, DRV_WAIT_50_MS, DRV_STRONG_CLICK, 0};
        bool double_click = haptic_pending == HAPTIC_ALERT;
        haptic_pending = HAPTIC_NONE;
        bool ready = lp_board_i2c_write(DRV_ADDR, DRV_SEQUENCE, double_click ? alert : goal,
                                        double_click ? sizeof(alert) : sizeof(goal)) == 0;
        if (ready && haptic_write(DRV_MODE, DRV_INTERNAL_TRIGGER)) {
            lp_board_delay_ms(1); /* Give the oscillator time to exit standby. */
            /* The bounded ROM sequence stops without further MCU traffic. */
            (void)haptic_write(DRV_GO, 1);
            haptic_active = true;
            haptic_last_check = now;
            haptic_check_delay = DRV_CHECK_DELAY_MS;
        }
    }
}

static void encoder_update(uint32_t now)
{
    static const int8_t transitions[4][4] = {
        {0, 1, -1, 0}, {-1, 0, 0, 1}, {1, 0, 0, -1}, {0, -1, 1, 0},
    };
    int ab = lp_board_encoder_ab();
    if (ab < 0 || ab > 3) {
        previous_ab = -1;
        encoder_steps = 0;
        return;
    }
    if (previous_ab >= 0 && ab != previous_ab) {
        int step = transitions[previous_ab][ab];
        encoder_steps = step ? encoder_steps + step : 0;
        if (encoder_steps == ENCODER_STEPS || encoder_steps == -ENCODER_STEPS) {
            if (!encoder_detent_seen || now - encoder_last_detent >= ENCODER_DEBOUNCE_MS) {
                encoder_delta += encoder_steps / ENCODER_STEPS;
                encoder_last_detent = now;
                encoder_detent_seen = true;
            }
            encoder_steps = 0;
        }
    }
    previous_ab = ab;
}

void lp_peripherals_init(void)
{
    lp_board_delay_ms(20);
    haptic_ready = haptic_init();
    if (oled_init()) {
        ui_init();
    }
}

void lp_peripherals_update(uint32_t jumps, uint16_t bpm, bool pulse_valid, uint8_t mode,
                           bool complete)
{
    uint32_t now = lp_board_ms();
    encoder_update(now);
    haptic_update(now);
    ui_update(now, jumps, bpm, pulse_valid, mode, complete);
}

int lp_peripherals_take_mode_delta(void)
{
    int delta = encoder_delta;
    encoder_delta = 0;
    return delta;
}

void lp_peripherals_goal(void)
{
    if (haptic_pending == HAPTIC_NONE) {
        haptic_pending = HAPTIC_GOAL;
    }
}

void lp_peripherals_alert(void)
{
    haptic_pending = HAPTIC_ALERT;
}

int lp_peripherals_battery_mv(void)
{
    return lp_board_battery_mv();
}
