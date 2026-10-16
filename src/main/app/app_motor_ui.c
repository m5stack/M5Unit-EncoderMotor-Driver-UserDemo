/**
 * @file app_motor_ui.c
 * @brief Shared 1280x720 LVGL interface for preview and real motor devices.
 *
 * The caller owns the model and holds the display lock for every UI entry point.
 * Event callbacks update staged values and enqueue snapshots; they never do I2C.
 */
#include "app_motor_ui.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

LV_FONT_DECLARE(motor_digits_96);

/* Shared palette for all control modes. */
#define BG 0x0B0E12
#define PANEL 0x13181E
#define LINE 0x29323D
#define MUTED 0x9BA8B7
#define WHITE 0xF1F5F9
#define BLUE 0x79C8FA
#define GREEN 0x8DE2AC
#define RED 0xF18D96

/**
 * @brief Borrowed model and cached objects; access only while the display lock is held.
 */
static motor_ui_model_t *m;
static lv_obj_t *root;
static lv_obj_t *body;
static lv_obj_t *nav;
static lv_obj_t *footer;
static lv_obj_t *device_label;
static lv_obj_t *status_label;
static lv_obj_t *status_dot;
static lv_obj_t *notice_label;
static lv_obj_t *run_button;
static lv_obj_t *stop_button;
static lv_obj_t *nav_buttons[MOTOR_DEVICE + 1];
static lv_obj_t *motor_panel;
static lv_obj_t *motor_count;
static lv_obj_t *motor_buttons[4];
static lv_obj_t *motor_names[4];
static lv_obj_t *motor_marks[4];
static lv_obj_t *motor_tracks[4];
static lv_obj_t *channel_fill[4];
static lv_obj_t *gauge_panel;
static lv_obj_t *gauge;

/* Only the changing content belongs to these child pages; the screen stays loaded. */
typedef struct {
    lv_obj_t *container;
    lv_obj_t *value_label;
    lv_obj_t *set_label;
    lv_obj_t *readout_direction;
    lv_obj_t *secondary_label;
    lv_obj_t *other_label;
    lv_obj_t *limit_label;
    lv_obj_t *target_slider;
    lv_obj_t *limit_slider;
    lv_obj_t *rgb_level_label;
    lv_obj_t *rgb_hex_label;
    lv_obj_t *rgb_swatch;
    lv_obj_t *rgb_pad;
    lv_obj_t *rgb_cursor;
    lv_obj_t *rgb_hue_slider;
    lv_color_hsv_t rgb_hsv;
} motor_content_page_t;

static motor_content_page_t content_pages[MOTOR_DEVICE + 1];
static motor_content_page_t *view;
static lv_obj_t *voltage_label;
static lv_obj_t *current_label;
static motor_ui_command_handler_t command_handler;
static void *command_context;

/* LVGL line objects borrow these buffers for their entire lifetime. */
static lv_point_precise_t ticks[41][2];
static const char *pages[] = {"PWM", "POSITION", "SPEED"};
static void refresh_view(void);

/**
 * @brief Preview and live control are editable only while device data is valid.
 */
static bool available(void)
{
    return (m->preview || m->hardware_control) && m->connected && !m->io_error;
}

static bool editable(void)
{
    return available() && !m->mode_switch_pending;
}

/**
 * @brief Copy the staged settings so later UI edits cannot alter an already queued command.
 */
static bool submit_page(motor_command_kind_t kind, motor_page_t page)
{
    motor_channel_ui_t *c = &m->motor[m->selected];
    motor_ui_command_t cmd = {
        .kind = kind,
        .page = page,
        .channel = m->selected,
        .device_index = m->device_index,
        .address = m->pending_address,
        .pwm = c->pwm_target,
        .speed = c->speed_target,
        .limit = c->limit,
        .position = c->position_target,
        .rgb_color = m->rgb_color,
        .rgb_brightness = m->rgb_brightness,
    };

    bool accepted = command_handler && command_handler(&cmd, command_context);
    if ((kind != MOTOR_CMD_UPDATE && kind != MOTOR_CMD_RGB) || !accepted) {
        snprintf(m->notice, sizeof(m->notice), "%s", accepted ? "Command queued" : "Command queue unavailable");
    }
    return accepted;
}

static bool submit(motor_command_kind_t kind)
{
    return submit_page(kind, m->page);
}

/**
 * @brief Reflect current hardware availability in the persistent header objects.
 */
static void update_device_status(void)
{
    uint32_t color = m->preview ? BLUE : m->io_error ? RED : m->connected ? GREEN : MUTED;
    lv_obj_set_style_text_color(status_label, lv_color_hex(color), 0);
    lv_obj_set_style_bg_color(status_dot, lv_color_hex(color), 0);
}

void motor_ui_set_command_handler(motor_ui_command_handler_t handler, void *context)
{
    command_handler = handler;
    command_context = context;
}

/**
 * @brief Create a noninteractive panel without inheriting theme padding or shadows.
 */
static lv_obj_t *box(lv_obj_t *p, int x, int y, int w, int h, uint32_t color, int radius)
{
    lv_obj_t *o = lv_obj_create(p);
    lv_obj_remove_style_all(o);
    lv_obj_set_pos(o, x, y);
    lv_obj_set_size(o, w, h);

    lv_obj_set_style_bg_color(o, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(o, radius, 0);
    if (radius > 0 && w >= 16 && h >= 16) {
        lv_obj_set_style_border_width(o, 1, 0);
        lv_obj_set_style_border_color(o, lv_color_hex(0x25303A), 0);
    }
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    return o;
}

/**
 * @brief Create a label with an explicit font, color and wrapping width.
 */
static lv_obj_t *text(lv_obj_t *p, int x, int y, int w, const char *s, const lv_font_t *f, uint32_t color)
{
    lv_obj_t *o = lv_label_create(p);
    lv_label_set_text(o, s);
    lv_obj_set_pos(o, x, y);
    lv_obj_set_width(o, w);

    lv_obj_set_style_text_font(o, f, 0);
    lv_obj_set_style_text_color(o, lv_color_hex(color), 0);
    lv_label_set_long_mode(o, LV_LABEL_LONG_WRAP);

    return o;
}

static lv_obj_t *center_text(lv_obj_t *p, int x, int y, int w, const char *s, const lv_font_t *f, uint32_t c)
{
    lv_obj_t *o = text(p, x, y, w, s, f, c);
    lv_obj_set_style_text_align(o, LV_TEXT_ALIGN_CENTER, 0);

    return o;
}

/**
 * @brief Keep normal, pressed, focus and disabled states consistent across pages.
 */
static lv_obj_t *button(lv_obj_t *p,
    int x,
    int y,
    int w,
    int h,
    const char *s,
    uint32_t bg,
    uint32_t fg,
    lv_event_cb_t cb,
    int id)
{
    lv_obj_t *o = lv_button_create(p);
    lv_obj_remove_style_all(o);
    lv_obj_set_pos(o, x, y);
    lv_obj_set_size(o, w, h);
    lv_obj_set_style_bg_color(o, lv_color_hex(bg), 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(o,
        lv_color_hex(bg == BLUE    ? 0xB0E3FF
                     : bg == GREEN ? 0x70C58F
                                   : 0x2A3D4C),
        LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(o, LV_OPA_40, LV_STATE_DISABLED);
    lv_obj_set_style_opa(o, LV_OPA_40, LV_STATE_DISABLED);
    lv_obj_set_style_radius(o, 12, 0);
    lv_obj_set_style_shadow_width(o, 0, LV_STATE_ANY);
    lv_obj_set_style_border_width(o, 1, 0);
    lv_obj_set_style_border_color(o, lv_color_hex(bg == GREEN ? 0xA9EDC0 : 0x2B3945), 0);
    lv_obj_set_style_border_color(o, lv_color_hex(BLUE), LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(o, 2, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_color(o, lv_color_hex(BLUE), LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_pad(o, 2, LV_STATE_FOCUS_KEY);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *t = text(o, 0, 0, w, s, &lv_font_montserrat_22, fg);
    lv_obj_set_style_text_align(t, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(t);
    if (cb) {
        lv_obj_add_event_cb(o, cb, LV_EVENT_CLICKED, (void *)(intptr_t)id);
    }
    return o;
}

static void notice(const char *s)
{
    snprintf(m->notice, sizeof(m->notice), "%s", s);
}

/**
 * @brief Clear stopped motion targets while leaving actual device feedback untouched.
 */
void motor_ui_reset_motion_targets(motor_ui_model_t *model, uint8_t channel)
{
    motor_channel_ui_t *c = &model->motor[channel];
    c->running = false;
    c->pwm_target = 0;
    c->speed_target = 0;
    c->position_target = 0;
}

/* Preview feedback can be stopped locally; live feedback comes from the worker. */
static void stop_all(void)
{
    for (int i = 0; i < 4; i++) {
        motor_ui_reset_motion_targets(m, i);
        m->motor[i].speed = 0;
        m->motor[i].pwm = 0;
    }
}

void motor_ui_finish_mode_switch(motor_ui_model_t *model, motor_page_t page, bool stopped)
{
    model->mode_switch_pending = false;
    if (stopped) {
        for (uint8_t ch = 0; ch < 4; ch++) {
            motor_ui_reset_motion_targets(model, ch);
        }
        model->page = page;
    }
}

static void on_nav(lv_event_t *e)
{
    motor_page_t page = (motor_page_t)(intptr_t)lv_event_get_user_data(e);
    if (page == m->page || m->mode_switch_pending) {
        return;
    }
    if (page == MOTOR_DEVICE) {
        m->page = page;
    } else if (m->hardware_control && m->connected) {
        if (submit_page(MOTOR_CMD_MODE, page)) {
            m->mode_switch_pending = true;
            notice("Stopping motors...");
        }
    } else {
        if (m->preview) {
            stop_all();
        }
        motor_ui_finish_mode_switch(m, page, true);
    }
    refresh_view();
}

static void on_channel(lv_event_t *e)
{
    if (m->mode_switch_pending) {
        return;
    }
    m->selected = (uint8_t)(intptr_t)lv_event_get_user_data(e);
    refresh_view();
}

/**
 * @brief Separate simulated motion from commands awaiting a hardware acknowledgement.
 */
static void on_run(lv_event_t *e)
{
    int action = (int)(intptr_t)lv_event_get_user_data(e);
    if (!editable()) {
        return;
    }

    if (m->hardware_control) {
        submit(action == 0 ? MOTOR_CMD_STOP_ALL : m->motor[m->selected].running ? MOTOR_CMD_STOP : MOTOR_CMD_START);
        refresh_view();

        return;
    }
    if (action == 0) {
        stop_all();
        notice("All channels stopped in preview.");
    } else {
        motor_channel_ui_t *c = &m->motor[m->selected];
        c->running = !c->running;
        if (c->running) {
            c->speed = m->page == MOTOR_PWM ? c->pwm / 4 : c->speed_target;
            if (m->page == MOTOR_POSITION) {
                c->encoder = c->position_target;
            }
        }
        if (!c->running) {
            motor_ui_reset_motion_targets(m, m->selected);
            c->speed = 0;
            c->pwm = 0;
        }
        notice(c->running ? "Preview running / simulated feedback" : "Selected channel stopped in preview.");
    }
    refresh_view();
}

/**
 * @brief Read the editable target; live PWM feedback is stored separately.
 */
static int target_value(void)
{
    motor_channel_ui_t *c = &m->motor[m->selected];
    if (m->page == MOTOR_PWM) {
        return m->hardware_control ? c->pwm_target : c->pwm;
    }

    if (m->page == MOTOR_SPEED) {
        return c->speed_target;
    }

    return c->position_target;
}

/**
 * @brief Center the magnitude independently of direction and unit labels.
 */
static void update_readout(void)
{
    if (!view->value_label) {
        return;
    }

    motor_channel_ui_t *c = &m->motor[m->selected];
    int32_t raw = m->page == MOTOR_PWM ? c->pwm * 100 / 127 : m->page == MOTOR_POSITION ? c->encoder : c->speed;

    /* Widen before negating so INT32_MIN remains representable. */
    int64_t magnitude = raw < 0 ? -(int64_t)raw : raw;
    char n[24];
    snprintf(n, sizeof(n), "%lld", (long long)magnitude);
    bool wide = strlen(n) > 5;
    lv_obj_set_style_text_font(view->value_label, wide ? &lv_font_montserrat_36 : &motor_digits_96, 0);
    lv_label_set_text(view->value_label, m->connected ? n : "--");
    lv_obj_align(view->value_label, LV_ALIGN_TOP_MID, 293 - 512, wide ? 189 : 174);
    if (view->readout_direction) {
        const char *sign = !m->connected               ? "--"
                           : raw < 0                   ? "-"
                           : raw > 0                   ? "+"
                           : m->page == MOTOR_POSITION ? "0"
                                                       : "--";
        lv_label_set_text_fmt(view->readout_direction, "%s %s", m->page == MOTOR_POSITION ? "Sign" : "Direction", sign);
    }
    if (gauge) {
        lv_arc_set_value(
            gauge, (int)(magnitude > lv_arc_get_max_value(gauge) ? lv_arc_get_max_value(gauge) : magnitude));
    }
}

static void update_target_labels(void)
{
    int v = target_value();
    if (view->set_label) {
        if (available()) {
            lv_label_set_text_fmt(view->set_label, "%+ld", (long)(m->page == MOTOR_PWM ? v * 100 / 127 : v));
        } else {
            lv_label_set_text(view->set_label, "--");
        }
    }
    update_readout();
    for (int i = 0; i < 4; i++) {
        if (channel_fill[i]) {
            int amount = m->connected && i < m->channels ? abs(m->motor[i].pwm) * 148 / 127 : 0;
            lv_obj_set_width(channel_fill[i], amount ? amount : 1);
            lv_obj_set_style_bg_opa(channel_fill[i], amount ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
        }
    }
}

/**
 * @brief Stage edits immediately; update hardware only for the acknowledged active mode.
 */
static void set_target(int v)
{
    motor_channel_ui_t *c = &m->motor[m->selected];
    if (m->page == MOTOR_PWM) {
        if (m->hardware_control) {
            c->pwm_target = LV_CLAMP(-127, v, 127);
        } else {
            c->pwm = LV_CLAMP(-127, v, 127);
        }
    } else if (m->page == MOTOR_SPEED) {
        c->speed_target = LV_CLAMP(-127, v, 127);
    } else {
        c->position_target = LV_CLAMP(-1000000, v, 1000000);
    }
    update_target_labels();
    if (m->hardware_control && c->running && c->active_page == m->page) {
        submit(MOTOR_CMD_UPDATE);
    }
}

static void on_slider(lv_event_t *e)
{
    if (editable()) {
        set_target(lv_slider_get_value((lv_obj_t *)lv_event_get_target(e)));
    }
}

static void update_limit_label(void)
{
    if (!view->limit_label) {
        return;
    }
    if (available()) {
        lv_label_set_text_fmt(view->limit_label, "%d %%", (m->motor[m->selected].limit * 100 + 63) / 127);
    } else {
        lv_label_set_text(view->limit_label, "-- %");
    }
}

static void on_limit_slider(lv_event_t *e)
{
    if (!editable()) {
        return;
    }

    motor_channel_ui_t *c = &m->motor[m->selected];
    c->limit = lv_slider_get_value((lv_obj_t *)lv_event_get_target(e));
    update_limit_label();
    if (m->hardware_control && c->running && c->active_page == MOTOR_POSITION) {
        submit(MOTOR_CMD_UPDATE);
    }
}

/**
 * @brief Use a 48px knob and extended hit area for finger operation on the panel.
 */
static lv_obj_t *large_slider(lv_obj_t *parent,
    int x,
    int y,
    int width,
    int low,
    int high,
    int value,
    lv_event_cb_t callback)
{
    lv_obj_t *slider = lv_slider_create(parent);
    lv_obj_set_pos(slider, x, y);
    lv_obj_set_size(slider, width, 20);
    lv_slider_set_range(slider, low, high);
    if (low < 0) {
        lv_slider_set_mode(slider, LV_SLIDER_MODE_SYMMETRICAL);
    }
    lv_slider_set_value(slider, LV_CLAMP(low, value, high), LV_ANIM_OFF);
    lv_obj_set_style_bg_color(slider, lv_color_hex(0x405466), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(slider, 10, LV_PART_MAIN);
    lv_obj_set_style_border_width(slider, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(slider, lv_color_hex(0x61788B), LV_PART_MAIN);
    lv_obj_set_style_bg_color(slider, lv_color_hex(BLUE), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(slider, lv_color_hex(WHITE), LV_PART_KNOB);
    lv_obj_set_style_pad_all(slider, 14, LV_PART_KNOB);
    lv_obj_set_style_shadow_width(slider, 0, LV_PART_KNOB | LV_STATE_ANY);
    lv_obj_set_ext_click_area(slider, 28);
    lv_obj_add_event_cb(slider, callback, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_set_style_opa(slider, LV_OPA_40, LV_STATE_DISABLED);
    if (!available()) {
        lv_obj_add_state(slider, LV_STATE_DISABLED);
    }
    return slider;
}

/**
 * @brief Switch deterministic preview fixtures without touching a real device.
 */
static void on_profile(lv_event_t *e)
{
    if (!m->preview || m->mode_switch_pending) {
        return;
    }

    unsigned channels = (unsigned)(uintptr_t)lv_event_get_user_data(e);
    motor_ui_demo_init(m, channels);
    m->page = MOTOR_DEVICE;
    notice("Preview profile changed. No hardware transaction.");
    refresh_view();
}

/**
 * @brief Only the apply action submits a persistent address write.
 */
static void on_address(lv_event_t *e)
{
    if (!editable()) {
        return;
    }

    int d = (int)(intptr_t)lv_event_get_user_data(e);
    if (d) {
        m->pending_address = (uint8_t)LV_CLAMP(8, m->pending_address + d, 0x77);
    } else if (m->hardware_control) {
        submit(MOTOR_CMD_ADDRESS);
    } else {
        m->address = m->pending_address;
        notice("Preview address updated. No Flash write.");
    }
    refresh_view();
}

static void on_next_device(lv_event_t *e)
{
    (void)e;
    if (!m->mode_switch_pending && !m->preview && m->device_count > 1) {
        m->device_index = (m->device_index + 1) % m->device_count;
        m->selected = 0;
        notice("Switching device...");
    }
}

/**
 * @brief Initialize controlled sample data shared by the app and simulator.
 */
void motor_ui_demo_init(motor_ui_model_t *model, unsigned channels)
{
    memset(model, 0, sizeof(*model));
    model->preview = true;
    model->channels = channels == 1 ? 1 : 4;
    model->connected = channels != 0;
    model->address = channels == 1 ? 0x2F : 0x24;
    model->pending_address = model->address;
    model->version = 1;
    model->millivolts = 11940;
    model->milliamps = 440;
    model->rgb_brightness = 128;
    for (int i = 0; i < 4; i++) {
        model->motor[i].limit = 23;
        model->motor[i].speed_target = 18;
        model->motor[i].position_target = 360;
    }
    snprintf(model->notice, sizeof(model->notice), "UI preview / simulated device and feedback");
}

/**
 * @brief Show channel availability and selection without inventing a device run state.
 */
static void channel_list(void)
{
    motor_panel = box(body, 0, 0, 240, 426, BG, 0);
    box(motor_panel, 32, 0, 208, 426, PANEL, 16);
    text(motor_panel, 50, 18, 124, "Motors", &lv_font_montserrat_22, MUTED);
    motor_count = text(motor_panel, 172, 18, 46, "", &lv_font_montserrat_22, WHITE);
    lv_obj_set_style_text_align(motor_count, LV_TEXT_ALIGN_RIGHT, 0);
    for (int i = 0; i < 4; i++) {
        lv_obj_t *b = motor_buttons[i] = button(motor_panel, 46, 60 + i * 83, 180, 70, "", PANEL, WHITE, on_channel, i);
        motor_marks[i] = box(b, 0, 16, 4, 38, BG, 0);
        char name[12];
        snprintf(name, sizeof(name), "M%d", i + 1);
        motor_names[i] = text(b, 16, 10, 100, name, &lv_font_montserrat_28, MUTED);
        motor_tracks[i] = box(b, 16, 51, 148, 3, LINE, 0);
        channel_fill[i] = box(b, 16, 51, 1, 3, 0x536F87, 0);
    }
}

/**
 * @brief Build the magnitude arc, persistent tick buffers and separate sign/unit labels.
 */
static void create_gauge(void)
{

    const int cx = 293, cy = 210, r = 180;
    gauge = lv_arc_create(gauge_panel);
    lv_obj_set_pos(gauge, cx - r, cy - r);
    lv_obj_set_size(gauge, r * 2, r * 2);
    lv_arc_set_bg_angles(gauge, 135, 405);
    lv_arc_set_range(gauge, 0, 100);
    lv_arc_set_value(gauge, 0);
    lv_obj_remove_style(gauge, NULL, LV_PART_KNOB);
    lv_obj_remove_flag(gauge, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_color(gauge, lv_color_hex(LINE), LV_PART_MAIN);
    lv_obj_set_style_arc_width(gauge, 7, LV_PART_MAIN);
    lv_obj_set_style_arc_color(gauge, lv_color_hex(BLUE), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(gauge, 7, LV_PART_INDICATOR);

    for (int i = 0; i <= 40; i++) {
        double a = (135.0 + 270.0 * i / 40.0) * 3.141592653589793 / 180.0;
        int ro = 166, ri = i % 5 == 0 ? 145 : 155;
        ticks[i][0] = (lv_point_precise_t){cx + ro * cos(a), cy + ro * sin(a)};
        ticks[i][1] = (lv_point_precise_t){cx + ri * cos(a), cy + ri * sin(a)};
        lv_obj_t *l = lv_line_create(gauge_panel);
        lv_line_set_points(l, ticks[i], 2);
        lv_obj_set_style_line_color(l, lv_color_hex(i % 5 == 0 ? MUTED : LINE), 0);
        lv_obj_set_style_line_width(l, i % 5 == 0 ? 2 : 1, 0);
    }
}

static void draw_gauge(int max, const char *unit)
{
    const int cx = 293, cy = 210;
    for (int i = 0; i <= 4; i++) {
        double a = (135.0 + 270.0 * i / 4.0) * 3.141592653589793 / 180.0;
        char number[16];
        snprintf(number, sizeof(number), "%d", (max * i + 2) / 4);
        center_text(view->container, cx + (int)(198 * cos(a)) - 36, cy + (int)(198 * sin(a)) - 11, 72, number,
            &lv_font_montserrat_18, MUTED);
    }

    text(view->container, 20, 10, 220, unit, &lv_font_montserrat_22, MUTED);
    view->readout_direction = text(view->container, 394, 10, 180, "", &lv_font_montserrat_22, MUTED);
    lv_obj_set_style_text_align(view->readout_direction, LV_TEXT_ALIGN_RIGHT, 0);
    view->value_label = text(view->container, 0, 174, LV_SIZE_CONTENT, "", &motor_digits_96, WHITE);
    update_readout();

    motor_channel_ui_t *c = &m->motor[m->selected];
    char secondary[48], other[48];
    if (!m->connected) {
        snprintf(secondary, sizeof(secondary), "--");
        snprintf(other, sizeof(other), "--");
    } else if (m->page == MOTOR_POSITION) {
        if (m->preview || m->hardware_control) {
            snprintf(secondary, sizeof(secondary), "%+d ct / 20 ms", c->speed);
            snprintf(other, sizeof(other), "%+lld counts", (long long)c->position_target - c->encoder);
        } else {
            snprintf(secondary, sizeof(secondary), "--");
            snprintf(other, sizeof(other), "--");
        }
    } else {
        snprintf(secondary, sizeof(secondary), "%+ld counts", (long)c->encoder);
        if (m->page == MOTOR_SPEED) {
            snprintf(other, sizeof(other), "%+d %%", c->pwm * 100 / 127);
        } else {
            snprintf(other, sizeof(other), "%+d ct / 20 ms", c->speed);
        }
    }
    text(view->container, 34, 370, 240, m->page == MOTOR_POSITION ? "Speed" : "Encoder", &lv_font_montserrat_22, MUTED);
    view->secondary_label = text(view->container, 34, 393, 260, secondary, &lv_font_montserrat_22, WHITE);
    lv_obj_t *t = text(view->container, 314, 370, 260,
        m->page == MOTOR_POSITION ? "Position delta"
        : m->page == MOTOR_SPEED  ? "Output"
                                  : "Speed",
        &lv_font_montserrat_22, MUTED);
    lv_obj_set_style_text_align(t, LV_TEXT_ALIGN_RIGHT, 0);
    view->other_label = t = text(view->container, 314, 393, 260, other, &lv_font_montserrat_22, WHITE);
    lv_obj_set_style_text_align(t, LV_TEXT_ALIGN_RIGHT, 0);
}

/**
 * @brief Build the target panel shared by PWM, position and speed pages.
 */
static void controls(void)
{
    box(view->container, 612, 0, 380, 426, PANEL, 16);
    const bool pos = m->page == MOTOR_POSITION;
    text(view->container, 636, pos ? 24 : 28, 332,
        m->page == MOTOR_PWM ? "Set output"
        : pos                ? "Target position"
                             : "Target speed",
        &lv_font_montserrat_28, MUTED);
    view->set_label = text(view->container, 636, pos ? 72 : 80, 332, "", &lv_font_montserrat_48, WHITE);
    text(view->container, 636, pos ? 128 : 142, 332,
        pos                    ? "counts"
        : m->page == MOTOR_PWM ? "%"
                               : "counts / 20 ms",
        &lv_font_montserrat_22, MUTED);
    view->target_slider = large_slider(
        view->container, 636, pos ? 186 : 252, 332, pos ? -2000 : -127, pos ? 2000 : 127, target_value(), on_slider);
    const int range_y = pos ? 226 : 306;
    const lv_font_t *range_font = pos ? &lv_font_montserrat_18 : &lv_font_montserrat_22;
    text(view->container, 636, range_y, 124,
        pos                    ? "-2,000"
        : m->page == MOTOR_PWM ? "-100"
                               : "-127",
        range_font, MUTED);
    center_text(view->container, 754, range_y, 96, "0", range_font, MUTED);
    lv_obj_t *max = text(view->container, 844, range_y, 124,
        pos                    ? "+2,000"
        : m->page == MOTOR_PWM ? "+100"
                               : "+127",
        range_font, MUTED);
    lv_obj_set_style_text_align(max, LV_TEXT_ALIGN_RIGHT, 0);
    if (pos) {
        motor_channel_ui_t *c = &m->motor[m->selected];
        box(view->container, 636, 264, 332, 1, LINE, 0);
        text(view->container, 636, 282, 230, "Max motor output", &lv_font_montserrat_22, MUTED);
        view->limit_label = text(view->container, 870, 282, 98, "", &lv_font_montserrat_22, BLUE);
        lv_obj_set_style_text_align(view->limit_label, LV_TEXT_ALIGN_RIGHT, 0);
        update_limit_label();
        view->limit_slider = large_slider(view->container, 636, 346, 332, 1, 127, c->limit, on_limit_slider);
        text(view->container, 636, 390, 70, "1 %", &lv_font_montserrat_18, MUTED);
        lv_obj_t *limit_max = text(view->container, 898, 390, 70, "100 %", &lv_font_montserrat_18, MUTED);
        lv_obj_set_style_text_align(limit_max, LV_TEXT_ALIGN_RIGHT, 0);
    }
    update_target_labels();
}

/**
 * @brief Refresh the selected color without replacing objects during a touch gesture.
 */
static void update_rgb_color(void)
{
    lv_color_t color = lv_color_hsv_to_rgb(view->rgb_hsv.h, view->rgb_hsv.s, view->rgb_hsv.v);
    uint32_t selected = ((uint32_t)color.red << 16) | ((uint32_t)color.green << 8) | color.blue;
    bool changed = selected != m->rgb_color;
    m->rgb_color = selected;
    if (view->rgb_hex_label) {
        lv_label_set_text_fmt(view->rgb_hex_label, "#%06lX", (unsigned long)m->rgb_color);
        lv_obj_set_style_bg_color(view->rgb_swatch, color, 0);
        lv_obj_set_style_bg_grad_color(view->rgb_pad, lv_color_hsv_to_rgb(view->rgb_hsv.h, 100, 100), 0);
        lv_obj_set_pos(view->rgb_cursor, LV_CLAMP(0, view->rgb_hsv.s * 561 / 100 - 10, 541),
            LV_CLAMP(0, (100 - view->rgb_hsv.v) * 151 / 100 - 10, 131));
    }
    if (changed && m->hardware_control) {
        submit(MOTOR_CMD_RGB);
    }
}

static void on_rgb_off(lv_event_t *e)
{
    (void)e;
    if (!editable()) {
        return;
    }
    view->rgb_hsv.v = 0;
    update_rgb_color();
}

/**
 * @brief Map the actual pointer position to saturation and value, including edge drags.
 */
static void on_rgb_pad(lv_event_t *e)
{
    (void)e;
    lv_indev_t *input = lv_indev_active();
    if (!editable() || !input) {
        return;
    }
    lv_point_t point;
    lv_area_t area;
    lv_indev_get_point(input, &point);
    lv_obj_get_coords(view->rgb_pad, &area);
    int x = LV_CLAMP(0, point.x - area.x1, 561);
    int y = LV_CLAMP(0, point.y - area.y1, 151);
    view->rgb_hsv.s = (uint8_t)((x * 100 + 280) / 561);
    view->rgb_hsv.v = (uint8_t)(100 - (y * 100 + 75) / 151);
    update_rgb_color();
}

static void on_rgb_hue(lv_event_t *e)
{
    if (!editable()) {
        return;
    }
    view->rgb_hsv.h = (uint16_t)lv_slider_get_value((lv_obj_t *)lv_event_get_target(e));
    update_rgb_color();
}

static void on_rgb_level(lv_event_t *e)
{
    if (!editable()) {
        return;
    }

    int level = lv_slider_get_value((lv_obj_t *)lv_event_get_target(e));

    /* Round percent to the device scale; keep the current slider alive while dragging. */
    m->rgb_brightness = (uint8_t)((level * 255 + 50) / 100);
    if (view->rgb_level_label) {
        lv_label_set_text_fmt(view->rgb_level_label, "%d %%", level);
    }
    if (m->hardware_control) {
        submit(MOTOR_CMD_RGB);
    }
}

/**
 * @brief Use layered LVGL gradients instead of a full-frame color-picker bitmap.
 */
static void rgb_controls(void)
{
    view->rgb_hsv = lv_color_rgb_to_hsv((m->rgb_color >> 16) & 255, (m->rgb_color >> 8) & 255, m->rgb_color & 255);
    text(view->container, 658, 24, 240, "LED Color", &lv_font_montserrat_22, MUTED);
    lv_obj_t *off = button(view->container, 910, 18, 74, 42, "OFF", LINE, WHITE, on_rgb_off, 0);
    view->rgb_hex_label = text(view->container, 1000, 26, 150, "", &lv_font_montserrat_22, WHITE);
    lv_label_set_text_fmt(view->rgb_hex_label, "#%06lX", (unsigned long)m->rgb_color);
    view->rgb_swatch = box(view->container, 1160, 18, 60, 42, m->rgb_color, 8);

    view->rgb_pad = box(view->container, 658, 80, 562, 152, 0xFFFFFF, 0);
    lv_obj_set_style_bg_grad_color(view->rgb_pad, lv_color_hsv_to_rgb(view->rgb_hsv.h, 100, 100), 0);
    lv_obj_set_style_bg_grad_dir(view->rgb_pad, LV_GRAD_DIR_HOR, 0);
    lv_obj_add_flag(view->rgb_pad, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_obj_t *shade = box(view->rgb_pad, 0, 0, 562, 152, 0, 0);
    lv_obj_set_style_bg_main_opa(shade, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_grad_color(shade, lv_color_black(), 0);
    lv_obj_set_style_bg_grad_opa(shade, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_grad_dir(shade, LV_GRAD_DIR_VER, 0);
    view->rgb_cursor = box(view->rgb_pad, LV_CLAMP(0, view->rgb_hsv.s * 561 / 100 - 10, 541),
        LV_CLAMP(0, (100 - view->rgb_hsv.v) * 151 / 100 - 10, 131), 21, 21, 0, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(view->rgb_cursor, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(view->rgb_cursor, 3, 0);
    lv_obj_set_style_border_color(view->rgb_cursor, lv_color_white(), 0);
    lv_obj_set_style_outline_width(view->rgb_cursor, 1, 0);
    lv_obj_set_style_outline_color(view->rgb_cursor, lv_color_black(), 0);
    lv_obj_add_event_cb(view->rgb_pad, on_rgb_pad, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(view->rgb_pad, on_rgb_pad, LV_EVENT_PRESSING, NULL);

    text(view->container, 658, 250, 240, "Hue", &lv_font_montserrat_22, MUTED);
    const uint32_t spectrum[] = {0xFF0000, 0xFFFF00, 0x00FF00, 0x00FFFF, 0x0000FF, 0xFF00FF, 0xFF0000};
    lv_obj_t *hue_track = box(view->container, 682, 292, 514, 20, LINE, 0);
    for (unsigned i = 0; i < 6; i++) {
        int start = (int)i * 514 / 6;
        int end = (int)(i + 1) * 514 / 6;
        lv_obj_t *segment = box(hue_track, start, 0, end - start, 20, spectrum[i], 0);
        lv_obj_set_style_bg_grad_color(segment, lv_color_hex(spectrum[i + 1]), 0);
        lv_obj_set_style_bg_grad_dir(segment, LV_GRAD_DIR_HOR, 0);
    }
    view->rgb_hue_slider = large_slider(view->container, 682, 292, 514, 0, 359, view->rgb_hsv.h, on_rgb_hue);
    lv_obj_set_style_bg_opa(view->rgb_hue_slider, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(view->rgb_hue_slider, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(view->rgb_hue_slider, LV_OPA_TRANSP, LV_PART_INDICATOR);
    text(view->container, 658, 334, 240, "Brightness", &lv_font_montserrat_22, MUTED);
    view->rgb_level_label = text(view->container, 1100, 334, 120, "", &lv_font_montserrat_22, WHITE);
    lv_obj_set_style_text_align(view->rgb_level_label, LV_TEXT_ALIGN_RIGHT, 0);
    if (m->connected) {
        lv_label_set_text_fmt(view->rgb_level_label, "%u %%", (m->rgb_brightness * 100 + 127) / 255);
    } else {
        lv_label_set_text(view->rgb_level_label, "--");
    }
    large_slider(view->container, 682, 378, 514, 0, 100, (m->rgb_brightness * 100 + 127) / 255, on_rgb_level);
    if (!available()) {
        lv_obj_add_state(off, LV_STATE_DISABLED);
        lv_obj_add_state(view->rgb_pad, LV_STATE_DISABLED);
        lv_obj_set_style_opa(view->rgb_pad, LV_OPA_40, LV_STATE_DISABLED);
        lv_obj_set_style_opa(hue_track, LV_OPA_40, 0);
    }
    if (!m->connected) {
        lv_label_set_text(view->rgb_hex_label, "--");
    }
}

/**
 * @brief Group module identity, persistent address setting and RGB control in one page.
 */
static void device_page(void)
{
    box(view->container, 32, 0, 574, 426, PANEL, 16);
    box(view->container, 630, 0, 618, 426, PANEL, 16);
    text(view->container, 60, 24, 500, "Device information", &lv_font_montserrat_22, MUTED);
    text(view->container, 60, 67, 510,
        m->connected ? (m->channels == 1 ? "EncoderMotor Driver" : "4EncoderMotor Driver") : "No device detected",
        &lv_font_montserrat_28, WHITE);
    const char *keys[] = {"Firmware", "I2C address", "Default addresses"};
    char vals[4][40];
    snprintf(vals[0], 40, m->connected ? "V%u" : "--", m->version);
    snprintf(vals[1], 40, m->connected ? "0x%02X" : "--", m->address);
    snprintf(vals[2], 40, "0x2F / 0x24");
    for (int i = 0; i < 3; i++) {
        text(view->container, 60, 120 + i * 49, 240, keys[i], &lv_font_montserrat_22, MUTED);
        text(view->container, 310, 120 + i * 49, i == 0 ? 75 : 270, vals[i], &lv_font_montserrat_22, WHITE);
        if (i < 2) {
            box(view->container, 60, 157 + i * 49, 518, 1, 0x232C36, 0);
        }
    }
    text(view->container, 400, 120, 120, "Channels", &lv_font_montserrat_22, MUTED);
    text(view->container, 540, 120, 38, m->connected ? (m->channels == 1 ? "1" : "4") : "--", &lv_font_montserrat_22,
        WHITE);
    box(view->container, 60, 258, 518, 1, LINE, 0);
    text(view->container, 60, 277, 240, "Change address", &lv_font_montserrat_22, MUTED);
    char a[16];
    snprintf(a, sizeof(a), "0x%02X", m->pending_address);
    text(view->container, 60, 311, 150, a, &lv_font_montserrat_40, WHITE);
    text(view->container, 310, 280, 268, "0x08 - 0x77", &lv_font_montserrat_18, MUTED);

    lv_obj_t *minus = button(view->container, 230, 311, 68, 44, "-", LINE, WHITE, on_address, -1);
    lv_obj_t *plus = button(view->container, 310, 311, 68, 44, "+", LINE, WHITE, on_address, 1);
    lv_obj_t *apply = button(view->container, 394, 311, 184, 44, "APPLY", 0x213344, BLUE, on_address, 0);

    rgb_controls();

    if (m->preview) {
        button(view->container, 60, 370, 156, 40, "1 CHANNEL", m->connected && m->channels == 1 ? 0x233C4E : LINE,
            m->connected && m->channels == 1 ? BLUE : WHITE, on_profile, 1);
        button(view->container, 230, 370, 172, 40, "4 CHANNELS", m->connected && m->channels == 4 ? 0x233C4E : LINE,
            m->connected && m->channels == 4 ? BLUE : WHITE, on_profile, 4);
        button(view->container, 416, 370, 162, 40, "OFFLINE", !m->connected ? 0x233C4E : LINE,
            !m->connected ? BLUE : WHITE, on_profile, 0);
    } else if (m->device_count > 1) {
        button(view->container, 60, 370, 518, 40, "NEXT DEVICE", LINE, BLUE, on_next_device, 0);
    }
    if (!available()) {
        lv_obj_add_state(minus, LV_STATE_DISABLED);
        lv_obj_add_state(plus, LV_STATE_DISABLED);
        lv_obj_add_state(apply, LV_STATE_DISABLED);
    }
}

/**
 * @brief Block pending touches without the disabled-state fade.
 */
static void set_control_state(lv_obj_t *object, bool enabled)
{
    if (enabled) {
        lv_obj_remove_state(object, LV_STATE_DISABLED);
    } else {
        lv_obj_add_state(object, LV_STATE_DISABLED);
    }
    if (enabled && !m->mode_switch_pending) {
        lv_obj_add_flag(object, LV_OBJ_FLAG_CLICKABLE);
    } else {
        lv_obj_remove_flag(object, LV_OBJ_FLAG_CLICKABLE);
    }
}

static void block_content_touches(lv_obj_t *parent)
{
    lv_obj_remove_flag(parent, LV_OBJ_FLAG_CLICKABLE);
    for (uint32_t i = 0; i < lv_obj_get_child_count(parent); i++) {
        block_content_touches(lv_obj_get_child(parent, i));
    }
}

static void select_button(lv_obj_t *object, bool active, uint32_t inactive, uint32_t text_color)
{
    lv_obj_set_style_bg_color(object, lv_color_hex(active ? BLUE : inactive), 0);
    lv_obj_set_style_bg_color(object, lv_color_hex(active ? 0xB0E3FF : 0x2A3D4C), LV_STATE_PRESSED);
    lv_obj_set_style_border_color(object, lv_color_hex(active ? 0xBEE9FF : 0x2B3945), 0);
    lv_obj_set_style_border_width(object, active ? 2 : 1, 0);
    lv_obj_set_style_text_color(lv_obj_get_child(object, 0), lv_color_hex(active ? BG : text_color), 0);
}

static void create_navigation(void)
{
    nav_buttons[MOTOR_DEVICE] = button(nav, 32, 0, 208, 54, "DEVICE", PANEL, WHITE, on_nav, MOTOR_DEVICE);
    box(nav, 252, 9, 1, 36, LINE, 0);
    lv_obj_t *mode_label = text(nav, 264, 15, 208, "Control Mode", &lv_font_montserrat_22, MUTED);
    lv_obj_set_style_text_align(mode_label, LV_TEXT_ALIGN_RIGHT, 0);
    const int mode_left = 488;
    const int mode_right = 1248;
    const int mode_gap = 16;
    const int mode_count = sizeof(pages) / sizeof(pages[0]);
    const int mode_total_width = mode_right - mode_left - (mode_count - 1) * mode_gap;
    for (int i = 0; i < mode_count; i++) {
        int x = mode_left + i * mode_total_width / mode_count + i * mode_gap;
        int width = (i + 1) * mode_total_width / mode_count - i * mode_total_width / mode_count;
        nav_buttons[i] = button(nav, x, 0, width, 54, pages[i], PANEL, MUTED, on_nav, i);
    }
    for (int i = 0; i <= MOTOR_DEVICE; i++) {
        lv_obj_set_style_text_font(lv_obj_get_child(nav_buttons[i], 0), &lv_font_montserrat_28, 0);
    }
}

static void create_footer(void)
{
    text(footer, 32, 0, 170, "Voltage", &lv_font_montserrat_22, MUTED);
    voltage_label = text(footer, 32, 23, 174, "", &lv_font_montserrat_28, WHITE);
    box(footer, 224, 3, 1, 46, LINE, 0);
    text(footer, 252, 0, 184, "Current", &lv_font_montserrat_22, MUTED);
    current_label = text(footer, 252, 23, 184, "", &lv_font_montserrat_28, WHITE);
    notice_label = text(footer, 32, 57, 1216, "", &lv_font_montserrat_18, MUTED);
    stop_button = button(footer, 772, 0, 222, 64, LV_SYMBOL_STOP "  STOP ALL", 0x38222B, RED, on_run, 0);
    run_button = button(footer, 1010, 0, 238, 64, LV_SYMBOL_PLAY "  START", GREEN, BG, on_run, 1);
    lv_obj_set_style_text_font(lv_obj_get_child(stop_button, 0), &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_font(lv_obj_get_child(run_button, 0), &lv_font_montserrat_28, 0);
}

static void update_common_regions(void)
{
    for (int i = 0; i <= MOTOR_DEVICE; i++) {
        select_button(nav_buttons[i], m->page == (motor_page_t)i, PANEL, i == MOTOR_DEVICE ? WHITE : MUTED);
        set_control_state(nav_buttons[i], true);
    }
    lv_label_set_text_fmt(motor_count, "%02u", m->connected ? m->channels : 0);
    for (int i = 0; i < 4; i++) {
        bool enabled = m->connected && i < m->channels;
        bool selected = enabled && m->selected == i;
        select_button(motor_buttons[i], selected, 0x171F28, MUTED);
        set_control_state(motor_buttons[i], enabled);
        lv_obj_set_style_text_font(motor_names[i], selected ? &lv_font_montserrat_36 : &lv_font_montserrat_28, 0);
        lv_obj_set_style_text_color(motor_names[i], lv_color_hex(selected ? BG : MUTED), 0);
        lv_obj_set_style_bg_opa(motor_marks[i], selected ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
        lv_obj_set_style_bg_color(motor_tracks[i], lv_color_hex(selected ? 0x5495B8 : LINE), 0);
        lv_obj_set_style_bg_color(channel_fill[i], lv_color_hex(selected ? BG : 0x536F87), 0);
    }
    lv_label_set_text(device_label, m->io_error ? "Connection lost" : !m->connected ? "No device connected" : "");
    const bool routine = !strcmp(m->notice, "UI preview / simulated device and feedback") ||
                         !strcmp(m->notice, "Read-only feedback / controls disabled");
    lv_label_set_text(notice_label, routine ? "" : m->notice);
    lv_obj_set_style_text_color(notice_label, lv_color_hex(m->io_error ? RED : MUTED), 0);
    lv_label_set_text(lv_obj_get_child(run_button, 0),
        m->motor[m->selected].running ? LV_SYMBOL_STOP "  STOP" : LV_SYMBOL_PLAY "  START");
    set_control_state(stop_button, available());
    set_control_state(run_button, available() && m->page != MOTOR_DEVICE);
}

/**
 * @brief Switch only changing child pages; keep navigation, motors, gauge and footer objects alive.
 */
static void refresh_view(void)
{
    update_common_regions();
    if (m->mode_switch_pending) {
        block_content_touches(view->container);
        motor_ui_refresh_feedback();
        return;
    }
    view = &content_pages[m->page];
    bool created = !view->container;
    if (created) {
        const bool device = m->page == MOTOR_DEVICE;
        view->container = box(body, device ? 0 : 256, 0, device ? 1280 : 1024, 426, BG, 0);
        lv_obj_set_style_bg_opa(view->container, LV_OPA_TRANSP, 0);
        lv_obj_add_flag(view->container, LV_OBJ_FLAG_HIDDEN);
    }
    if (m->page == MOTOR_DEVICE) {
        /* Device identity and address controls can change after discovery or profile selection. */
        lv_obj_t *container = view->container;
        lv_obj_clean(container);
        memset(view, 0, sizeof(*view));
        view->container = container;
        device_page();
    } else {
        int max = m->page == MOTOR_PWM ? 100 : m->page == MOTOR_POSITION ? 2000 : 127;
        lv_arc_set_range(gauge, 0, max);
        lv_obj_set_style_arc_color(gauge, lv_color_hex(m->page == MOTOR_SPEED ? GREEN : BLUE), LV_PART_INDICATOR);
        if (created) {
            draw_gauge(max, m->page == MOTOR_PWM ? "%" : m->page == MOTOR_POSITION ? "counts" : "counts / 20 ms");
            controls();
        }
        lv_slider_set_value(view->target_slider, target_value(), LV_ANIM_OFF);
        set_control_state(view->target_slider, available());
        if (view->limit_slider) {
            lv_slider_set_value(view->limit_slider, m->motor[m->selected].limit, LV_ANIM_OFF);
            set_control_state(view->limit_slider, available());
            update_limit_label();
        }
    }
    motor_ui_refresh_feedback();
    for (int i = 0; i <= MOTOR_DEVICE; i++) {
        if (content_pages[i].container) {
            lv_obj_set_flag(content_pages[i].container, LV_OBJ_FLAG_HIDDEN, i != (int)m->page);
        }
    }
    lv_obj_set_flag(motor_panel, LV_OBJ_FLAG_HIDDEN, m->page == MOTOR_DEVICE);
    lv_obj_set_flag(gauge_panel, LV_OBJ_FLAG_HIDDEN, m->page == MOTOR_DEVICE);
}

/**
 * @brief Create fixed header regions, then build the selected page and footer.
 */
void motor_ui_init(motor_ui_model_t *model)
{
    m = model;
    root = lv_obj_create(NULL);
    lv_obj_remove_style_all(root);
    lv_obj_set_size(root, 1280, 720);
    lv_obj_set_style_bg_color(root, lv_color_hex(BG), 0);
    lv_obj_set_style_bg_opa(root, LV_OPA_COVER, 0);
    lv_obj_remove_flag(root, LV_OBJ_FLAG_SCROLLABLE);
    text(root, 96, 24, 600, "ENCODER MOTOR", &lv_font_montserrat_36, WHITE);
    text(root, 96, 60, 300, "", &lv_font_montserrat_18, MUTED);
    device_label = text(root, 690, 36, 330, "", &lv_font_montserrat_22, RED);
    nav = box(root, 0, 100, 1280, 54, BG, 0);
    body = box(root, 0, 174, 1280, 426, BG, 0);
    footer = box(root, 0, 630, 1280, 80, BG, 0);
    box(root, 1040, 24, 208, 40, PANEL, 20);
    status_dot = box(root, 1058, 41, 6, 6, BLUE, LV_RADIUS_CIRCLE);
    status_label = text(root, 1080, 32, 156,
        m->preview            ? "SIMULATION"
        : m->hardware_control ? "LIVE DEVICE"
                              : "READ ONLY",
        &lv_font_montserrat_22, BLUE);

    lv_obj_t *icon = box(root, 32, 22, 44, 44, 0x1F3545, 12);
    box(icon, 10, 11, 23, 22, 0x79C8FA, 4);
    box(icon, 15, 7, 13, 5, 0x79C8FA, 0);
    box(icon, 20, 33, 3, 6, 0x79C8FA, 0);
    box(icon, 15, 16, 3, 11, 0x1F3545, 0);
    box(icon, 24, 16, 3, 11, 0x1F3545, 0);
    box(root, 32, 613, 1216, 1, 0x202B35, 0);
    create_navigation();
    create_footer();
    channel_list();
    gauge_panel = box(body, 256, 0, 592, 426, BG, 0);
    lv_obj_set_style_bg_opa(gauge_panel, LV_OPA_TRANSP, 0);
    create_gauge();
    refresh_view();
    lv_screen_load(root);
}

void motor_ui_refresh(void)
{
    if (root) {
        refresh_view();
    }
}

/**
 * @brief Refresh existing labels and bars so polling does not interrupt dragging.
 */
void motor_ui_refresh_feedback(void)
{
    if (!root || !view) {
        return;
    }

    update_device_status();
    update_target_labels();
    motor_channel_ui_t *c = &m->motor[m->selected];
    if (view->secondary_label && view->other_label) {
        if (!m->connected) {
            lv_label_set_text(view->secondary_label, "--");
            lv_label_set_text(view->other_label, "--");
        } else if (m->page == MOTOR_POSITION) {
            lv_label_set_text_fmt(view->secondary_label, "%+d ct / 20 ms", c->speed);
            if (m->preview || m->hardware_control) {
                lv_label_set_text_fmt(view->other_label, "%+lld counts", (long long)c->position_target - c->encoder);
            } else {
                lv_label_set_text(view->other_label, "--");
            }
        } else {
            lv_label_set_text_fmt(view->secondary_label, "%+ld counts", (long)c->encoder);
            if (m->page == MOTOR_SPEED) {
                lv_label_set_text_fmt(view->other_label, "%+d %%", c->pwm * 100 / 127);
            } else {
                lv_label_set_text_fmt(view->other_label, "%+d ct / 20 ms", c->speed);
            }
        }
    }
    if (voltage_label && current_label) {
        if (m->connected) {
            lv_label_set_text_fmt(voltage_label, "%d.%02d V", m->millivolts / 1000, (m->millivolts % 1000) / 10);
            lv_label_set_text_fmt(current_label, "%d.%02d A", m->milliamps / 1000, abs((m->milliamps % 1000) / 10));
        } else {
            lv_label_set_text(voltage_label, "-- V");
            lv_label_set_text(current_label, "-- A");
        }
    }
}
