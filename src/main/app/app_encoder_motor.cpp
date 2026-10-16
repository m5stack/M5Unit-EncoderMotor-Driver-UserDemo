/**
 * @file app_encoder_motor.cpp
 * @brief Tab5 display setup and the single owner of motor I2C transactions.
 *
 * LVGL callbacks enqueue value snapshots. This app loop executes commands and
 * publishes feedback under the display lock; no I2C transfer holds that lock.
 */
#include "app_encoder_motor.h"
#include "app_motor_ui.h"
#include "bsp/esp-bsp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "m5_unit_motor_driver.h"
#include "driver/usb_serial_jtag.h"
#include <cstdio>
#include <cstring>

/**
 * @brief Shared model is protected by the BSP display lock; the queue copies commands.
 */
static motor_ui_model_t model;
static QueueHandle_t commands;
static const char *TAG = "motor_live";

/**
 * @brief Give stop requests priority and discard pending target changes before stopping.
 */
static bool enqueue(const motor_ui_command_t *cmd, void *)
{
    if (!commands) {
        return false;
    }

    if (cmd->kind == MOTOR_CMD_STOP_ALL || cmd->kind == MOTOR_CMD_STOP || cmd->kind == MOTOR_CMD_MODE) {
        xQueueReset(commands);

        return xQueueSendToFront(commands, cmd, 0) == pdTRUE;
    }
    return xQueueSendToBack(commands, cmd, 0) == pdTRUE;
}

/**
 * @brief Attempt all three stop writes and preserve the first failure for the caller.
 */
static esp_err_t stop_channel(m5_motor_handle_t h, uint8_t ch)
{
    esp_err_t first = m5_motor_set_speed_target(h, ch, 0);
    esp_err_t e = m5_motor_set_mode(h, ch, M5_MOTOR_OPEN_LOOP);
    if (first == ESP_OK) {
        first = e;
    }
    e = m5_motor_set_pwm(h, ch, 0);

    return first == ESP_OK ? e : first;
}

/**
 * @brief Apply one snapshotted command; start a new mode from a stopped channel.
 */
static esp_err_t execute(m5_motor_handle_t h, const motor_ui_command_t &c)
{
    if (c.kind == MOTOR_CMD_RGB) {
        esp_err_t e = m5_motor_set_rgb(h, c.rgb_color, c.rgb_brightness);
        m5_motor_rgb_t read = {};
        if (e == ESP_OK) {
            e = m5_motor_get_rgb(h, &read);
        }
        return e == ESP_OK && (read.color != c.rgb_color || read.brightness != c.rgb_brightness)
                   ? ESP_ERR_INVALID_RESPONSE
                   : e;
    }
    if (c.kind == MOTOR_CMD_STOP_ALL || c.kind == MOTOR_CMD_MODE) {
        return m5_motor_stop_all(h);
    }

    if (c.kind == MOTOR_CMD_STOP) {
        return stop_channel(h, c.channel);
    }

    /* Stop outputs before the persistent address update changes the I2C handle. */
    if (c.kind == MOTOR_CMD_ADDRESS) {
        esp_err_t e = m5_motor_stop_all(h);

        return e == ESP_OK ? m5_motor_set_address(h, c.address) : e;
    }
    esp_err_t e = ESP_OK;
    if (c.kind == MOTOR_CMD_START) {
        e = stop_channel(h, c.channel);
    }
    if (e != ESP_OK) {
        return e;
    }

    if (c.page == MOTOR_PWM) {
        e = m5_motor_set_pwm(h, c.channel, c.pwm);
    } else if (c.page == MOTOR_SPEED) {
        e = m5_motor_set_speed_target(h, c.channel, c.speed);
        if (e == ESP_OK && c.kind == MOTOR_CMD_START) {
            e = m5_motor_set_mode(h, c.channel, M5_MOTOR_SPEED);
        }
    } else if (c.page == MOTOR_POSITION) {
        e = m5_motor_set_position_target(h, c.channel, c.position);
        if (e == ESP_OK) {
            e = m5_motor_set_position_limit(h, c.channel, c.limit);
        }
        if (e == ESP_OK && c.kind == MOTOR_CMD_START) {
            e = m5_motor_set_mode(h, c.channel, M5_MOTOR_POSITION);
        }
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    /* A failed mode or target write must not leave a partially started command unchecked. */
    if (e != ESP_OK) {
        stop_channel(h, c.channel);
    }
    return e;
}

/**
 * @brief Log actual register feedback, keeping speed in encoder counts per 20ms.
 */
static void log_feedback(const m5_motor_telemetry_t &f, esp_err_t e)
{
    ESP_LOGI(TAG, "feedback=%s VIN=%u mV I=%.4f A PWM=[%d,%d,%d,%d] SPEED=[%d,%d,%d,%d] ENC=[%ld,%ld,%ld,%ld]",
        esp_err_to_name(e), f.vin_mv, (double)f.current_a, f.pwm[0], f.pwm[1], f.pwm[2], f.pwm[3], f.speed[0],
        f.speed[1], f.speed[2], f.speed[3], (long)f.encoder[0], (long)f.encoder[1], (long)f.encoder[2],
        (long)f.encoder[3]);
}

/**
 * @brief Let an explicitly queued stop interrupt the bounded motor diagnostic.
 */
static bool test_cancelled()
{
    motor_ui_command_t c = {};

    return xQueuePeek(commands, &c, 0) == pdTRUE &&
           (c.kind == MOTOR_CMD_STOP_ALL || c.kind == MOTOR_CMD_STOP || c.kind == MOTOR_CMD_MODE);
}

/**
 * @brief Verify RGB register round trips and restore the pre-test color and brightness.
 */
static esp_err_t diagnostic_rgb(m5_motor_handle_t h)
{
    m5_motor_rgb_t saved = {};
    esp_err_t e = m5_motor_get_rgb(h, &saved);
    if (e != ESP_OK) {
        return e;
    }

    const uint32_t colors[] = {0xFF0000, 0x00FF00, 0x0000FF, 0xFFFFFF};
    ESP_LOGI(TAG, "RGB TEST BEGIN saved=%06lX brightness=%u", (unsigned long)saved.color, saved.brightness);
    for (uint32_t color : colors) {
        e = m5_motor_set_rgb(h, color, 32);
        if (e != ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
        m5_motor_rgb_t actual = {};
        e = m5_motor_get_rgb(h, &actual);
        if (e == ESP_OK && (actual.color != color || actual.brightness != 32)) {
            e = ESP_ERR_INVALID_RESPONSE;
        }
        ESP_LOGI(TAG, "RGB TEST requested=%06lX read=%06lX brightness=%u result=%s", (unsigned long)color,
            (unsigned long)actual.color, actual.brightness, esp_err_to_name(e));
        if (e != ESP_OK) {
            break;
        }
    }

    /* Always attempt restoration, including when an intermediate test color failed. */
    esp_err_t restore = m5_motor_set_rgb(h, saved.color, saved.brightness);
    m5_motor_rgb_t actual = {};
    if (restore == ESP_OK) {
        restore = m5_motor_get_rgb(h, &actual);
    }
    if (restore == ESP_OK && (actual.color != saved.color || actual.brightness != saved.brightness)) {
        restore = ESP_ERR_INVALID_RESPONSE;
    }
    if (e == ESP_OK) {
        e = restore;
    }
    ESP_LOGI(TAG, "RGB TEST END result=%s restore=%s color=%06lX brightness=%u", esp_err_to_name(e),
        esp_err_to_name(restore), (unsigned long)actual.color, actual.brightness);

    return e;
}

/**
 * @brief Explicit USB diagnostic only: bounded +/-8 raw PWM, no Flash or encoder reset.
 */
static esp_err_t diagnostic(m5_motor_handle_t h)
{
    esp_err_t e = m5_motor_stop_all(h);
    ESP_LOGI(TAG, "TEST BEGIN channels=%u raw_pwm=+/-8 pulse_ms=150 plus I2C read latency", m5_motor_channels(h));
    for (uint8_t ch = 0; ch < m5_motor_channels(h) && e == ESP_OK; ch++) {
        m5_motor_control_t saved = {};
        m5_motor_telemetry_t f = {};
        e = m5_motor_read_control(h, ch, &saved);
        if (e != ESP_OK) {
            break;
        }
        for (int pass = 0; pass < 2; pass++) {
            int sign = pass == 0 ? 1 : -1;
            if (test_cancelled()) {
                e = ESP_ERR_INVALID_STATE;
                break;
            }
            e = m5_motor_set_pwm(h, ch, sign * 8);
            if (e != ESP_OK) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(150));
            e = m5_motor_read_telemetry(h, &f);
            ESP_LOGI(TAG, "TEST M%u request=%d read=%d speed=%d encoder=%ld io=%s", ch + 1, sign * 8, f.pwm[ch],
                f.speed[ch], (long)f.encoder[ch], esp_err_to_name(e));
            esp_err_t stop = stop_channel(h, ch);
            if (e == ESP_OK && f.pwm[ch] != sign * 8) {
                e = ESP_ERR_INVALID_RESPONSE;
            }
            if (e == ESP_OK) {
                e = stop;
            }
            if (e != ESP_OK) {
                break;
            }
        }

        /* Use current position and a low output limit; this checks registers, not motion tracking. */
        if (e == ESP_OK) {
            e = m5_motor_set_position_target(h, ch, f.encoder[ch]);
            if (e == ESP_OK) {
                e = m5_motor_set_position_limit(h, ch, 8);
            }
            if (e == ESP_OK) {
                e = m5_motor_set_mode(h, ch, M5_MOTOR_POSITION);
            }
            m5_motor_control_t read = {};
            if (e == ESP_OK) {
                e = m5_motor_read_control(h, ch, &read);
            }
            if (e == ESP_OK &&
                (read.mode != M5_MOTOR_POSITION || read.position_target != f.encoder[ch] || read.position_limit != 8)) {
                e = ESP_ERR_INVALID_RESPONSE;
            }
            ESP_LOGI(TAG, "TEST M%u position=current readback=%s", ch + 1, esp_err_to_name(e));
            esp_err_t stop = stop_channel(h, ch);
            if (e == ESP_OK) {
                e = stop;
            }
            stop = m5_motor_set_position_target(h, ch, saved.position_target);
            if (e == ESP_OK) {
                e = stop;
            }
            stop = m5_motor_set_position_limit(h, ch, saved.position_limit);
            if (e == ESP_OK) {
                e = stop;
            }
        }
        if (e == ESP_OK) {
            e = m5_motor_set_speed_target(h, ch, 0);
            if (e == ESP_OK) {
                e = m5_motor_set_mode(h, ch, M5_MOTOR_SPEED);
            }
            m5_motor_control_t read = {};
            if (e == ESP_OK) {
                e = m5_motor_read_control(h, ch, &read);
            }
            if (e == ESP_OK && (read.mode != M5_MOTOR_SPEED || read.speed_target != 0)) {
                e = ESP_ERR_INVALID_RESPONSE;
            }
            ESP_LOGI(TAG, "TEST M%u speed=0 readback=%s", ch + 1, esp_err_to_name(e));
        }
        esp_err_t stop = stop_channel(h, ch);
        if (e == ESP_OK) {
            e = stop;
        }
    }
    esp_err_t stop = m5_motor_stop_all(h);
    if (e == ESP_OK) {
        e = stop;
    }
    m5_motor_telemetry_t f = {};
    esp_err_t read = m5_motor_read_telemetry(h, &f);
    if (e == ESP_OK) {
        e = read;
    }
    for (uint8_t ch = 0; ch < m5_motor_channels(h) && e == ESP_OK; ch++) {
        m5_motor_control_t c = {};
        e = m5_motor_read_control(h, ch, &c);
        if (e == ESP_OK && (f.pwm[ch] != 0 || c.mode != M5_MOTOR_OPEN_LOOP || c.speed_target != 0)) {
            e = ESP_ERR_INVALID_RESPONSE;
        }
    }
    log_feedback(f, read);
    ESP_LOGI(TAG, "TEST END result=%s stop=%s", esp_err_to_name(e), esp_err_to_name(stop));

    return e;
}

/**
 * @brief Start LVGL, then service discovery, explicit commands and periodic feedback.
 */
void app_encoder_motor(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    lv_display_t *display = bsp_display_start();
    if (!display) {
        ESP_LOGE(TAG, "Display initialization failed");

        return;
    }

    /* Build the shared landscape UI under the BSP display lock. */
    bsp_display_brightness_set(80);
    bsp_display_lock(0);
    lv_display_rotation_t native = lv_display_get_rotation(display);
    bsp_display_rotate(display, native == LV_DISPLAY_ROTATION_180 ? LV_DISPLAY_ROTATION_270 : LV_DISPLAY_ROTATION_90);
    motor_ui_demo_init(&model, 4);
#if CONFIG_TAB5_MOTOR_READ_ONLY_PROBE || CONFIG_TAB5_MOTOR_HARDWARE_CONTROL
    model.preview = false;
    model.connected = false;
    model.channels = 0;
    model.millivolts = model.milliamps = 0;
    std::memset(model.motor, 0, sizeof(model.motor));
    std::snprintf(model.notice, sizeof(model.notice), "Connecting...");
#endif
#if CONFIG_TAB5_MOTOR_HARDWARE_CONTROL
    model.hardware_control = true;
    commands = xQueueCreate(8, sizeof(motor_ui_command_t));
    motor_ui_set_command_handler(enqueue, nullptr);
#endif
    motor_ui_init(&model);
    bsp_display_unlock();
    ESP_LOGI(TAG, "1280x720 motor UI / LVGL %d.%d.%d / %s", LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR, LVGL_VERSION_PATCH,
        model.preview            ? "SIMULATION"
        : model.hardware_control ? "LIVE DEVICE"
                                 : "READ ONLY");
#if CONFIG_TAB5_MOTOR_READ_ONLY_PROBE || CONFIG_TAB5_MOTOR_HARDWARE_CONTROL

    /* PORT.A uses a dedicated master bus: SDA G53, SCL G54. */
    i2c_master_bus_config_t cfg = {};
    cfg.i2c_port = I2C_NUM_1;
    cfg.sda_io_num = GPIO_NUM_53;
    cfg.scl_io_num = GPIO_NUM_54;
    cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt = 7;
    cfg.flags.enable_internal_pullup = true;
    i2c_master_bus_handle_t bus = nullptr;
    esp_err_t init = i2c_new_master_bus(&cfg, &bus);
    ESP_LOGI(TAG, "I2C1 master SDA=53 SCL=54 clock=100000 init=%s", esp_err_to_name(init));
    bool usb = false;
    if (model.hardware_control && commands) {
        usb_serial_jtag_driver_config_t uc = {
            .tx_buffer_size = 1024,
            .rx_buffer_size = 256,
        };
        usb = usb_serial_jtag_driver_install(&uc) == ESP_OK;
        ESP_LOGI(TAG, "USB diagnostics=%s; commands: test pwm / test rgb / stop", usb ? "ready" : "unavailable");
    }
    m5_motor_handle_t devices[2] = {};
    size_t count = 0;
    unsigned active = 0, polls = 0;
    TickType_t next_poll = 0, next_discover = 0;
    char serial[40] = {};
    unsigned serial_len = 0;
    while (true) {
        TickType_t now = xTaskGetTickCount();

        /* Probe default addresses every two seconds while disconnected; never auto-run diagnostics. */
        if (!count && (int32_t)(now - next_discover) >= 0) {
            esp_err_t e = init;
            if (e == ESP_OK) {
                e = m5_motor_discover(bus, devices, &count);
            }
            ESP_LOGI(TAG, "discovery=%s count=%u", esp_err_to_name(e), (unsigned)count);
            for (size_t i = 0; i < count && e == ESP_OK; i++) {
                uint8_t uid[12] = {}, fw = 0;
                e = m5_motor_get_uid(devices[i], uid);
                if (e == ESP_OK) {
                    e = m5_motor_get_version(devices[i], &fw);
                }
                ESP_LOGI(TAG,
                    "addr=0x%02X channels=%u fw=%u uid=%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X io=%s",
                    m5_motor_address(devices[i]), m5_motor_channels(devices[i]), fw, uid[0], uid[1], uid[2], uid[3],
                    uid[4], uid[5], uid[6], uid[7], uid[8], uid[9], uid[10], uid[11], esp_err_to_name(e));
                m5_motor_rgb_t rgb = {};
                if (e == ESP_OK) {
                    e = m5_motor_get_rgb(devices[i], &rgb);
                }
                ESP_LOGI(TAG, "RGB read color=%06lX brightness=%u io=%s", (unsigned long)rgb.color, rgb.brightness,
                    esp_err_to_name(e));
                if (i == 0 && e == ESP_OK) {
                    bsp_display_lock(0);
                    model.rgb_color = rgb.color;
                    model.rgb_brightness = rgb.brightness;
                    bsp_display_unlock();
                }

                /* After connection or reconnection, begin with all outputs stopped. */
                if (model.hardware_control && e == ESP_OK) {
                    e = m5_motor_stop_all(devices[i]);
                }
                for (uint8_t ch = 0; ch < m5_motor_channels(devices[i]) && e == ESP_OK; ch++) {
                    m5_motor_control_t c = {};
                    e = m5_motor_read_control(devices[i], ch, &c);
                    ESP_LOGI(TAG,
                        "M%u config mode=%d position=%ld limit=%u speed_target=%d posPID=%u,%u,%u "
                        "speedPID=%u,%u,%u io=%s",
                        ch + 1, c.mode, (long)c.position_target, c.position_limit, c.speed_target, c.position_pid[0],
                        c.position_pid[1], c.position_pid[2], c.speed_pid[0], c.speed_pid[1], c.speed_pid[2],
                        esp_err_to_name(e));
                    if (i == 0 && e == ESP_OK) {
                        bsp_display_lock(0);
                        model.motor[ch].position_target = c.position_target;
                        model.motor[ch].limit = c.position_limit;
                        model.motor[ch].speed_target = c.speed_target;
                        model.motor[ch].pwm_target = 0;
                        model.motor[ch].running = false;
                        bsp_display_unlock();
                    }
                }
            }
            bsp_display_lock(0);
            model.device_count = count;
            model.device_index = 0;
            active = 0;
            model.io_error = e != ESP_OK;
            std::snprintf(model.notice, sizeof(model.notice), "%s",
                e != ESP_OK ? esp_err_to_name(e)
                : count     ? ""
                            : "No module at 0x24 or 0x2F");
            motor_ui_refresh();
            bsp_display_unlock();
            if (e != ESP_OK) {
                for (size_t i = 0; i < count; i++) {
                    if (model.hardware_control) {
                        m5_motor_stop_all(devices[i]);
                    }
                    m5_motor_delete(&devices[i]);
                }
                count = 0;
            }
            next_discover = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
            next_poll = 0;
        }

        /* USB lines request diagnostics explicitly; parsing itself never accesses I2C. */
        if (usb) {
            char b[32];
            int n = usb_serial_jtag_read_bytes(b, sizeof(b), 0);
            for (int i = 0; i < n; i++) {
                if (b[i] == '\n' || b[i] == '\r') {
                    serial[serial_len] = 0;
                    motor_ui_command_t c = {};
                    c.device_index = active;
                    if (!std::strcmp(serial, "test pwm") && count) {
                        c.kind = MOTOR_CMD_TEST;
                        enqueue(&c, nullptr);
                    } else if (!std::strcmp(serial, "test rgb") && count) {
                        c.kind = MOTOR_CMD_RGB_TEST;
                        enqueue(&c, nullptr);
                    } else if (!std::strcmp(serial, "stop") && count) {
                        c.kind = MOTOR_CMD_STOP_ALL;
                        enqueue(&c, nullptr);
                    }
                    serial_len = 0;
                } else if (serial_len < sizeof(serial) - 1) {
                    serial[serial_len++] = b[i];
                } else {
                    serial_len = 0;
                }
            }
        }

        /* Drain one queued command before polling so stop and target actions stay responsive. */
        motor_ui_command_t command = {};
        if (commands && xQueueReceive(commands, &command, 0) == pdTRUE) {
            esp_err_t e = ESP_ERR_INVALID_STATE;
            if (count && command.device_index == active) {
                e = command.kind == MOTOR_CMD_TEST       ? diagnostic(devices[active])
                    : command.kind == MOTOR_CMD_RGB_TEST ? diagnostic_rgb(devices[active])
                                                         : execute(devices[active], command);
            }
            ESP_LOGI(TAG, "command=%d M%u page=%d pwm=%d speed=%d position=%ld result=%s", command.kind,
                command.channel + 1, command.page, command.pwm, command.speed, (long)command.position,
                esp_err_to_name(e));
            if (e != ESP_OK && count) {
                m5_motor_stop_all(devices[active]);
            }

            /* Publish acknowledgement state only after the corresponding I2C operations finish. */
            bsp_display_lock(0);
            if (command.kind == MOTOR_CMD_MODE) {
                motor_ui_finish_mode_switch(&model, command.page, e == ESP_OK);
                if (e != ESP_OK) {
                    model.io_error = true;
                }
            }
            if (command.kind == MOTOR_CMD_STOP_ALL || command.kind == MOTOR_CMD_STOP) {
                model.mode_switch_pending = false;
            }
            if (e == ESP_OK) {
                if (command.kind == MOTOR_CMD_START) {
                    model.motor[command.channel].running = true;
                    model.motor[command.channel].active_page = command.page;
                }
                if (command.kind == MOTOR_CMD_STOP) {
                    motor_ui_reset_motion_targets(&model, command.channel);
                }
                if (command.kind == MOTOR_CMD_STOP_ALL) {
                    for (uint8_t ch = 0; ch < 4; ch++) {
                        motor_ui_reset_motion_targets(&model, ch);
                    }
                }
                if (command.kind == MOTOR_CMD_TEST || command.kind == MOTOR_CMD_ADDRESS) {
                    for (auto &c : model.motor) {
                        c.running = false;
                        c.pwm_target = 0;
                    }
                }
                if (command.kind == MOTOR_CMD_ADDRESS) {
                    model.address = model.pending_address = m5_motor_address(devices[active]);
                }
            } else {
                for (auto &c : model.motor) {
                    c.running = false;
                }
            }
            std::snprintf(model.notice, sizeof(model.notice), "%s",
                e == ESP_OK ? command.kind == MOTOR_CMD_TEST       ? "Diagnostic complete. All outputs zero."
                              : command.kind == MOTOR_CMD_RGB_TEST ? "RGB test complete."
                                                                   : ""
                            : esp_err_to_name(e));
            if ((command.kind != MOTOR_CMD_UPDATE && command.kind != MOTOR_CMD_RGB) || e != ESP_OK) {
                motor_ui_refresh();
            } else {
                motor_ui_refresh_feedback();
            }
            bsp_display_unlock();
            next_poll = 0;
        }

        /* Poll actual values at 200ms intervals, keeping staged targets separate from feedback. */
        if (count && (int32_t)(xTaskGetTickCount() - next_poll) >= 0) {
            bsp_display_lock(0);
            unsigned index = model.device_index;
            bsp_display_unlock();
            if (index >= count) {
                index = 0;
            }
            bool switched = index != active;
            if (switched && model.hardware_control) {
                m5_motor_stop_all(devices[active]);
                m5_motor_stop_all(devices[index]);
            }
            active = index;
            uint8_t fw = 0;
            m5_motor_telemetry_t feedback = {};
            esp_err_t e = m5_motor_get_version(devices[active], &fw);
            if (e == ESP_OK) {
                e = m5_motor_read_telemetry(devices[active], &feedback);
            }
            if ((polls++ % 5) == 0 || e != ESP_OK) {
                log_feedback(feedback, e);
            }
            if (e != ESP_OK && model.hardware_control) {
                m5_motor_stop_all(devices[active]);
            }
            bsp_display_lock(0);
            bool changed = model.connected != (e == ESP_OK) || model.io_error != (e != ESP_OK) || switched;
            model.io_error = e != ESP_OK;
            model.connected = e == ESP_OK;
            if (e == ESP_OK) {
                model.channels = m5_motor_channels(devices[active]);
                model.version = fw;
                if (model.address != m5_motor_address(devices[active]) || changed) {
                    model.address = model.pending_address = m5_motor_address(devices[active]);
                }
                model.millivolts = feedback.vin_mv;
                model.milliamps = (int)(feedback.current_a * 1000.0f);
                for (unsigned ch = 0; ch < model.channels; ch++) {
                    model.motor[ch].pwm = feedback.pwm[ch];
                    model.motor[ch].speed = feedback.speed[ch];
                    model.motor[ch].encoder = feedback.encoder[ch];
                    if (switched) {
                        model.motor[ch].running = false;
                        model.motor[ch].pwm_target = 0;
                    }
                }
                if (model.selected >= model.channels) {
                    model.selected = 0;
                }
            } else {
                std::snprintf(model.notice, sizeof(model.notice), "I2C: %s", esp_err_to_name(e));
            }

            /* Routine feedback updates labels in place and preserves an active slider gesture. */
            if (changed) {
                motor_ui_refresh();
            } else {
                motor_ui_refresh_feedback();
            }
            bsp_display_unlock();

            /* Release failed handles and return to discovery instead of reusing stale communication state. */
            if (e != ESP_OK) {
                for (size_t i = 0; i < count; i++) {
                    m5_motor_delete(&devices[i]);
                }
                count = 0;
                next_discover = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
            }
            next_poll = xTaskGetTickCount() + pdMS_TO_TICKS(200);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
#else
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
}
