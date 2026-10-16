/**
 * @file app_motor_ui.h
 * @brief Model and queued-command boundary for the Tab5 motor UI.
 */
#pragma once

#include "lvgl.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_PWM,
    MOTOR_POSITION,
    MOTOR_SPEED,
    MOTOR_DEVICE,
} motor_page_t;

typedef enum {
    MOTOR_CMD_START,
    MOTOR_CMD_UPDATE,
    MOTOR_CMD_STOP,
    MOTOR_CMD_STOP_ALL,
    MOTOR_CMD_ADDRESS,
    MOTOR_CMD_TEST,
    MOTOR_CMD_RGB,
    MOTOR_CMD_RGB_TEST,
    MOTOR_CMD_MODE,
} motor_command_kind_t;

/**
 * @brief Immutable value snapshot copied into the worker queue.
 */
typedef struct {
    motor_command_kind_t kind;
    motor_page_t page;
    uint8_t channel; /**< Zero-based motor channel. */
    uint8_t device_index;
    uint8_t address;
    int pwm;            /**< Staged raw duty, -127..127. */
    int speed;          /**< Staged encoder count delta per 20 ms. */
    int limit;          /**< Position-loop raw output limit, 0..127. */
    int32_t position;   /**< Staged position in encoder counts. */
    uint32_t rgb_color; /**< 0xRRGGBB. */
    uint8_t rgb_brightness;
} motor_ui_command_t;

/**
 * @brief Copy a UI command to the backend without blocking or accessing I2C.
 *
 * @param command Snapshot valid only for the duration of this callback.
 * @param context Caller-supplied backend context.
 * @return true if the command was accepted; false if the queue is unavailable/full.
 */
typedef bool (*motor_ui_command_handler_t)(const motor_ui_command_t *command, void *context);

/**
 * @brief Feedback and staged controls are independent for each channel.
 */
typedef struct {
    int pwm;        /**< Actual raw PWM readback in live mode. */
    int pwm_target; /**< User's staged raw PWM command. */
    int speed;      /**< Actual encoder count delta per 20 ms. */
    int speed_target;
    int limit;
    int32_t encoder; /**< Actual signed encoder count. */
    int32_t position_target;
    bool running;             /**< Last acknowledged start/stop, not a firmware run-state register. */
    motor_page_t active_page; /**< Mode of the last acknowledged start command. */
} motor_channel_ui_t;

/**
 * @brief Caller-owned state; live updates require the BSP display lock.
 */
typedef struct {
    bool preview;
    bool hardware_control;
    bool connected;
    bool io_error;
    bool mode_switch_pending; /**< Controls stay disabled until the stop acknowledgement. */
    uint8_t channels;
    uint8_t address;
    uint8_t version;
    uint8_t selected;
    uint8_t pending_address;
    uint8_t device_count;
    uint8_t device_index; /**< Selector used when discovery finds both module families. */
    motor_page_t page;
    int millivolts;
    int milliamps;
    uint32_t rgb_color;
    uint8_t rgb_brightness;
    motor_channel_ui_t motor[4];
    char notice[100];
} motor_ui_model_t;

/**
 * @brief Build and load the shared 1280x720 UI while the caller holds the LVGL lock.
 *
 * @param model Initialized, non-NULL model that remains alive for the UI lifetime.
 * @return None.
 */
void motor_ui_init(motor_ui_model_t *model);

/**
 * @brief Rebuild the current page after navigation or a structural state change.
 *
 * @note The caller holds the LVGL lock and has initialized the UI.
 * @return None.
 */
void motor_ui_refresh(void);

/**
 * @brief Update feedback labels and bars in place without replacing touch controls.
 *
 * @note The caller holds the LVGL lock and has initialized the UI.
 * @return None.
 */
void motor_ui_refresh_feedback(void);

/**
 * @brief Register the nonblocking command sink used by live-control callbacks.
 *
 * @param handler Command-copy callback, or NULL to clear the sink. It must not do I2C.
 * @param context Opaque pointer passed to the handler.
 * @return None.
 */
void motor_ui_set_command_handler(motor_ui_command_handler_t handler, void *context);

/**
 * @brief Reset a model to deterministic preview data without hardware transactions.
 *
 * @param model Non-NULL caller-owned model; previous state is overwritten.
 * @param channels 1 for single-channel, 4 for quad, or 0 for an offline quad fixture.
 * @return None.
 */
void motor_ui_demo_init(motor_ui_model_t *model, unsigned channels);

/**
 * @brief Reset one stopped channel's staged motion targets without changing telemetry.
 *
 * @param model Initialized caller-owned model, protected by the display lock when live.
 * @param channel Zero-based channel to reset after a successful stop, in the range 0..3.
 * @note Output limits, encoder feedback and LED settings are preserved.
 * @return None.
 */
void motor_ui_reset_motion_targets(motor_ui_model_t *model, uint8_t channel);

/**
 * @brief Finish navigation after the backend acknowledges stopping all channels.
 *
 * @param model Caller-owned model, protected by the display lock.
 * @param page Requested destination page.
 * @param stopped true only when all stop writes succeeded; failures preserve page and targets.
 * @return None.
 */
void motor_ui_finish_mode_switch(motor_ui_model_t *model, motor_page_t page, bool stopped);

#ifdef __cplusplus
}
#endif
