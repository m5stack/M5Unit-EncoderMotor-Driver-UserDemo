/* SPDX-License-Identifier: MIT */
/**
 * @file m5_unit_motor_driver.h
 * @brief Single and four-channel EncoderMotor driver using the ESP-IDF I2C master API.
 *
 * Handles borrow the caller's bus. The caller must serialize all operations on a
 * handle, including deletion and address changes. Channels are zero based.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define M5_MOTOR_SINGLE_ADDRESS 0x2F
#define M5_MOTOR_QUAD_ADDRESS 0x24

/**
 * @brief Module family; the enum value also gives the channel count.
 */
typedef enum {
    M5_MOTOR_SINGLE = 1,
    M5_MOTOR_QUAD = 4,
} m5_motor_type_t;

/**
 * @brief Firmware control mode written to the selected channel.
 */
typedef enum {
    M5_MOTOR_OPEN_LOOP = 0,
    M5_MOTOR_POSITION = 1,
    M5_MOTOR_SPEED = 2,
} m5_motor_mode_t;

typedef struct m5_motor_device *m5_motor_handle_t;

/**
 * @brief Transport settings for one module.
 */
typedef struct {
    i2c_master_bus_handle_t bus; /**< Borrowed I2C master bus. */
    m5_motor_type_t type;        /**< Explicit family, including at a nondefault address. */
    uint8_t address;             /**< 7-bit I2C address, 0x08..0x77. */
    uint32_t clock_hz;           /**< I2C clock: 100000 or 400000 Hz. */
    int timeout_ms;              /**< Finite transaction timeout, greater than zero. */
} m5_motor_config_t;

/**
 * @brief Complete telemetry snapshot; unused channel entries remain zero.
 */
typedef struct {
    uint16_t vin_mv;    /**< Input voltage in millivolts. */
    uint16_t adc;       /**< Raw 12-bit voltage ADC sample. */
    float current_a;    /**< Current in amperes. */
    int8_t pwm[4];      /**< Signed raw PWM readback. */
    int8_t speed[4];    /**< Encoder count delta per 20 ms, not RPM. */
    int32_t encoder[4]; /**< Signed encoder count. */
} m5_motor_telemetry_t;

/**
 * @brief Selected channel's control registers.
 */
typedef struct {
    m5_motor_mode_t mode;
    uint8_t position_pid[3]; /**< Position coefficients in P, I, D order. */
    uint8_t speed_pid[3];    /**< Speed coefficients in P, I, D order. */
    uint8_t position_limit;  /**< Raw position-loop output limit, 0..127. */
    int32_t position_target; /**< Target encoder count. */
    int8_t speed_target;     /**< Target encoder count delta per 20 ms. */
} m5_motor_control_t;

/**
 * @brief RGB color and raw brightness read from the module.
 */
typedef struct {
    uint32_t color;     /**< Color encoded as 0xRRGGBB. */
    uint8_t brightness; /**< Brightness, 0..255. */
} m5_motor_rgb_t;

/**
 * @brief Create a transport handle without probing or writing motor registers.
 *
 * @param config Module family, borrowed bus, address, clock and timeout settings.
 * @param out Receives the owned handle; set to NULL on creation failure.
 * @return
 *     - ESP_OK Handle created.
 *     - ESP_ERR_INVALID_ARG Invalid configuration or output pointer.
 *     - ESP_ERR_NO_MEM Allocation failed.
 *     - Other ESP-IDF I2C errors if the transport could not be attached.
 */
esp_err_t m5_motor_create(const m5_motor_config_t *config, m5_motor_handle_t *out);

/**
 * @brief Remove transport devices and free a handle without deleting its bus.
 *
 * @param handle Address of an owned handle; set to NULL on success. A transport
 *               removal failure leaves the handle available for a cleanup retry.
 * @return
 *     - ESP_OK Handle deleted.
 *     - ESP_ERR_INVALID_ARG NULL pointer or handle.
 *     - Other ESP-IDF I2C errors if transport removal failed.
 */
esp_err_t m5_motor_delete(m5_motor_handle_t *handle);

/**
 * @brief Discover modules at 0x2F and 0x24 without writing motor registers.
 *
 * Family is inferred from the default address because there is no product-ID
 * register. Address echo and firmware reads must succeed before discovery returns.
 *
 * @param bus Borrowed I2C master bus, probed at 100 kHz.
 * @param out Space for two owned handles; delete each discovered handle after use.
 * @param count Receives 0..2 handles on success, or zero on failure.
 * @return
 *     - ESP_OK Discovery completed, including when neither address responded.
 *     - ESP_ERR_INVALID_ARG NULL bus or output pointer.
 *     - ESP_ERR_INVALID_RESPONSE Address echo did not match the probed address.
 *     - Other allocation or ESP-IDF I2C errors. Created handles are cleaned up.
 */
esp_err_t m5_motor_discover(i2c_master_bus_handle_t bus, m5_motor_handle_t out[2], size_t *count);

/**
 * @brief Get the channel count recorded in the handle without I2C traffic.
 *
 * @param handle Module handle.
 * @return 1 or 4 for a valid handle; 0 for NULL.
 */
uint8_t m5_motor_channels(m5_motor_handle_t handle);

/**
 * @brief Get the address currently recorded in the handle without I2C traffic.
 *
 * @param handle Module handle.
 * @return 7-bit I2C address, or 0 for NULL.
 */
uint8_t m5_motor_address(m5_motor_handle_t handle);

/**
 * @brief Read the firmware version byte from register 0xFE.
 *
 * @param handle Module handle.
 * @param version Receives the version on success; unchanged on failure.
 * @return
 *     - ESP_OK Version read.
 *     - ESP_ERR_INVALID_ARG NULL handle or version pointer.
 *     - Other ESP-IDF I2C errors if the read failed.
 */
esp_err_t m5_motor_get_version(m5_motor_handle_t handle, uint8_t *version);

/**
 * @brief Read the 12-byte device identity from register 0xE0.
 *
 * @param handle Module handle.
 * @param uid Buffer for 12 bytes; unchanged on failure.
 * @return
 *     - ESP_OK Identity read.
 *     - ESP_ERR_INVALID_ARG NULL handle or buffer.
 *     - Other ESP-IDF I2C errors if the read failed.
 */
esp_err_t m5_motor_get_uid(m5_motor_handle_t handle, uint8_t uid[12]);

/**
 * @brief Read per-channel feedback, input voltage and current as one complete snapshot.
 *
 * Channels are read separately to respect the firmware response boundaries.
 * Speed is encoder counts per 20 ms; no RPM conversion is performed.
 *
 * @param handle Module handle.
 * @param out Receives the complete snapshot; unchanged if any read or validation fails.
 * @return
 *     - ESP_OK Snapshot read.
 *     - ESP_ERR_INVALID_ARG NULL handle or output pointer.
 *     - ESP_ERR_INVALID_RESPONSE Non-finite current or ADC outside 0..4095.
 *     - Other ESP-IDF I2C errors if a read failed.
 */
esp_err_t m5_motor_read_telemetry(m5_motor_handle_t handle, m5_motor_telemetry_t *out);

/**
 * @brief Read a channel's mode, PID coefficients, targets and position output limit.
 *
 * @param handle Module handle.
 * @param channel Zero-based channel: 0 for single, 0..3 for quad.
 * @param out Receives validated control settings; unchanged on failure.
 * @return
 *     - ESP_OK Control registers read.
 *     - ESP_ERR_INVALID_ARG Invalid handle, channel or output pointer.
 *     - ESP_ERR_INVALID_RESPONSE Invalid mode, position limit or speed target byte.
 *     - Other ESP-IDF I2C errors if the read failed.
 */
esp_err_t m5_motor_read_control(m5_motor_handle_t handle, uint8_t channel, m5_motor_control_t *out);

/**
 * @brief Read RGB color and brightness using separate firmware response blocks.
 *
 * @param handle Module handle.
 * @param out Receives 0xRRGGBB color and 0..255 brightness; unchanged on failure.
 * @return
 *     - ESP_OK RGB settings read.
 *     - ESP_ERR_INVALID_ARG NULL handle or output pointer.
 *     - Other ESP-IDF I2C errors if a read failed.
 */
esp_err_t m5_motor_get_rgb(m5_motor_handle_t handle, m5_motor_rgb_t *out);

/**
 * @brief Write RGB color and brightness in the firmware's B,G,R,0,brightness order.
 *
 * @param handle Module handle.
 * @param color Color in 0x000000..0xFFFFFF (0xRRGGBB); zero selects black/off.
 * @param brightness Raw brightness, 0..255.
 * @return
 *     - ESP_OK Write acknowledged; no readback is performed by this function.
 *     - ESP_ERR_INVALID_ARG Invalid handle or color outside 24 bits.
 *     - Other ESP-IDF I2C errors if the write failed.
 */
esp_err_t m5_motor_set_rgb(m5_motor_handle_t handle, uint32_t color, uint8_t brightness);

/**
 * @brief Write a channel's signed raw PWM output without changing its control mode.
 *
 * @param handle Module handle.
 * @param channel Zero-based channel: 0 for single, 0..3 for quad.
 * @param value Raw duty command, -127..127; zero stops open-loop output.
 * @return
 *     - ESP_OK Write acknowledged.
 *     - ESP_ERR_INVALID_ARG Invalid handle, channel or duty range (-128 is rejected).
 *     - Other ESP-IDF I2C errors if the write failed.
 */
esp_err_t m5_motor_set_pwm(m5_motor_handle_t handle, uint8_t channel, int value);

/**
 * @brief Select open-loop PWM, position control or speed control for a channel.
 *
 * This writes the mode only; the caller sets targets and sequences any required stop.
 *
 * @param handle Module handle.
 * @param channel Zero-based channel: 0 for single, 0..3 for quad.
 * @param mode M5_MOTOR_OPEN_LOOP (0), M5_MOTOR_POSITION (1), or M5_MOTOR_SPEED (2).
 * @return
 *     - ESP_OK Mode write acknowledged.
 *     - ESP_ERR_INVALID_ARG Invalid handle, channel or control mode.
 *     - Other ESP-IDF I2C errors if the write failed.
 */
esp_err_t m5_motor_set_mode(m5_motor_handle_t handle, uint8_t channel, m5_motor_mode_t mode);

/**
 * @brief Overwrite a channel's signed encoder count in the module.
 *
 * @param handle Module handle.
 * @param channel Zero-based channel: 0 for single, 0..3 for quad.
 * @param count New 32-bit encoder count; zero resets the stored count.
 * @return
 *     - ESP_OK Count write acknowledged.
 *     - ESP_ERR_INVALID_ARG Invalid handle or channel.
 *     - Other ESP-IDF I2C errors if the write failed.
 */
esp_err_t m5_motor_set_encoder(m5_motor_handle_t handle, uint8_t channel, int32_t count);

/**
 * @brief Set a channel's position target without changing its control mode.
 *
 * @param handle Module handle.
 * @param channel Zero-based channel: 0 for single, 0..3 for quad.
 * @param count Signed 32-bit target in encoder counts, not degrees or revolutions.
 * @return
 *     - ESP_OK Target write acknowledged.
 *     - ESP_ERR_INVALID_ARG Invalid handle or channel.
 *     - Other ESP-IDF I2C errors if the write failed.
 */
esp_err_t m5_motor_set_position_target(m5_motor_handle_t handle, uint8_t channel, int32_t count);

/**
 * @brief Set a channel's speed target without changing its control mode.
 *
 * @param handle Module handle.
 * @param channel Zero-based channel: 0 for single, 0..3 for quad.
 * @param value Target encoder count delta per 20 ms, -127..127; not RPM.
 * @return
 *     - ESP_OK Target write acknowledged.
 *     - ESP_ERR_INVALID_ARG Invalid handle, channel or target range (-128 is rejected).
 *     - Other ESP-IDF I2C errors if the write failed.
 */
esp_err_t m5_motor_set_speed_target(m5_motor_handle_t handle, uint8_t channel, int value);

/**
 * @brief Set the raw output limit used by a channel's position loop.
 *
 * @param handle Module handle.
 * @param channel Zero-based channel: 0 for single, 0..3 for quad.
 * @param value Unsigned raw output limit, 0..127; not a percentage.
 * @return
 *     - ESP_OK Limit write acknowledged.
 *     - ESP_ERR_INVALID_ARG Invalid handle, channel or limit range.
 *     - Other ESP-IDF I2C errors if the write failed.
 */
esp_err_t m5_motor_set_position_limit(m5_motor_handle_t handle, uint8_t channel, unsigned value);

/**
 * @brief Write the P, I and D coefficient bytes for a position or speed loop.
 *
 * @param handle Module handle.
 * @param channel Zero-based channel: 0 for single, 0..3 for quad.
 * @param loop M5_MOTOR_POSITION or M5_MOTOR_SPEED; open-loop PWM is rejected.
 * @param p Raw proportional coefficient, 0..255.
 * @param i Raw integral coefficient, 0..255.
 * @param d Raw derivative coefficient, 0..255.
 * @return
 *     - ESP_OK Coefficient write acknowledged.
 *     - ESP_ERR_INVALID_ARG Invalid handle, channel or loop selection.
 *     - Other ESP-IDF I2C errors if the write failed.
 */
esp_err_t m5_motor_set_pid(m5_motor_handle_t handle,
    uint8_t channel,
    m5_motor_mode_t loop,
    uint8_t p,
    uint8_t i,
    uint8_t d);

/**
 * @brief Attempt zero speed target, open-loop mode and zero PWM on every channel.
 *
 * Continues after failures and returns the first error. This is a software stop,
 * not a hardware emergency stop, and does not verify register readback.
 *
 * @param handle Module handle.
 * @return
 *     - ESP_OK All stop writes acknowledged.
 *     - ESP_ERR_INVALID_ARG NULL handle.
 *     - First ESP-IDF I2C error encountered while attempting the stop sequence.
 */
esp_err_t m5_motor_stop_all(m5_motor_handle_t handle);

/**
 * @brief Persist a new I2C address and verify address echo and device identity.
 *
 * Rejects an occupied address and skips Flash writes when the address is unchanged.
 * Waits 350 ms after an acknowledged write. The handle follows the new address
 * even if verification then fails. A transmit error leaves the actual address
 * uncertain; the caller must rediscover or reopen it explicitly.
 *
 * @param handle Module handle; caller is responsible for stopping outputs first.
 * @param address New 7-bit address, 0x08..0x77.
 * @return
 *     - ESP_OK Address unchanged, or changed and identity verified.
 *     - ESP_ERR_INVALID_ARG Invalid handle or address range.
 *     - ESP_ERR_INVALID_STATE Address occupied or old transport cleanup pending.
 *     - ESP_ERR_INVALID_RESPONSE Address echo or UID verification failed.
 *     - Other allocation or ESP-IDF I2C errors if transport or access failed.
 */
esp_err_t m5_motor_set_address(m5_motor_handle_t handle, uint8_t address);

#ifdef __cplusplus
}
#endif
