/* SPDX-License-Identifier: MIT */
/**
 * @file m5_unit_motor_driver.c
 * @brief Register transport for single and four-channel EncoderMotor drivers.
 *
 * The caller serializes operations per handle. Reads publish complete snapshots
 * only, and multi-byte integers are decoded explicitly from little-endian bytes.
 */
#include "m5_unit_motor_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

struct m5_motor_device {
    m5_motor_config_t config;

    /* Keep an old transport handle if removal failed so deletion can retry cleanup. */
    i2c_master_dev_handle_t dev, retired;
};

static int valid_address(uint8_t a)
{
    return a >= 8 && a <= 0x77;
}

/**
 * @brief Use a register-address write followed by a repeated-start read.
 */
static esp_err_t read_reg(m5_motor_handle_t h, uint8_t reg, void *out, size_t len)
{
    if (!h || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(h->dev, &reg, 1, out, len, h->config.timeout_ms);
}

/**
 * @brief Build the bounded register-plus-payload packet on the stack.
 */
static esp_err_t write_reg(m5_motor_handle_t h, uint8_t reg, const uint8_t *data, size_t len)
{
    if (!h || !data || len > 12) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t packet[13];
    packet[0] = reg;
    memcpy(packet + 1, data, len);

    return i2c_master_transmit(h->dev, packet, len + 1, h->config.timeout_ms);
}

/**
 * @brief Attach a device to the borrowed bus without probing or writing registers.
 */
static esp_err_t add_dev(const m5_motor_config_t *c, uint8_t address, i2c_master_dev_handle_t *out)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = c->clock_hz,
    };

    return i2c_master_bus_add_device(c->bus, &cfg, out);
}

esp_err_t m5_motor_create(const m5_motor_config_t *c, m5_motor_handle_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }

    *out = NULL;
    if (!c || !c->bus || !valid_address(c->address) || (c->type != M5_MOTOR_SINGLE && c->type != M5_MOTOR_QUAD) ||
        (c->clock_hz != 100000 && c->clock_hz != 400000) || c->timeout_ms <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    m5_motor_handle_t h = calloc(1, sizeof(*h));
    if (!h) {
        return ESP_ERR_NO_MEM;
    }

    h->config = *c;
    esp_err_t err = add_dev(c, c->address, &h->dev);
    if (err != ESP_OK) {
        free(h);

        return err;
    }
    *out = h;

    return ESP_OK;
}

esp_err_t m5_motor_delete(m5_motor_handle_t *h)
{
    if (!h || !*h) {
        return ESP_ERR_INVALID_ARG;
    }

    if ((*h)->retired) {
        esp_err_t e = i2c_master_bus_rm_device((*h)->retired);
        if (e != ESP_OK) {
            return e;
        }

        (*h)->retired = NULL;
    }
    esp_err_t err = i2c_master_bus_rm_device((*h)->dev);
    if (err != ESP_OK) {
        return err;
    }

    free(*h);
    *h = NULL;

    return ESP_OK;
}

uint8_t m5_motor_channels(m5_motor_handle_t h)
{
    return h ? (uint8_t)h->config.type : 0;
}

uint8_t m5_motor_address(m5_motor_handle_t h)
{
    return h ? h->config.address : 0;
}

esp_err_t m5_motor_get_version(m5_motor_handle_t h, uint8_t *v)
{
    if (!v) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t t;
    esp_err_t e = read_reg(h, 0xFE, &t, 1);
    if (e == ESP_OK) {
        *v = t;
    }
    return e;
}

esp_err_t m5_motor_get_uid(m5_motor_handle_t h, uint8_t uid[12])
{
    if (!uid) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t t[12];
    esp_err_t e = read_reg(h, 0xE0, t, 12);
    if (e == ESP_OK) {
        memcpy(uid, t, 12);
    }
    return e;
}

/**
 * @brief Infer family only at default addresses, then validate address echo and firmware.
 */
esp_err_t m5_motor_discover(i2c_master_bus_handle_t bus, m5_motor_handle_t out[2], size_t *count)
{
    if (!bus || !out || !count) {
        return ESP_ERR_INVALID_ARG;
    }

    *count = 0;
    out[0] = out[1] = NULL;
    const uint8_t addresses[] = {M5_MOTOR_SINGLE_ADDRESS, M5_MOTOR_QUAD_ADDRESS};
    const m5_motor_type_t types[] = {M5_MOTOR_SINGLE, M5_MOTOR_QUAD};
    esp_err_t err = ESP_OK;
    for (unsigned i = 0; i < 2; i++) {
        err = i2c_master_probe(bus, addresses[i], 100);
        if (err == ESP_ERR_NOT_FOUND) {
            continue;
        }
        if (err != ESP_OK) {
            goto fail;
        }
        m5_motor_config_t c = {
            .bus = bus,
            .type = types[i],
            .address = addresses[i],
            .clock_hz = 100000,
            .timeout_ms = 100,
        };
        err = m5_motor_create(&c, &out[*count]);
        if (err != ESP_OK) {
            goto fail;
        }
        ++*count;
        uint8_t address = 0, version = 0;
        err = read_reg(out[*count - 1], 0xFF, &address, 1);
        if (err != ESP_OK) {
            goto fail;
        }
        if (address != addresses[i]) {
            err = ESP_ERR_INVALID_RESPONSE;
            goto fail;
        }
        err = m5_motor_get_version(out[*count - 1], &version);
        if (err != ESP_OK) {
            goto fail;
        }
    }
    return ESP_OK;
fail:
    for (size_t j = 0; j < *count; j++) {
        m5_motor_delete(&out[j]);
    }
    *count = 0;

    return err;
}

/**
 * @brief Decode the wire representation without unaligned loads or signed shifts.
 */
static int32_t decode_i32(const uint8_t b[4])
{
    uint32_t u = (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24;
    int32_t v;
    memcpy(&v, &u, 4);

    return v;
}

/**
 * @brief Convert protocol two-complement bytes without relying on a signed narrowing cast.
 */
static int8_t decode_i8(uint8_t u)
{
    return (int8_t)(u < 128 ? (int)u : (int)u - 256);
}

/**
 * @brief Collect one complete sample; leave the caller output unchanged on any error.
 */
esp_err_t m5_motor_read_telemetry(m5_motor_handle_t h, m5_motor_telemetry_t *out)
{
    if (!h || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    m5_motor_telemetry_t t = {0};
    uint8_t bytes[16];
    uint32_t bits;
    esp_err_t e;
    size_t n = m5_motor_channels(h);
    /* Slave responses stop at the selected channel; adjacent registers do not
     * imply support for a cross-channel burst (quad FW Prepare_I2C_Read_Response). */
    for (size_t i = 0; i < n; i++) {
        e = read_reg(h, 0x20 + i, bytes, 1);
        if (e != ESP_OK) {
            return e;
        }

        t.pwm[i] = decode_i8(bytes[0]);
        e = read_reg(h, 0x30 + 4 * i, bytes, 4);
        if (e != ESP_OK) {
            return e;
        }

        t.encoder[i] = decode_i32(bytes);
        e = read_reg(h, 0x40 + i, bytes, 1);
        if (e != ESP_OK) {
            return e;
        }

        t.speed[i] = decode_i8(bytes[0]);
    }
    e = read_reg(h, 0x90, bytes, 4);
    if (e != ESP_OK) {
        return e;
    }

    bits = (uint32_t)bytes[0] | (uint32_t)bytes[1] << 8 | (uint32_t)bytes[2] << 16 | (uint32_t)bytes[3] << 24;
    _Static_assert(sizeof(float) == 4, "Protocol requires 32-bit float");
    memcpy(&t.current_a, &bits, 4);
    if (!isfinite(t.current_a)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    e = read_reg(h, 0xB0, bytes, 4);
    if (e != ESP_OK) {
        return e;
    }

    t.adc = (uint16_t)bytes[0] | (uint16_t)bytes[1] << 8;
    t.vin_mv = (uint16_t)bytes[2] | (uint16_t)bytes[3] << 8;
    if (t.adc > 4095) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *out = t;

    return ESP_OK; /* Do not expose partially read snapshots. */
}

static int valid_channel(m5_motor_handle_t h, uint8_t ch)
{
    return h && ch < m5_motor_channels(h);
}

/**
 * @brief Read the color block and brightness using the firmware response boundaries.
 */
esp_err_t m5_motor_get_rgb(m5_motor_handle_t h, m5_motor_rgb_t *out)
{
    if (!h || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t b[4], brightness;
    esp_err_t e = read_reg(h, 0xC0, b, 4);
    if (e != ESP_OK) {
        return e;
    }

    /* FW exposes C0..C3 and C4 as separate responses, not a five-byte read. */
    e = read_reg(h, 0xC4, &brightness, 1);
    if (e != ESP_OK) {
        return e;
    }

    m5_motor_rgb_t value = {
        .color = ((uint32_t)b[2] << 16) | ((uint32_t)b[1] << 8) | b[0],
        .brightness = brightness,
    };
    *out = value;

    return ESP_OK;
}

/**
 * @brief Wire order is blue, green, red, dummy, brightness; API color is 0xRRGGBB.
 */
esp_err_t m5_motor_set_rgb(m5_motor_handle_t h, uint32_t color, uint8_t brightness)
{
    if (!h || color > 0xFFFFFF) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t b[5] = {(uint8_t)color, (uint8_t)(color >> 8), (uint8_t)(color >> 16), 0, brightness};

    return write_reg(h, 0xC0, b, sizeof(b));
}

/**
 * @brief Decode the selected channel control block and reject invalid mode/range bytes.
 */
esp_err_t m5_motor_read_control(m5_motor_handle_t h, uint8_t ch, m5_motor_control_t *out)
{
    if (!valid_channel(h, ch) || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t b[13];
    esp_err_t e = read_reg(h, 0x50 + 16 * ch, b, sizeof(b));
    if (e != ESP_OK) {
        return e;
    }

    if (b[0] > M5_MOTOR_SPEED || b[8] > 127 || b[12] == 128) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    m5_motor_control_t t = {
        .mode = (m5_motor_mode_t)b[0],
        .position_target = decode_i32(b + 4),
        .position_limit = b[8],
        .speed_target = decode_i8(b[12]),
    };
    memcpy(t.position_pid, b + 1, 3);
    memcpy(t.speed_pid, b + 9, 3);
    *out = t;

    return ESP_OK;
}

static esp_err_t write_signed(m5_motor_handle_t h, uint8_t ch, uint8_t reg, int v)
{
    if (!valid_channel(h, ch) || v < -127 || v > 127) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t b = (uint8_t)v;

    return write_reg(h, reg, &b, 1);
}

esp_err_t m5_motor_set_pwm(m5_motor_handle_t h, uint8_t ch, int v)
{
    return write_signed(h, ch, 0x20 + ch, v);
}

esp_err_t m5_motor_set_speed_target(m5_motor_handle_t h, uint8_t ch, int v)
{
    return write_signed(h, ch, 0x5C + 16 * ch, v);
}

esp_err_t m5_motor_set_mode(m5_motor_handle_t h, uint8_t ch, m5_motor_mode_t mode)
{
    if (!valid_channel(h, ch) || mode < M5_MOTOR_OPEN_LOOP || mode > M5_MOTOR_SPEED) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t b = (uint8_t)mode;

    return write_reg(h, 0x50 + 16 * ch, &b, 1);
}

static esp_err_t write_i32(m5_motor_handle_t h, uint8_t ch, uint8_t reg, int32_t value)
{
    if (!valid_channel(h, ch)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t u = (uint32_t)value;
    uint8_t b[4] = {(uint8_t)u, (uint8_t)(u >> 8), (uint8_t)(u >> 16), (uint8_t)(u >> 24)};

    return write_reg(h, reg, b, 4);
}

esp_err_t m5_motor_set_encoder(m5_motor_handle_t h, uint8_t ch, int32_t v)
{
    return write_i32(h, ch, 0x30 + 4 * ch, v);
}

esp_err_t m5_motor_set_position_target(m5_motor_handle_t h, uint8_t ch, int32_t v)
{
    return write_i32(h, ch, 0x54 + 16 * ch, v);
}

esp_err_t m5_motor_set_position_limit(m5_motor_handle_t h, uint8_t ch, unsigned v)
{
    if (v > 127 || !valid_channel(h, ch)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t b = (uint8_t)v;

    return write_reg(h, 0x58 + 16 * ch, &b, 1);
}

esp_err_t m5_motor_set_pid(m5_motor_handle_t h, uint8_t ch, m5_motor_mode_t loop, uint8_t p, uint8_t i, uint8_t d)
{
    if (!valid_channel(h, ch) || (loop != M5_MOTOR_POSITION && loop != M5_MOTOR_SPEED)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t b[3] = {p, i, d};

    return write_reg(h, (loop == M5_MOTOR_POSITION ? 0x51 : 0x59) + 16 * ch, b, 3);
}

/**
 * @brief Try every channel even after an error; report the first failed write.
 */
esp_err_t m5_motor_stop_all(m5_motor_handle_t h)
{
    if (!h) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t first = ESP_OK, e;
    for (uint8_t ch = 0; ch < m5_motor_channels(h); ch++) {
        e = m5_motor_set_speed_target(h, ch, 0);
        if (first == ESP_OK) {
            first = e;
        }
        e = m5_motor_set_mode(h, ch, M5_MOTOR_OPEN_LOOP);
        if (first == ESP_OK) {
            first = e;
        }
        e = m5_motor_set_pwm(h, ch, 0);
        if (first == ESP_OK) {
            first = e;
        }
    }
    return first;
}

/**
 * @brief Move a handle after one persistent write, then verify the same device by UID.
 */
esp_err_t m5_motor_set_address(m5_motor_handle_t h, uint8_t address)
{
    if (!h || !valid_address(address)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (address == h->config.address) {
        return ESP_OK; /* Never wear Flash for a no-op. */
    }
    if (h->retired) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t e = i2c_master_probe(h->config.bus, address, h->config.timeout_ms);
    if (e == ESP_OK) {
        return ESP_ERR_INVALID_STATE;
    }

    if (e != ESP_ERR_NOT_FOUND) {
        return e;
    }

    uint8_t uid[12], after[12];
    e = m5_motor_get_uid(h, uid);
    if (e != ESP_OK) {
        return e;
    }

    i2c_master_dev_handle_t next = NULL;
    e = add_dev(&h->config, address, &next);
    if (e != ESP_OK) {
        return e;
    }

    /* Allocate the new transport first; allocation failure must not alter device Flash. */
    e = write_reg(h, 0xFF, &address, 1);
    if (e != ESP_OK) {
        i2c_master_bus_rm_device(next);

        return e;
    }

    /* After an acknowledged write, follow the new address even if later verification fails. */
    h->retired = h->dev;
    h->dev = next;
    h->config.address = address;
    e = i2c_master_bus_rm_device(h->retired);
    if (e != ESP_OK) {
        return e;
    }

    h->retired = NULL;

    /* Allow the single-channel firmware to reset and the quad firmware to change address. */
    vTaskDelay(pdMS_TO_TICKS(350));
    uint8_t echo = 0;
    e = read_reg(h, 0xFF, &echo, 1);
    if (e != ESP_OK) {
        return e;
    }

    if (echo != address) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    e = m5_motor_get_uid(h, after);
    if (e != ESP_OK) {
        return e;
    }

    return memcmp(uid, after, 12) ? ESP_ERR_INVALID_RESPONSE : ESP_OK;
}
