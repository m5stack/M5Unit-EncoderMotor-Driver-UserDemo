/**
 * @file test_driver.c
 * @brief Exercise the production driver against a strict fake I2C transport.
 *
 * Register response lengths match the module firmware. Fault injection checks
 * atomic read outputs, cleanup, address migration and best-effort stop behavior.
 */
#include "m5_unit_motor_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>

struct fake_bus {
    int unused;
};

struct fake_device {
    uint8_t address;
};
static struct fake_bus bus;

static struct {
    int present;
    uint8_t reg[256];
} chips[128];

static int handles, writes, reads, delays, fail_read = -1, fail_write = -1, fail_probe = -1, fail_add, fail_remove,
                                           checks;
#define CHECK(x)                                                                                                       \
    do {                                                                                                               \
        ++checks;                                                                                                      \
        if (!(x)) {                                                                                                    \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x);                                               \
            exit(1);                                                                                                   \
        }                                                                                                              \
    } while (0)

esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t b,
    const i2c_device_config_t *c,
    i2c_master_dev_handle_t *out)
{
    CHECK(b == &bus && c->scl_speed_hz == 100000 && c->device_address < 128);
    if (fail_add) {
        return ESP_ERR_NO_MEM;
    }

    *out = malloc(sizeof(**out));
    CHECK(*out);
    (*out)->address = c->device_address;
    handles++;

    return ESP_OK;
}

esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t d)
{
    CHECK(d);
    if (fail_remove) {
        fail_remove--;

        return ESP_ERR_INVALID_STATE;
    }
    free(d);
    handles--;

    return ESP_OK;
}

esp_err_t i2c_master_probe(i2c_master_bus_handle_t b, uint16_t a, int timeout)
{
    CHECK(b == &bus && a < 128 && timeout > 0);
    if (a == fail_probe) {
        return ESP_ERR_TIMEOUT;
    }

    return chips[a].present ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t d,
    const uint8_t *tx,
    size_t txlen,
    uint8_t *rx,
    size_t n,
    int timeout)
{
    CHECK(d && txlen == 1 && n > 0 && tx[0] + n <= 256 && timeout > 0);
    reads++;
    if (tx[0] == fail_read) {
        return ESP_ERR_TIMEOUT;
    }

    if (!chips[d->address].present) {
        return ESP_ERR_NOT_FOUND;
    }

    /* Match FW Prepare_I2C_Read_Response: it does not stream into next motor. */
    if (tx[0] >= 0x20 && tx[0] <= 0x23 && n != 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (tx[0] >= 0x30 && tx[0] <= 0x3F && n > (size_t)(4 - (tx[0] - 0x30) % 4)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (tx[0] >= 0x40 && tx[0] <= 0x43 && n != 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (tx[0] == 0xC0 && n > 4) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (tx[0] == 0xC4 && n != 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    memcpy(rx, chips[d->address].reg + tx[0], n);

    return ESP_OK;
}

esp_err_t i2c_master_transmit(i2c_master_dev_handle_t d, const uint8_t *tx, size_t n, int timeout)
{
    CHECK(d && n >= 2 && tx[0] + n - 1 <= 256 && timeout > 0);
    writes++;
    if (tx[0] == fail_write) {
        return ESP_ERR_TIMEOUT;
    }

    if (!chips[d->address].present) {
        return ESP_ERR_NOT_FOUND;
    }

    if (tx[0] == 0xFF) {
        CHECK(n == 2 && tx[1] >= 8 && tx[1] <= 0x77 && !chips[tx[1]].present);
        chips[tx[1]] = chips[d->address];
        memset(&chips[d->address], 0, sizeof(chips[0]));
        chips[tx[1]].reg[0xFF] = tx[1];
    } else {
        memcpy(chips[d->address].reg + tx[0], tx + 1, n - 1);
    }
    return ESP_OK;
}

void vTaskDelay(unsigned ticks)
{
    delays += ticks;
}

/**
 * @brief Require cleanup from the previous case before resetting registers and injected faults.
 */
static void reset(void)
{
    CHECK(handles == 0);
    memset(chips, 0, sizeof(chips));
    writes = reads = delays = 0;
    fail_read = fail_write = fail_probe = -1;
    fail_add = fail_remove = 0;
}

/**
 * @brief Seed one present module with deterministic identity and telemetry registers.
 */
static void chip(uint8_t a)
{
    chips[a].present = 1;
    chips[a].reg[0xFF] = a;
    chips[a].reg[0xFE] = 1;
    for (int i = 0; i < 12; i++) {
        chips[a].reg[0xE0 + i] = (uint8_t)(a + i);
    }
    chips[a].reg[0xB0] = 0xBC;
    chips[a].reg[0xB1] = 0x0A;
    chips[a].reg[0xB2] = 0xA4;
    chips[a].reg[0xB3] = 0x2E; /* 11940 mV */
    float current = 0.5f;
    memcpy(chips[a].reg + 0x90, &current, 4);
}

static m5_motor_handle_t open_chip(m5_motor_type_t type, uint8_t address)
{
    chip(address);
    m5_motor_handle_t h = NULL;
    m5_motor_config_t c = {&bus, type, address, 100000, 100};
    CHECK(m5_motor_create(&c, &h) == ESP_OK);

    return h;
}

static void close_chip(m5_motor_handle_t *h)
{
    CHECK(m5_motor_delete(h) == ESP_OK && !*h);
}

/**
 * @brief Cover absent, single, quad, simultaneous and invalid discovery responses.
 */
static void discovery(void)
{
    reset();
    m5_motor_handle_t h[2];
    size_t n = 99;
    CHECK(m5_motor_discover(&bus, h, &n) == ESP_OK && n == 0 && writes == 0);
    chip(0x2F);
    CHECK(m5_motor_discover(&bus, h, &n) == ESP_OK && n == 1 && m5_motor_channels(h[0]) == 1);
    close_chip(&h[0]);
    chip(0x24);
    CHECK(m5_motor_discover(&bus, h, &n) == ESP_OK && n == 2 && writes == 0);
    CHECK(m5_motor_address(h[0]) == 0x2F && m5_motor_channels(h[1]) == 4);
    close_chip(&h[0]);
    close_chip(&h[1]);
    chips[0x2F].present = 0;
    CHECK(m5_motor_discover(&bus, h, &n) == ESP_OK && n == 1 && m5_motor_channels(h[0]) == 4);
    close_chip(&h[0]);
    chip(0x2F);
    fail_read = 0xFE;
    CHECK(m5_motor_discover(&bus, h, &n) == ESP_ERR_TIMEOUT && n == 0 && handles == 0);
    fail_read = -1;
    chips[0x24].reg[0xFF] = 0x25;
    CHECK(m5_motor_discover(&bus, h, &n) == ESP_ERR_INVALID_RESPONSE && n == 0 && handles == 0);
    fail_probe = 0x2F;
    CHECK(m5_motor_discover(&bus, h, &n) == ESP_ERR_TIMEOUT && n == 0);
    puts("PASS discovery: none/single/quad/both/address echo/read and bus errors");
}

/**
 * @brief Check command encoding, channel bounds and stop behavior across all four motors.
 */
static void commands(void)
{
    reset();
    m5_motor_handle_t h = open_chip(M5_MOTOR_QUAD, 0x24);
    for (uint8_t ch = 0; ch < 4; ch++) {
        CHECK(m5_motor_set_pwm(h, ch, -127) == ESP_OK && chips[0x24].reg[0x20 + ch] == 0x81);
        CHECK(m5_motor_set_mode(h, ch, M5_MOTOR_SPEED) == ESP_OK && chips[0x24].reg[0x50 + 16 * ch] == 2);
        CHECK(m5_motor_set_position_target(h, ch, -1234567) == ESP_OK);
        const uint8_t expected[] = {0x79, 0x29, 0xED, 0xFF};
        CHECK(!memcmp(chips[0x24].reg + 0x54 + 16 * ch, expected, 4));
        CHECK(m5_motor_set_speed_target(h, ch, -23) == ESP_OK && chips[0x24].reg[0x5C + 16 * ch] == 0xE9);
        CHECK(m5_motor_set_position_limit(h, ch, 23) == ESP_OK && chips[0x24].reg[0x58 + 16 * ch] == 23);
        CHECK(m5_motor_set_pid(h, ch, M5_MOTOR_POSITION, 3, 1, 15) == ESP_OK);
        CHECK(chips[0x24].reg[0x51 + 16 * ch] == 3 && chips[0x24].reg[0x53 + 16 * ch] == 15);
        CHECK(m5_motor_set_pid(h, ch, M5_MOTOR_SPEED, 10, 1, 15) == ESP_OK && chips[0x24].reg[0x59 + 16 * ch] == 10);
        CHECK(m5_motor_set_encoder(h, ch, INT32_MIN) == ESP_OK && chips[0x24].reg[0x33 + 4 * ch] == 0x80);
        m5_motor_control_t control = {0};
        CHECK(m5_motor_read_control(h, ch, &control) == ESP_OK);
        CHECK(control.mode == M5_MOTOR_SPEED && control.position_target == -1234567 && control.speed_target == -23);
        CHECK(control.position_limit == 23 && control.position_pid[0] == 3 && control.position_pid[2] == 15 &&
              control.speed_pid[0] == 10);
        m5_motor_control_t saved = control;
        fail_read = 0x50 + 16 * ch;
        CHECK(m5_motor_read_control(h, ch, &control) == ESP_ERR_TIMEOUT && !memcmp(&saved, &control, sizeof(control)));
        fail_read = -1;
        chips[0x24].reg[0x50 + 16 * ch] = 3;
        CHECK(m5_motor_read_control(h, ch, &control) == ESP_ERR_INVALID_RESPONSE &&
              !memcmp(&saved, &control, sizeof(control)));
        chips[0x24].reg[0x50 + 16 * ch] = 2;
    }
    int before = writes;
    CHECK(m5_motor_set_pwm(h, 4, 0) == ESP_ERR_INVALID_ARG);
    CHECK(m5_motor_set_pwm(h, 0, -128) == ESP_ERR_INVALID_ARG);
    CHECK(m5_motor_set_speed_target(h, 0, 128) == ESP_ERR_INVALID_ARG);
    CHECK(m5_motor_set_position_limit(h, 0, 128) == ESP_ERR_INVALID_ARG);
    CHECK(m5_motor_set_mode(h, 0, (m5_motor_mode_t)3) == ESP_ERR_INVALID_ARG);
    CHECK(m5_motor_set_pid(h, 0, M5_MOTOR_OPEN_LOOP, 0, 0, 0) == ESP_ERR_INVALID_ARG && writes == before);
    fail_write = 0x20;
    before = writes;
    CHECK(m5_motor_stop_all(h) == ESP_ERR_TIMEOUT && writes - before == 12);
    CHECK(chips[0x24].reg[0x80] == 0 && chips[0x24].reg[0x23] == 0 && chips[0x24].reg[0x8C] == 0);
    fail_write = -1;
    CHECK(m5_motor_stop_all(h) == ESP_OK);
    close_chip(&h);
    h = open_chip(M5_MOTOR_SINGLE, 0x2F);
    before = writes;
    CHECK(m5_motor_set_encoder(h, 1, 0) == ESP_ERR_INVALID_ARG && writes == before);
    close_chip(&h);
    puts("PASS commands: all channel offsets, little endian, signed limits, PID, best-effort stop");
}

/**
 * @brief Check signed decoding and ensure partial reads never replace caller data.
 */
static void telemetry(void)
{
    reset();
    m5_motor_handle_t h = open_chip(M5_MOTOR_QUAD, 0x24);
    chips[0x24].reg[0x20] = 0x81;
    chips[0x24].reg[0x43] = 0xE9;
    chips[0x24].reg[0x21] = 8;
    chips[0x24].reg[0x22] = 0xF8;
    chips[0x24].reg[0x23] = 127;
    chips[0x24].reg[0x34] = 0x78;
    chips[0x24].reg[0x35] = 0x56;
    chips[0x24].reg[0x36] = 0x34;
    chips[0x24].reg[0x37] = 0x12;
    chips[0x24].reg[0x3F] = 0x80;
    m5_motor_telemetry_t t;
    CHECK(m5_motor_read_telemetry(h, &t) == ESP_OK);
    CHECK(t.pwm[0] == -127 && t.speed[3] == -23 && t.encoder[3] == INT32_MIN && t.vin_mv == 11940 && t.adc == 0xABC &&
          t.current_a == 0.5f);
    CHECK(t.pwm[1] == 8 && t.pwm[2] == -8 && t.pwm[3] == 127 && t.encoder[1] == 0x12345678);
    m5_motor_telemetry_t saved = t;
    fail_read = 0x38;
    CHECK(m5_motor_read_telemetry(h, &t) == ESP_ERR_TIMEOUT && !memcmp(&saved, &t, sizeof(t)));
    fail_read = -1;
    m5_motor_telemetry_t before = t;
    fail_read = 0xB0;
    CHECK(m5_motor_read_telemetry(h, &t) == ESP_ERR_TIMEOUT && !memcmp(&before, &t, sizeof(t)));
    fail_read = -1;
    float nan = NAN;
    memcpy(chips[0x24].reg + 0x90, &nan, 4);
    CHECK(m5_motor_read_telemetry(h, &t) == ESP_ERR_INVALID_RESPONSE && !memcmp(&before, &t, sizeof(t)));
    float current = 0.5f;
    memcpy(chips[0x24].reg + 0x90, &current, 4);
    chips[0x24].reg[0xB1] = 0x10;
    CHECK(m5_motor_read_telemetry(h, &t) == ESP_ERR_INVALID_RESPONSE);
    uint8_t version = 99;
    fail_read = 0xFE;
    CHECK(m5_motor_get_version(h, &version) == ESP_ERR_TIMEOUT && version == 99);
    close_chip(&h);
    h = open_chip(M5_MOTOR_SINGLE, 0x2F);
    fail_read = -1;
    memset(&t, 0xFF, sizeof(t));
    CHECK(m5_motor_read_telemetry(h, &t) == ESP_OK && t.encoder[3] == 0);
    close_chip(&h);
    puts("PASS telemetry: signed decoding, VIN/float, atomic error snapshots, invalid data");
}

/**
 * @brief Check collision handling, persistent writes, identity verification and cleanup.
 */
static void addresses(void)
{
    for (int type = 1; type <= 4; type += 3) {
        reset();
        uint8_t old = type == 1 ? 0x2F : 0x24;
        m5_motor_handle_t h = open_chip((m5_motor_type_t)type, old);
        int w = writes;
        CHECK(m5_motor_set_address(h, old) == ESP_OK && writes == w);
        CHECK(m5_motor_set_address(h, 7) == ESP_ERR_INVALID_ARG && writes == w);
        chip(0x30);
        CHECK(m5_motor_set_address(h, 0x30) == ESP_ERR_INVALID_STATE && writes == w);
        chips[0x30].present = 0;
        fail_probe = 0x30;
        CHECK(m5_motor_set_address(h, 0x30) == ESP_ERR_TIMEOUT && writes == w);
        fail_probe = -1;
        fail_add = 1;
        CHECK(m5_motor_set_address(h, 0x30) == ESP_ERR_NO_MEM && writes == w);
        fail_add = 0;
        fail_write = 0xFF;
        CHECK(m5_motor_set_address(h, 0x30) == ESP_ERR_TIMEOUT && handles == 1 && m5_motor_address(h) == old);
        fail_write = -1;
        CHECK(m5_motor_set_address(h, 0x30) == ESP_OK && m5_motor_address(h) == 0x30 && delays >= 300 && handles == 1);
        uint8_t v = 0;
        CHECK(m5_motor_get_version(h, &v) == ESP_OK && v == 1);
        close_chip(&h);
    }
    reset();
    m5_motor_handle_t h = open_chip(M5_MOTOR_SINGLE, 0x2F);
    fail_read = 0xFF;
    CHECK(m5_motor_set_address(h, 0x31) == ESP_ERR_TIMEOUT && m5_motor_address(h) == 0x31 && handles == 1);
    close_chip(&h);
    reset();
    h = open_chip(M5_MOTOR_QUAD, 0x24);
    fail_remove = 1;
    CHECK(m5_motor_set_address(h, 0x32) == ESP_ERR_INVALID_STATE && handles == 2 && m5_motor_address(h) == 0x32);
    close_chip(&h);
    CHECK(handles == 0);
    puts("PASS address: no-op, bounds, collision, failures, relocation, restart delay, handle cleanup");
}

/**
 * @brief Check BGR wire order, separate brightness reads and failure atomicity.
 */
static void rgb(void)
{
    reset();
    m5_motor_handle_t h = open_chip(M5_MOTOR_QUAD, 0x24);
    CHECK(m5_motor_set_rgb(h, 0x123456, 255) == ESP_OK);
    const uint8_t expected[] = {0x56, 0x34, 0x12, 0, 255};
    CHECK(!memcmp(chips[0x24].reg + 0xC0, expected, 5));
    m5_motor_rgb_t read = {0};
    CHECK(m5_motor_get_rgb(h, &read) == ESP_OK && read.color == 0x123456 && read.brightness == 255);
    m5_motor_rgb_t saved = read;
    fail_read = 0xC4;
    CHECK(m5_motor_get_rgb(h, &read) == ESP_ERR_TIMEOUT && !memcmp(&saved, &read, sizeof(read)));
    fail_read = -1;
    int before = writes;
    CHECK(m5_motor_set_rgb(h, 0x1000000, 0) == ESP_ERR_INVALID_ARG && writes == before);
    CHECK(
        m5_motor_set_rgb(h, 0, 0) == ESP_OK && m5_motor_get_rgb(h, &read) == ESP_OK && !read.color && !read.brightness);
    close_chip(&h);
    h = open_chip(M5_MOTOR_SINGLE, 0x2F);
    CHECK(m5_motor_set_rgb(h, 0xFF0000, 32) == ESP_OK && m5_motor_get_rgb(h, &read) == ESP_OK &&
          read.color == 0xFF0000 && read.brightness == 32);
    close_chip(&h);
    puts("PASS RGB: BGR/dummy/brightness wire order, split response, bounds, atomic read and both families");
}

int main(void)
{
    discovery();
    commands();
    telemetry();
    addresses();
    rgb();
    reset();
    m5_motor_handle_t h = NULL;
    m5_motor_config_t c = {&bus, M5_MOTOR_SINGLE, 0x2F, 100000, 0};
    CHECK(m5_motor_create(&c, &h) == ESP_ERR_INVALID_ARG && !h);
    c.timeout_ms = 100;
    c.type = (m5_motor_type_t)2;
    CHECK(m5_motor_create(&c, &h) == ESP_ERR_INVALID_ARG);
    CHECK(m5_motor_get_version(NULL, NULL) == ESP_ERR_INVALID_ARG);
    m5_motor_control_t control;
    CHECK(m5_motor_read_control(NULL, 0, &control) == ESP_ERR_INVALID_ARG);
    printf("All %d checks passed; zero leaked handles. Host transport model / no hardware.\n", checks);

    return 0;
}
