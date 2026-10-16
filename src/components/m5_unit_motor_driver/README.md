# m5_unit_motor_driver

[English](README.md) | [简体中文](README_CN.md)

ESP-IDF >=5.4 I2C component for U232 (one channel) and U233 (four channels).
Uses the new I2C master API. The Tab5 application requires ESP-IDF 5.5.5.

## Integration

The caller owns and initializes the I2C bus:

```c
#include "m5_unit_motor_driver.h"

m5_motor_handle_t devices[2] = {0};
size_t count = 0;
esp_err_t err = m5_motor_discover(bus, devices, &count);
if (err == ESP_OK) {
    for (size_t i = 0; i < count; ++i) {
        uint8_t version;
        if (m5_motor_get_version(devices[i], &version) == ESP_OK) {
            /* Publish version outside the I2C worker as appropriate. */
        }
        m5_motor_delete(&devices[i]);
    }
}
```

Discovery probes 0x2F and 0x24, checks the address echo and firmware register,
and infers the family from the default address. The protocol has no product ID.
For a nondefault address use `m5_motor_create()` with a known family explicitly.

Handles borrow the bus. Delete them before deleting the bus and serialize all operations
on each handle. Synchronous calls have finite timeouts and belong outside LVGL callbacks
and the display lock. Creation and discovery do not start a motor.

## Data and control

- Channels are zero based.
- PWM and speed targets: -127..127; -128 is rejected.
- Position and encoder: signed 32-bit little endian.
- Speed: encoder count delta per 20 ms.
- Position output limit: raw magnitude 0..127.
- RGB: public 0xRRGGBB and brightness 0..255; transport uses B,G,R,dummy,brightness.

See the documented [public header](include/m5_unit_motor_driver.h) for all APIs.
Readback supports telemetry, control parameters, firmware, UID and RGB.
Setters cover PWM, mode, encoder origin, position/speed targets, limit, PID and address.

Telemetry is published only after all transfers succeed. Quad firmware returns only the
selected channel, so the implementation reads PWM, speed and encoder per channel.
The caller's previous snapshot is preserved on read failure.

`m5_motor_stop_all()` clears speed targets, selects open loop and zeros PWM on every channel,
continuing after failures and returning the first error. A communication failure leaves
the physical stop unconfirmed.

## Persistent address changes

Address writes save Flash. The driver rejects occupied addresses, skips the current
address, prepares a new handle, writes, waits 350 ms and verifies address echo and UID.
After an acknowledged write the handle tracks the new address even if verification fails.
A transmit failure leaves the physical address uncertain; reopen/discover explicitly.
A handle is retained if deletion fails.

The Tab5 LIVE DEVICE mode connects UI controls to these APIs.
SIMULATION uses a model; READ ONLY disables writes.

## Protocol reference and tests

Based on internal firmware revision `dbf00770cddbaf6058454faa976d7e5ef4607e9d`,
V1 / 2026-08-27. The driver includes its own license.

Run the production driver host tests from the application's `src/` directory:

```sh
cmake -S components/m5_unit_motor_driver/test_apps/host -B /tmp/motor-driver-tests
cmake --build /tmp/motor-driver-tests --parallel 4
ctest --test-dir /tmp/motor-driver-tests --output-on-failure
```

The modeled I2C transport covers protocol values, channel offsets, failed transfers,
discovery, address relocation and cleanup under ASan/UBSan.
