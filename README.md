# M5Unit EncoderMotor Driver UserDemo

[English](README.md) | [简体中文](README_CN.md)

SKU: U232 / U233

## Description

Unit EncoderMotor Driver (U232) and Unit 4EncoderMotor Driver (U233) provide I2C control
for one and four encoder motors, respectively. This project provides a Tab5 touchscreen
demo with PWM, position, speed and RGB LED control, built with ESP-IDF 5.5.5 and LVGL 9.4.0.

## Related Link

- Unit EncoderMotor Driver (U232): [Document & Datasheet](https://docs.m5stack.com/en/products/sku/U232)
- Unit 4EncoderMotor Driver (U233): [Document & Datasheet](https://docs.m5stack.com/en/products/sku/U233)

## Build and use

Connect the module to Tab5 PORT.A (SDA: GPIO53, SCL: GPIO54), and connect the motor
and its power supply. The default configuration targets ESP32-P4 v1.x.

Activate the ESP-IDF 5.5.5 environment and run the following from the project root.

Flash the prebuilt [V1 firmware](firmware/P359-UnitEncoderMotorDriver-ESP32P4-UserDemo-V1-20261016_0x0.bin) at `0x0`:

```sh
python -m esptool --chip esp32p4 -p PORT -b 460800 write_flash \
  --flash_mode dio --flash_freq 80m --flash_size 16MB \
  0x0 firmware/P359-UnitEncoderMotorDriver-ESP32P4-UserDemo-V1-20261016_0x0.bin
```

To build and flash from source:

```sh
cd src
idf.py build
idf.py -p PORT flash monitor
```

Replace `PORT` with the Tab5 serial port. Press Ctrl+] to exit the monitor.

Select a motor channel and PWM, POSITION or SPEED mode, adjust the target and press START.
POSITION provides target position and Max motor output (%) sliders.
STOP stops the selected channel; STOP ALL stops all channels. DEVICE provides module
information, I2C address settings and LED controls.

Switching control modes stops all channels and clears their motion targets.
A green LIVE DEVICE badge indicates a connected device.

## License

- M5Unit EncoderMotor Driver UserDemo - [MIT](LICENSE)
