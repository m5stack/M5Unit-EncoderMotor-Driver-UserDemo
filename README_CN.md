# M5Unit EncoderMotor Driver 用户示例

[English](README.md) | [简体中文](README_CN.md)

SKU: U232 / U233

## 产品说明

Unit EncoderMotor Driver（U232）和 Unit 4EncoderMotor Driver（U233）分别通过 I2C 控制单路和四路编码电机。
本项目使用 Tab5 触摸屏演示 PWM、位置、速度及 RGB LED 控制，基于 ESP-IDF 5.5.5 和 LVGL 9.4.0。

## 相关链接

- Unit EncoderMotor Driver（U232）：[Document & Datasheet](https://docs.m5stack.com/en/products/sku/U232)
- Unit 4EncoderMotor Driver（U233）：[Document & Datasheet](https://docs.m5stack.com/en/products/sku/U233)

## 编译使用

模块接 Tab5 PORT.A（SDA：GPIO53，SCL：GPIO54），并连接电机及其供电。
默认配置适用于 ESP32-P4 v1.x。

加载 ESP-IDF 5.5.5 环境后，在项目根目录执行以下命令。

下载并烧录预编译的 [V1 固件](firmware/P359-UnitEncoderMotorDriver-ESP32P4-UserDemo-V1-20261016_0x0.bin)，烧录地址为 `0x0`：

```sh
python -m esptool --chip esp32p4 -p PORT -b 460800 write_flash \
  --flash_mode dio --flash_freq 80m --flash_size 16MB \
  0x0 firmware/P359-UnitEncoderMotorDriver-ESP32P4-UserDemo-V1-20261016_0x0.bin
```

如需从源码编译并烧录：

```sh
cd src
idf.py build
idf.py -p PORT flash monitor
```

将 `PORT` 替换为 Tab5 串口，按 Ctrl+] 退出监视器。

选择电机通道和 PWM、POSITION 或 SPEED 模式，调节目标后按 START。
POSITION 页可调节目标位置和最大电机输出（Max motor output，百分比）。
STOP 停止所选通道，STOP ALL 停止全部通道；DEVICE 提供设备信息、I2C 地址设置和 LED 控制。

切换控制模式时会停止全部通道并清零运动目标。
LIVE DEVICE 显示绿色时表示设备已连接。

## License

- M5Unit EncoderMotor Driver 用户示例 - [MIT](LICENSE)
