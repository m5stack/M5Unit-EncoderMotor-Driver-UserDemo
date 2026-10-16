# m5_unit_motor_driver

[English](README.md) | [简体中文](README_CN.md)

面向 U232（单通道）和 U233（四通道）的 ESP-IDF >=5.4 I2C 组件，
使用新版 I2C master API。Tab5 示例应用要求 ESP-IDF 5.5.5。

## 接入方式

调用方负责创建和初始化 I2C 总线：

```c
#include "m5_unit_motor_driver.h"

m5_motor_handle_t devices[2] = {0};
size_t count = 0;
esp_err_t err = m5_motor_discover(bus, devices, &count);
if (err == ESP_OK) {
    for (size_t i = 0; i < count; ++i) {
        uint8_t version;
        if (m5_motor_get_version(devices[i], &version) == ESP_OK) {
            /* 通过应用的显示锁或消息机制发布版本信息。 */
        }
        m5_motor_delete(&devices[i]);
    }
}
```

发现逻辑探测 0x2F 和 0x24，检查地址回显和固件版本，
再根据默认地址推断型号。协议没有产品 ID。
非默认地址须使用已知型号调用 `m5_motor_create()`。

组件借用总线，删除总线前须释放全部句柄。
同一句柄的所有操作由调用方串行执行。
调用为带有限超时的同步 I2C，不应在 LVGL 回调或持有显示锁时执行。
创建句柄和发现设备不会启动电机。

## 数据与控制

- 通道从 0 开始编号。
- PWM 与速度目标：-127～127，拒绝 -128。
- 位置和编码器：有符号 32 位小端数值。
- 速度单位：每 20 ms 的编码器计数增量。
- 位置环输出限幅：0～127 的原始幅值。
- RGB 接口：0xRRGGBB，亮度 0～255；总线按 B、G、R、dummy、brightness 发送。

全部接口见带注释的[公开头文件](include/m5_unit_motor_driver.h)。
支持读取遥测、控制参数、固件、UID 和 RGB；
写接口涵盖 PWM、模式、编码器原点、位置/速度目标、限幅、PID 和地址。

全部传输成功后才发布遥测快照。
四通道固件仅返回所选通道的响应，组件分别读取各通道的 PWM、速度和编码器。
读取失败时保留调用方原有快照。

`m5_motor_stop_all()` 清除速度目标、切换开环并将全部通道 PWM 置零；
即使发生失败也继续尝试其他通道，返回首个错误。
通信失败时不能确认实体电机已经停止。

## 持久地址修改

地址写入会保存 Flash。组件拒绝已占用地址、跳过相同地址，
预建新句柄后写入，等待 350 ms，再核对地址回显和 UID。
写入确认后，即使校验失败，句柄也跟随新地址。
传输失败时地址状态不确定，须显式重新打开或发现；
删除失败时保留句柄，供调用方重试。

Tab5 的 LIVE DEVICE 模式已将 UI 控制接入这些接口；
SIMULATION 使用模型，READ ONLY 禁用写操作。

## 协议依据与测试

依据内部固件 `dbf00770cddbaf6058454faa976d7e5ef4607e9d`，
V1 / 2026-08-27。组件保留独立许可证。

在应用 `src/` 目录执行：

```sh
cmake -S components/m5_unit_motor_driver/test_apps/host -B /tmp/motor-driver-tests
cmake --build /tmp/motor-driver-tests --parallel 4
ctest --test-dir /tmp/motor-driver-tests --output-on-failure
```

模拟传输测试在 ASan/UBSan 下覆盖协议值、通道偏移、传输失败、
设备发现、地址迁移及资源释放。
