#pragma once


#include <stdint.h>
#include "driver/i2c_master.h"


class Pi4ioe5v6408
{
public:
    Pi4ioe5v6408(i2c_master_bus_handle_t bus, uint8_t addr = 0x43);

    bool begin();

    // GPIO 配置
    void set_direction(uint8_t pin, bool direction);     // false = input, true = output
    void enable_pull(uint8_t pin, bool enablePull);      // 开启/关闭内部上拉/下拉
    void set_pull_mode(uint8_t pin, bool mode);          // false = down, true = up
    void set_high_impedance(uint8_t pin, bool enable);   // 高阻态使能

    // GPIO 读写
    bool get_write_value(uint8_t pin);
    void digital_write(uint8_t pin, bool level);
    bool digital_read(uint8_t pin);

    // 中断控制
    void reset_irq();
    void disable_irq();
    void enable_irq();

private:
    // 内部寄存器读写封装
    uint8_t read_reg(uint8_t reg);
    bool write_reg(uint8_t reg, uint8_t data);

    i2c_master_dev_handle_t i2c_dev;
    uint8_t device_address;
};
