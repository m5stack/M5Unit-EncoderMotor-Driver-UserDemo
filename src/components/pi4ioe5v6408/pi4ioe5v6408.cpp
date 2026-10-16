/**********************************************************************



**********************************************************************/
#include "pi4ioe5v6408.hpp"
#include "esp_log.h"


static const char * TAG = "PI4IOE";


Pi4ioe5v6408::Pi4ioe5v6408(i2c_master_bus_handle_t i2c_bus, uint8_t dev_addr)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = 400000, // 400kHz
    };

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus: %s", esp_err_to_name(ret));
        i2c_dev = nullptr;
    }
}

bool Pi4ioe5v6408::begin()
{
    uint8_t id = read_reg(0x01);
    return (id != 0);
}

// false input, true output
void Pi4ioe5v6408::set_direction(uint8_t pin, bool direction)
{
    uint8_t reg = read_reg(0x03);
    if (direction) reg |= (1 << pin);
    else reg &= ~(1 << pin);
    write_reg(0x03, reg);
}

void Pi4ioe5v6408::enable_pull(uint8_t pin, bool enable)
{
    uint8_t reg = read_reg(0x0B);
    if (enable) reg |= (1 << pin);
    else reg &= ~(1 << pin);
    write_reg(0x0B, reg);
}

// false down, true up
void Pi4ioe5v6408::set_pull_mode(uint8_t pin, bool mode)
{
    uint8_t reg = read_reg(0x0D);
    if (mode) reg |= (1 << pin);
    else reg &= ~(1 << pin);
    write_reg(0x0D, reg);
}

void Pi4ioe5v6408::set_high_impedance(uint8_t pin, bool enable)
{
    uint8_t reg = read_reg(0x07);
    if (enable) reg |= (1 << pin);
    else reg &= ~(1 << pin);
    write_reg(0x07, reg);
}

bool Pi4ioe5v6408::get_write_value(uint8_t pin)
{
    uint8_t reg = read_reg(0x05);
    return (reg & (1 << pin)) != 0;
}

void Pi4ioe5v6408::digital_write(uint8_t pin, bool level)
{
    uint8_t reg = read_reg(0x05);
    if (level) reg |= (1 << pin);
    else reg &= ~(1 << pin);
    write_reg(0x05, reg);
}

bool Pi4ioe5v6408::digital_read(uint8_t pin)
{
    uint8_t reg = read_reg(0x0F);
    return (reg & (1 << pin)) != 0;
}

void Pi4ioe5v6408::reset_irq()
{
    (void)read_reg(0x13); // 读一下清除中断
}

void Pi4ioe5v6408::disable_irq()
{
    write_reg(0x11, 0xFF);
}

void Pi4ioe5v6408::enable_irq()
{
    write_reg(0x11, 0x00);
}

uint8_t Pi4ioe5v6408::read_reg(uint8_t reg)
{
    uint8_t data = 0;
    if (i2c_dev == nullptr) return 0;

    esp_err_t ret = i2c_master_transmit_receive(i2c_dev, &reg, 1, &data, 1, 200);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read reg 0x%02X: %s", reg, esp_err_to_name(ret));
        return 0;
    }
    return data;
}

bool Pi4ioe5v6408::write_reg(uint8_t reg, uint8_t data)
{
    if (i2c_dev == nullptr) return false;

    uint8_t buf[2] = {reg, data};
    esp_err_t ret = i2c_master_transmit(i2c_dev, buf, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write reg 0x%02X: %s", reg, esp_err_to_name(ret));
        return false;
    }
    return true;
}
