/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_ldo_regulator.h"
#include "esp_vfs_fat.h"

// #include "usb/usb_host.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "esp_lcd_ili9881c.h"
#include "esp_lcd_st7121.h"
#include "esp_lcd_st7123.h"
#include "bsp/m5stack_tab5.h"
#include "bsp/display.h"
#include "bsp/touch.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_lcd_touch_st7123.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"
#include "ili9881_init_data.h"
#include "string.h"


static const char *TAG = "Tab5";


// Global SD card handler
static sdmmc_card_t *bsp_sdcard = NULL;
// sys i2c
static bool i2c_initialized = false;
static i2c_master_bus_handle_t i2c_handle = NULL;
// ext i2c
static bool ext_i2c_initialized = false;
static i2c_master_bus_handle_t ext_i2c_bus_handle = NULL;
// grove i2c
static bool grove_i2c_initialized = false;
static i2c_master_bus_handle_t grove_i2c_bus_handle = NULL;
// i2s
static i2s_chan_handle_t i2s_tx_chan = NULL;
static i2s_chan_handle_t i2s_rx_chan = NULL;
static const audio_codec_data_if_t *i2s_data_if = NULL;
static bool codec_initialized = false;
// display & touch
#define BSP_ST7123_TOUCH_I2C_ADDRESS    0x55
#define BSP_ST7123_FW_VERSION_REG       0x0000
static lv_indev_t *disp_indev = NULL;
static esp_lcd_touch_handle_t _tp = NULL;
static bool display_type_inited = false;
static bsp_display_type_t display_type = DISPLAY_ILI9881;
static bsp_tp_type_t tp_type = TP_GT911;


esp_err_t bsp_i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized) {
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = BSP_I2C_SDA,
        .scl_io_num = BSP_I2C_SCL,
        .i2c_port = BSP_I2C_NUM,
        .flags.enable_internal_pullup = true,
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_new_master_bus(&i2c_bus_conf, &i2c_handle));
    i2c_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(i2c_del_master_bus(i2c_handle));
    i2c_initialized = false;
    return ESP_OK;
}

i2c_master_bus_handle_t bsp_i2c_get_handle(void)
{
    return i2c_handle;
}

esp_err_t bsp_i2c_scan(i2c_master_bus_handle_t i2c_bus)
{
    esp_err_t ret;
    uint8_t address;

    printf("scan i2c device\n");
    printf("\n     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            address = i + j;
            ret = i2c_master_probe(i2c_bus, address, 50);
            if (ret == ESP_OK) {
                printf("%02x ", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }
    printf("\nscan i2c device finished\n");

    return ESP_OK;
}


//==================================================================================
// I/O Expander PI4IOE5V6416
//==================================================================================
#define I2C_DEV_ADDR_PI4IOE1  0x43  // addr pin low
#define I2C_DEV_ADDR_PI4IOE2  0x44  // addr pin high
#define I2C_MASTER_TIMEOUT_MS 50

static i2c_master_dev_handle_t i2c_dev_handle_pi4ioe1 = NULL;
static i2c_master_dev_handle_t i2c_dev_handle_pi4ioe2 = NULL;

// PI4IO registers
#define PI4IO_REG_CHIP_RESET 0x01
#define PI4IO_REG_IO_DIR     0x03
#define PI4IO_REG_OUT_SET    0x05
#define PI4IO_REG_OUT_H_IM   0x07
#define PI4IO_REG_IN_DEF_STA 0x09
#define PI4IO_REG_PULL_EN    0x0B
#define PI4IO_REG_PULL_SEL   0x0D
#define PI4IO_REG_IN_STA     0x0F
#define PI4IO_REG_INT_MASK   0x11
#define PI4IO_REG_IRQ_STA    0x13

#define setbit(x, y) ((x) |= (0x01 << (y)))
#define clrbit(x, y) ((x) &= ~(0x01 << (y)))

void bsp_io_expander_pi4ioe_init(i2c_master_bus_handle_t bus_handle)
{
    uint8_t write_buf[2] = {0};
    uint8_t read_buf[1]  = {0};

    /* */
    i2c_device_config_t dev_cfg1 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = I2C_DEV_ADDR_PI4IOE1,
        .scl_speed_hz    = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg1, &i2c_dev_handle_pi4ioe1));

    write_buf[0] = PI4IO_REG_CHIP_RESET;
    write_buf[1] = 0xFF;
    i2c_master_transmit(i2c_dev_handle_pi4ioe1, write_buf, 2, I2C_MASTER_TIMEOUT_MS);
    write_buf[0] = PI4IO_REG_CHIP_RESET;
    i2c_master_transmit_receive(i2c_dev_handle_pi4ioe1, write_buf, 1, read_buf, 1, I2C_MASTER_TIMEOUT_MS);
    write_buf[0] = PI4IO_REG_IO_DIR;
    write_buf[1] = 0b01111111;
    i2c_master_transmit(i2c_dev_handle_pi4ioe1, write_buf, 2, I2C_MASTER_TIMEOUT_MS);  // 0: input 1: output
    write_buf[0] = PI4IO_REG_OUT_H_IM;
    write_buf[1] = 0b00000000;
    i2c_master_transmit(i2c_dev_handle_pi4ioe1, write_buf, 2, I2C_MASTER_TIMEOUT_MS);  // 使用到的引脚关闭 High-Impedance
    write_buf[0] = PI4IO_REG_PULL_SEL;
    write_buf[1] = 0b01111111;
    i2c_master_transmit(i2c_dev_handle_pi4ioe1, write_buf, 2, I2C_MASTER_TIMEOUT_MS);  // pull up/down select, 0 down, 1 up
    write_buf[0] = PI4IO_REG_PULL_EN;
    write_buf[1] = 0b01111111;

    i2c_master_transmit(i2c_dev_handle_pi4ioe1, write_buf, 2, I2C_MASTER_TIMEOUT_MS);  // P7 中断使能 0 enable, 1 disable
    /* Output Port Register P1(SPK_EN), P2(EXT5V_EN), P4(LCD_RST), P5(TP_RST), P6(CAM)RST 输出高电平 */
    write_buf[0] = PI4IO_REG_OUT_SET;
    write_buf[1] = 0b01110110;
    i2c_master_transmit(i2c_dev_handle_pi4ioe1, write_buf, 2, I2C_MASTER_TIMEOUT_MS);

    /* */
    i2c_device_config_t dev_cfg2 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = I2C_DEV_ADDR_PI4IOE2,
        .scl_speed_hz    = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg2, &i2c_dev_handle_pi4ioe2));

    write_buf[0] = PI4IO_REG_CHIP_RESET;
    write_buf[1] = 0xFF;
    i2c_master_transmit(i2c_dev_handle_pi4ioe2, write_buf, 2, I2C_MASTER_TIMEOUT_MS);
    write_buf[0] = PI4IO_REG_CHIP_RESET;
    i2c_master_transmit_receive(i2c_dev_handle_pi4ioe2, write_buf, 1, read_buf, 1, I2C_MASTER_TIMEOUT_MS);
    write_buf[0] = PI4IO_REG_IO_DIR;
    write_buf[1] = 0b10111001;
    i2c_master_transmit(i2c_dev_handle_pi4ioe2, write_buf, 2, I2C_MASTER_TIMEOUT_MS);  // 0: input 1: output
    write_buf[0] = PI4IO_REG_OUT_H_IM;
    write_buf[1] = 0b00000110;
    i2c_master_transmit(i2c_dev_handle_pi4ioe2, write_buf, 2, I2C_MASTER_TIMEOUT_MS);  // 使用到的引脚关闭 High-Impedance
    write_buf[0] = PI4IO_REG_PULL_SEL;
    write_buf[1] = 0b10111001;
    i2c_master_transmit(i2c_dev_handle_pi4ioe2, write_buf, 2, I2C_MASTER_TIMEOUT_MS);  // pull up/down select, 0 down, 1 up
    write_buf[0] = PI4IO_REG_PULL_EN;
    write_buf[1] = 0b11111001;
    i2c_master_transmit(i2c_dev_handle_pi4ioe2, write_buf, 2, I2C_MASTER_TIMEOUT_MS);  // pull up/down enable, 0 disable, 1 enable
    write_buf[0] = PI4IO_REG_IN_DEF_STA;
    write_buf[1] = 0b01000000;
    i2c_master_transmit(i2c_dev_handle_pi4ioe2, write_buf, 2, I2C_MASTER_TIMEOUT_MS);  // P6 默认高电平
    write_buf[0] = PI4IO_REG_INT_MASK;
    write_buf[1] = 0b10111111;
    i2c_master_transmit(i2c_dev_handle_pi4ioe2, write_buf, 2, I2C_MASTER_TIMEOUT_MS);  // P6 中断使能 0 enable, 1 disable
    /* Output Port Register P0(WLAN_PWR_EN), P3(USB5V_EN), P7(CHG_EN) 输出高电平 */
    write_buf[0] = PI4IO_REG_OUT_SET;
    // write_buf[1] = 0b10001001;
    write_buf[1] = 0b00001001;

    i2c_master_transmit(i2c_dev_handle_pi4ioe2, write_buf, 2, I2C_MASTER_TIMEOUT_MS);
}

void bsp_reset_tp()
{
    ESP_LOGI(TAG, "reset tp");

    ESP_LOGI(TAG, "reset gpio %d", GPIO_NUM_23);
    gpio_reset_pin(GPIO_NUM_23);

    uint8_t write_buf[2] = {0};
    uint8_t read_buf[1]  = {0};

    write_buf[0] = PI4IO_REG_OUT_SET;
    i2c_master_transmit_receive(i2c_dev_handle_pi4ioe1, write_buf, 1, read_buf, 1, I2C_MASTER_TIMEOUT_MS);

    write_buf[0] = PI4IO_REG_OUT_SET;
    write_buf[1] = read_buf[0];
    clrbit(write_buf[1], 4);
    clrbit(write_buf[1], 5);
    i2c_master_transmit(i2c_dev_handle_pi4ioe1, write_buf, 2, I2C_MASTER_TIMEOUT_MS);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    write_buf[0] = PI4IO_REG_OUT_SET;
    write_buf[1] = read_buf[0];
    setbit(write_buf[1], 4);
    setbit(write_buf[1], 5);
    i2c_master_transmit(i2c_dev_handle_pi4ioe1, write_buf, 2, I2C_MASTER_TIMEOUT_MS);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

//==================================================================================
// sd card
//==================================================================================
#define BSP_LDO_PROBE_SD_CHAN       4
#define BSP_LDO_PROBE_SD_VOLTAGE_MV 3300
#define SDMMC_BUS_WIDTH (4)
#define SDCARD_SDMMC_HOST_SLOT (SDMMC_HOST_SLOT_0)
#define GPIO_SDMMC_DET  (GPIO_NUM_NC)
#define GPIO_SDMMC_CLK (GPIO_NUM_43)
#define GPIO_SDMMC_CMD (GPIO_NUM_44)
#define GPIO_SDMMC_D0  (GPIO_NUM_39)
#define GPIO_SDMMC_D1  (GPIO_NUM_40)
#define GPIO_SDMMC_D2  (GPIO_NUM_41)
#define GPIO_SDMMC_D3  (GPIO_NUM_42)

esp_err_t bsp_sdcard_mount(void)
{
    bsp_sdcard_cfg_t cfg = {0};
    return bsp_sdcard_sdmmc_mount(&cfg);
}

esp_err_t bsp_sdcard_unmount(void)
{
    esp_err_t ret = ESP_OK;

    ret |= esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
    bsp_sdcard = NULL;

    return ret;
}

sdmmc_card_t *bsp_sdcard_get_handle(void)
{
    return bsp_sdcard;
}

void bsp_sdcard_get_sdmmc_host(const int slot, sdmmc_host_t *config)
{
    assert(config);
    memset(config, 0, sizeof(sdmmc_host_t));

    esp_err_t ret_val = ESP_OK;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot         = slot;
    host.max_freq_khz                   = SDMMC_FREQ_HIGHSPEED;
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = BSP_LDO_PROBE_SD_CHAN,  // `LDO_VO4` is used as the SDMMC IO power
    };
    static sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;

    if (pwr_ctrl_handle == NULL) {
        ret_val = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
        if (ret_val != ESP_OK) {
            ESP_LOGE(TAG, "Failed to new an on-chip ldo power control driver");
            return;
        }
    }
    host.pwr_ctrl_handle = pwr_ctrl_handle;

    memcpy(config, &host, sizeof(sdmmc_host_t));
}

void bsp_sdcard_get_sdspi_host(const int slot, sdmmc_host_t *config)
{
    assert(config);
    memset(config, 0, sizeof(sdmmc_host_t));
    ESP_LOGE(TAG, "SD card SPI mode is not supported yet");
}

void bsp_sdcard_sdmmc_get_slot(const int slot, sdmmc_slot_config_t *config)
{
    assert(config);
    memset(config, 0, sizeof(sdmmc_slot_config_t));

    config->width = SDMMC_BUS_WIDTH;
    config->clk   = GPIO_SDMMC_CLK;
    config->cmd   = GPIO_SDMMC_CMD;
    config->d0    = GPIO_SDMMC_D0;
    config->d1    = GPIO_SDMMC_D1;
    config->d2    = GPIO_SDMMC_D2;
    config->d3    = GPIO_SDMMC_D3;
}

void bsp_sdcard_sdspi_get_slot(const spi_host_device_t spi_host, sdspi_device_config_t *config)
{
    assert(config);
    memset(config, 0, sizeof(sdspi_device_config_t));
    ESP_LOGE(TAG, "SD card SPI mode is not supported yet");
}

esp_err_t bsp_sdcard_sdmmc_mount(bsp_sdcard_cfg_t *cfg)
{
    esp_err_t ret_val = ESP_OK;

    if (NULL != bsp_sdcard) {
        return ESP_ERR_INVALID_STATE;
    }

    /**
     * @brief Use settings defined above to initialize SD card and mount FAT filesystem.
     *   Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
     *   Please check its source code and implement error recovery when developing
     *   production applications.
     *
     */
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    bsp_sdcard_get_sdmmc_host(SDCARD_SDMMC_HOST_SLOT, &host);

    /**
     * @brief This initializes the slot without card detect (CD) and write protect (WP) signals.
     *   Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
     *
     */
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    bsp_sdcard_sdmmc_get_slot(SDCARD_SDMMC_HOST_SLOT, &slot_config);

    /**
     * @brief Options for mounting the filesystem.
     *   If format_if_mount_failed is set to true, SD card will be partitioned and
     *   formatted in case when mounting fails.
     */
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 25,
        .allocation_unit_size = 16 * 1024
    };

    ret_val = esp_vfs_fat_sdmmc_mount(CONFIG_BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);

    /* Check for SDMMC mount result. */
    if (ret_val != ESP_OK) {
        if (ret_val == ESP_FAIL) {
            ESP_LOGE(TAG,
                    "Failed to mount filesystem. "
                    "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG,
                    "Failed to initialize the card (%s). "
                    "Make sure SD card lines have pull-up resistors in place.",
                    esp_err_to_name(ret_val));
        }
        return ret_val;
    }

    /* Card has been initialized, print its properties. */
    sdmmc_card_print_info(stdout, bsp_sdcard);

    return ret_val;
}

esp_err_t bsp_sdcard_sdspi_mount(bsp_sdcard_cfg_t *cfg)
{
    ESP_LOGE(TAG, "SD card SPIFFS mode is not supported yet");
    return ESP_ERR_NOT_SUPPORTED;
}

//==================================================================================
// spiffs
//==================================================================================
esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path       = CONFIG_BSP_SPIFFS_MOUNT_POINT,
        .partition_label = CONFIG_BSP_SPIFFS_PARTITION_LABEL,
        .max_files       = CONFIG_BSP_SPIFFS_MAX_FILES,
#ifdef CONFIG_BSP_SPIFFS_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
    };

    esp_err_t ret_val = esp_vfs_spiffs_register(&conf);

    BSP_ERROR_CHECK_RETURN_ERR(ret_val);

    size_t total = 0, used = 0;
    ret_val = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret_val;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister(CONFIG_BSP_SPIFFS_PARTITION_LABEL);
}

//==================================================================================
// audio es7210 + es8388
//==================================================================================
typedef esp_err_t (*bsp_i2s_read_fn)(void *audio_buffer, size_t len, size_t *bytes_read, uint32_t timeout_ms);
typedef esp_err_t (*bsp_i2s_write_fn)(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms);
typedef esp_err_t (*bsp_codec_set_in_gain_fn)(float gain);
typedef esp_err_t (*bsp_codec_mute_fn)(bool enable);
typedef int (*bsp_codec_volume_fn)(int volume);
typedef esp_err_t (*bsp_codec_get_volume_fn)(void);
typedef esp_err_t (*bsp_codec_reconfig_fn)(uint32_t rate, uint32_t bps, i2s_slot_mode_t ch);
typedef esp_err_t (*bsp_i2s_reconfig_clk_fn)(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch);

typedef struct {
    bsp_i2s_read_fn i2s_read;
    bsp_i2s_write_fn i2s_write;
    bsp_codec_mute_fn set_mute;
    bsp_codec_volume_fn set_volume;
    bsp_codec_get_volume_fn get_volume;
    bsp_codec_set_in_gain_fn set_in_gain;
    bsp_codec_reconfig_fn codec_reconfig_fn;
    bsp_i2s_reconfig_clk_fn i2s_reconfig_clk_fn;
} bsp_codec_config_t;

static void bsp_codec_init(void);
static bsp_codec_config_t *bsp_get_codec_handle(void);
static uint8_t bsp_codec_feed_channel(void);

static esp_codec_dev_handle_t play_dev_handle;
static esp_codec_dev_handle_t record_dev_handle;
static bsp_codec_config_t g_codec_handle;
static int volume;

/* Can be used for `i2s_std_gpio_config_t` and/or `i2s_std_config_t` initialization */
#define BSP_I2S_GPIO_CFG                                                                                           \
    {                                                                                                              \
        .mclk = BSP_I2S_MCLK, .bclk = BSP_I2S_SCLK, .ws = BSP_I2S_LCLK, .dout = BSP_I2S_DOUT, .din = BSP_I2S_DSIN, \
        .invert_flags = {                                                                                          \
            .mclk_inv = false,                                                                                     \
            .bclk_inv = false,                                                                                     \
            .ws_inv   = false,                                                                                     \
        },                                                                                                         \
    }

/* This configuration is used by default in `bsp_extra_audio_init()` */
#define BSP_I2S_DUPLEX_MONO_CFG(_sample_rate)                                                         \
    {                                                                                                 \
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(_sample_rate),                                         \
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO), \
        .gpio_cfg = BSP_I2S_GPIO_CFG,                                                                 \
    }

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config)
{
    if (i2s_tx_chan && i2s_rx_chan) {
        /* Audio was initialized before */
        return ESP_OK;
    }

    /* Setup I2S peripheral */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear        = true;  // Auto clear the legacy data in the DMA buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx_chan, &i2s_rx_chan));

    /* Setup I2S channels */
    // const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(16000);
    const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(48000);
    const i2s_std_config_t *p_i2s_cfg      = &std_cfg_default;
    if (i2s_config != NULL) {
        p_i2s_cfg = i2s_config;
    }

    if (i2s_tx_chan != NULL) {
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx_chan, p_i2s_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx_chan));
    }

    i2s_tdm_config_t tdm_cfg = {
        .clk_cfg =
        {
            .sample_rate_hz  = (uint32_t)48000,
            .clk_src         = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple   = I2S_MCLK_MULTIPLE_256,
            .bclk_div        = 8,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode      = I2S_SLOT_MODE_STEREO,
            .slot_mask      = (I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3),
            .ws_width       = I2S_TDM_AUTO_WS_WIDTH,
            .ws_pol         = false,
            .bit_shift      = true,
            .left_align     = false,
            .big_endian     = false,
            .bit_order_lsb  = false,
            .skip_mask      = false,
            .total_slot     = I2S_TDM_AUTO_SLOT_NUM
        },
        .gpio_cfg = BSP_I2S_GPIO_CFG,
    };

    if (i2s_rx_chan != NULL) {
        ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(i2s_rx_chan, &tdm_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(i2s_rx_chan));
    }

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port      = CONFIG_BSP_I2S_NUM,
        .tx_handle = i2s_tx_chan,
        .rx_handle = i2s_rx_chan,
    };
    i2s_data_if = audio_codec_new_i2s_data(&i2s_cfg);

    return ESP_OK;
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    static esp_codec_dev_handle_t codec = NULL;
    if (codec) {
        return codec;
    }

    if (i2s_data_if == NULL) {
        /* Initilize I2C */
        bsp_i2c_init();
        /* Configure I2S peripheral and Power Amplifier */
        bsp_audio_init(NULL);
    }
    assert(i2s_data_if);

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    i2c_master_bus_handle_t i2c_bus_handle = bsp_i2c_get_handle();
    audio_codec_i2c_cfg_t i2c_cfg          = {
        .port       = BSP_I2C_NUM,
        .addr       = ES8388_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_bus_handle,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    BSP_NULL_CHECK(i2c_ctrl_if, NULL);

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage        = 5.0,
        .codec_dac_voltage = 3.3,
    };

    es8388_codec_cfg_t es8388_cfg = {
        .codec_mode  = ESP_CODEC_DEV_WORK_MODE_DAC,
        .master_mode = false,
        .ctrl_if     = i2c_ctrl_if,
        .pa_pin      = -1,  // PI4IOE1 P1 控制
    };
    const audio_codec_if_t *es8388_dev = es8388_codec_new(&es8388_cfg);
    BSP_NULL_CHECK(es8388_dev, NULL);

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = es8388_dev,
        .data_if  = i2s_data_if,
    };
    codec = esp_codec_dev_new(&codec_dev_cfg);
    BSP_NULL_CHECK(codec, NULL);

    return codec;
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    if (i2s_data_if == NULL) {
        /* Initilize I2C */
        ESP_ERROR_CHECK(bsp_i2c_init());
        /* Configure I2S peripheral and Power Amplifier */
        ESP_ERROR_CHECK(bsp_audio_init(NULL));
        // i2s_data_if = bsp_get_codec_data_if();
    }
    assert(i2s_data_if);

    i2c_master_bus_handle_t i2c_bus_handle = bsp_i2c_get_handle();
    audio_codec_i2c_cfg_t i2c_cfg          = {
        .port       = BSP_I2C_NUM,
        .addr       = ES7210_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_bus_handle,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    BSP_NULL_CHECK(i2c_ctrl_if, NULL);

    es7210_codec_cfg_t es7210_cfg = {
        .ctrl_if = i2c_ctrl_if,  // Codec Control interface
    };
    es7210_cfg.mic_selected            = ES7210_SEL_MIC1 | ES7210_SEL_MIC2 | ES7210_SEL_MIC3 | ES7210_SEL_MIC4;
    const audio_codec_if_t *es7210_dev = es7210_codec_new(&es7210_cfg);
    BSP_NULL_CHECK(es7210_dev, NULL);

    esp_codec_dev_cfg_t codec_es7210_dev_cfg = {
        .dev_type =
        ESP_CODEC_DEV_TYPE_IN,   // Codec device type: Codec input device like ADC (capture data from microphone)
        .codec_if = es7210_dev,  // Codec interface
        .data_if  = i2s_data_if, // Codec data interface
    };

    return esp_codec_dev_new(&codec_es7210_dev_cfg);
}

static esp_err_t bsp_i2s_read(void *audio_buffer, size_t len, size_t *bytes_read, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ret           = esp_codec_dev_read(record_dev_handle, audio_buffer, len);
    *bytes_read   = len;
    return ret;
}

static esp_err_t bsp_i2s_write(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    esp_err_t ret  = ESP_OK;
    ret            = esp_codec_dev_write(play_dev_handle, audio_buffer, len);
    *bytes_written = len;
    return ret;
}

static esp_err_t bsp_codec_set_in_gain(float gain)
{
    return esp_codec_dev_set_in_gain(record_dev_handle, gain);
}

static esp_err_t bsp_codec_set_mute(bool enable)
{
    esp_err_t ret = ESP_OK;
    ret           = esp_codec_dev_set_out_mute(play_dev_handle, enable);
    return ret;
}

static esp_err_t bsp_codec_set_volume(int v)
{
    esp_err_t ret = ESP_OK;

    if (v <= 0) {
        volume = 0;
        ret    = esp_codec_dev_set_out_mute(play_dev_handle, true);
    } else {
        volume = v;
        ret    = esp_codec_dev_set_out_mute(play_dev_handle, false);
        ret |= esp_codec_dev_set_out_vol(play_dev_handle, volume);
    }

    return ret;
}

static int bsp_codec_get_volume(void)
{
    return volume;
}

static bsp_codec_config_t *bsp_get_codec_handle(void)
{
    return &g_codec_handle;
}

static esp_err_t bsp_codec_es8388_set(uint32_t rate, uint32_t bps, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;

    esp_codec_dev_sample_info_t fs = {
        .sample_rate     = rate,
        .channel         = ch,
        .bits_per_sample = bps,
    };

    if (play_dev_handle) {
        ret = esp_codec_dev_close(play_dev_handle);
    }
    ret = esp_codec_dev_open(play_dev_handle, &fs);

    return ret;
}

static esp_err_t bsp_codec_es7210_set(uint32_t rate, uint32_t bps, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;

    esp_codec_dev_sample_info_t fs = {
        .sample_rate     = rate,
        .channel         = ch,
        .bits_per_sample = bps,
    };

    if (record_dev_handle) {
        ret = esp_codec_dev_close(record_dev_handle);
    }
    ret = esp_codec_dev_open(record_dev_handle, &fs);

    return ret;
}

static void bsp_codec_init(void)
{
    play_dev_handle = bsp_audio_codec_speaker_init();
    assert((play_dev_handle) && "play_dev_handle not initialized");

    record_dev_handle = bsp_audio_codec_microphone_init();
    assert((record_dev_handle) && "record_dev_handle not initialized");

    bsp_codec_es7210_set(48000, 16, 4);
    bsp_codec_es8388_set(48000, 16, 2);

    bsp_codec_config_t *codec_cfg  = bsp_get_codec_handle();
    codec_cfg->i2s_read            = bsp_i2s_read;
    codec_cfg->i2s_write           = bsp_i2s_write;
    codec_cfg->set_mute            = bsp_codec_set_mute;
    codec_cfg->set_volume          = bsp_codec_set_volume;
    codec_cfg->get_volume          = bsp_codec_get_volume;
    codec_cfg->set_in_gain         = bsp_codec_set_in_gain;
    codec_cfg->codec_reconfig_fn   = bsp_codec_es7210_set;
    codec_cfg->i2s_reconfig_clk_fn = bsp_codec_es8388_set;

    codec_cfg->set_volume(80);
}

static uint8_t bsp_codec_feed_channel(void)
{
    return 3;  // 2*mic_num + ref_num
}

const audio_codec_data_if_t *bsp_audio_get_codec_itf(void)
{
    return i2s_data_if;
}


//==================================================================================
// display & touch
//==================================================================================
static void bsp_detect_display_tp_type(void)
{
    if (display_type_inited) {
        return;
    }

    /* Check I2C address 0x55 for ST7121/ST7123 */
    esp_err_t ret = i2c_master_probe(i2c_handle, BSP_ST7123_TOUCH_I2C_ADDRESS, 200);
    if (ret == ESP_OK) {
        /* Read register 0x00 to determine type */
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_ST7123_CONFIG();
        tp_io_config.scl_speed_hz = 100000;
        if (esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle) == ESP_OK) {
            uint8_t fw_version = 0;
            if (esp_lcd_panel_io_rx_param(tp_io_handle, BSP_ST7123_FW_VERSION_REG, &fw_version, 1) == ESP_OK) {
                if (fw_version == 1) {
                    display_type = DISPLAY_ST7121;
                    tp_type = TP_ST7121;
                    ESP_LOGI(TAG, "Detected ST7121 (FW version: %u)", fw_version);
                } else if (fw_version == 3) {
                    display_type = DISPLAY_ST7123;
                    tp_type = TP_ST7123;
                    ESP_LOGI(TAG, "Detected ST7123 (FW version: %u)", fw_version);
                }
            }
            esp_lcd_panel_io_del(tp_io_handle);
        }
        display_type_inited = true;
        return;
    }

    /* No device at 0x55, default to ILI9881 (GT911) */
    display_type = DISPLAY_ILI9881;
    tp_type = TP_GT911;
    display_type_inited = true;
    ESP_LOGI(TAG, "No device at 0x%02X, defaulting to ILI9881 (GT911)", BSP_ST7123_TOUCH_I2C_ADDRESS);
}

bsp_display_type_t bsp_get_display_type(void)
{
    bsp_detect_display_tp_type();
    return display_type;
}

bsp_tp_type_t bsp_get_tp_type(void)
{
    bsp_detect_display_tp_type();
    return tp_type;
}

// Bit number used to represent command and parameter
#define LCD_LEDC_CH LEDC_CHANNEL_1  // CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH
esp_err_t bsp_display_brightness_init(void)
{
    // Setup LEDC peripheral for PWM backlight control
    const ledc_timer_config_t lcd_backlight_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&lcd_backlight_timer));

    const ledc_channel_config_t lcd_backlight_channel = {
        .gpio_num   = BSP_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LCD_LEDC_CH,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&lcd_backlight_channel));

    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent > 100) {
        brightness_percent = 100;
    }
    if (brightness_percent < 0) {
        brightness_percent = 0;
    }

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness_percent);
    uint32_t duty_cycle = (4095 * brightness_percent) / 100;  // LEDC resolution set to 12bits, thus: 100% = 4095
    BSP_ERROR_CHECK_RETURN_ERR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, duty_cycle));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH));
    return ESP_OK;
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

static esp_err_t bsp_enable_dsi_phy_power(void)
{
#if BSP_MIPI_DSI_PHY_PWR_LDO_CHAN > 0
    // Turn on the power for MIPI DSI PHY, so it can go from "No Power" state to "Shutdown" state
    static esp_ldo_channel_handle_t phy_pwr_chan = NULL;
    esp_ldo_channel_config_t ldo_cfg             = {
        .chan_id    = BSP_MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = BSP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_cfg, &phy_pwr_chan), TAG, "Acquire LDO channel for DPHY failed");
    ESP_LOGI(TAG, "MIPI DSI PHY Powered on");
#endif  // BSP_MIPI_DSI_PHY_PWR_LDO_CHAN > 0

    return ESP_OK;
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel,
                        esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;
    bsp_lcd_handles_t handles;
    ret = bsp_display_new_with_handles(config, &handles);

    *ret_panel = handles.panel;
    *ret_io    = handles.io;

    return ret;
}

esp_err_t bsp_display_new_with_handles(const bsp_display_config_t *config, bsp_lcd_handles_t *ret_handles)
{
    esp_err_t ret                     = ESP_OK;
    esp_lcd_panel_io_handle_t io      = NULL;
    esp_lcd_panel_handle_t disp_panel = NULL;
    bsp_display_type_t detected_display = bsp_get_display_type();
    bool use_st7121                   = (detected_display == DISPLAY_ST7121);
    bool use_st7123                   = (detected_display == DISPLAY_ST7123);

    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");
    ESP_RETURN_ON_ERROR(bsp_enable_dsi_phy_power(), TAG, "DSI PHY power failed");

    /* create MIPI DSI bus first, it will initialize the DSI PHY as well */
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t bus_config   = {
        .bus_id             = 0,
        .num_data_lanes     = BSP_LCD_MIPI_DSI_LANE_NUM,
        .phy_clk_src        = 0, // Let IDF select the revision-correct DSI PLL source.
        .lane_bit_rate_mbps = BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus), TAG, "New DSI bus init failed");

    ESP_LOGI(TAG, "Install MIPI DSI LCD control panel");
    // we use DBI interface to send LCD commands and parameters
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits    = 8,  // according to the LCD spec
        .lcd_param_bits  = 8,  // according to the LCD spec
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io), err, TAG, "New panel IO failed");

    if (use_st7121 || use_st7123) {
        ESP_LOGI(TAG, "Install LCD driver of %s", use_st7121 ? "st7121" : "st7123");
        esp_lcd_dpi_panel_config_t dpi_config = {
            .virtual_channel    = 0,
            .dpi_clk_src        = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
            .dpi_clock_freq_mhz = BSP_LCD_MIPI_DSI_DPI_CLK_MHZ,
            .pixel_format       = LCD_COLOR_PIXEL_FORMAT_RGB565,
            .num_fbs            = 1,
            .video_timing = {
                .h_size            = BSP_LCD_H_RES,
                .v_size            = BSP_LCD_V_RES,
                .hsync_back_porch  = BSP_LCD_ST7121_HSYNC_BACK_PORCH,
                .hsync_pulse_width = BSP_LCD_ST7121_HSYNC_PULSE_WIDTH,
                .hsync_front_porch = BSP_LCD_ST7121_HSYNC_FRONT_PORCH,
                .vsync_back_porch  = BSP_LCD_ST7121_VSYNC_BACK_PORCH,
                .vsync_pulse_width = BSP_LCD_ST7121_VSYNC_PULSE_WIDTH,
                .vsync_front_porch = BSP_LCD_ST7121_VSYNC_FRONT_PORCH,
            },
            .flags.use_dma2d = true,
        };

        if (use_st7123) {
            dpi_config.video_timing.hsync_back_porch  = BSP_LCD_ST7123_HSYNC_BACK_PORCH;
            dpi_config.video_timing.hsync_pulse_width = BSP_LCD_ST7123_HSYNC_PULSE_WIDTH;
            dpi_config.video_timing.hsync_front_porch = BSP_LCD_ST7123_HSYNC_FRONT_PORCH;
            dpi_config.video_timing.vsync_back_porch  = BSP_LCD_ST7123_VSYNC_BACK_PORCH;
            dpi_config.video_timing.vsync_pulse_width = BSP_LCD_ST7123_VSYNC_PULSE_WIDTH;
            dpi_config.video_timing.vsync_front_porch = BSP_LCD_ST7123_VSYNC_FRONT_PORCH;
        }

        st7123_vendor_config_t vendor_config = {
            .init_cmds = NULL,
            .init_cmds_size = 0,
            .mipi_config = {
                .dsi_bus    = mipi_dsi_bus,
                .dpi_config = &dpi_config,
                .lane_num   = BSP_LCD_MIPI_DSI_LANE_NUM,
            },
        };

        const esp_lcd_panel_dev_config_t lcd_dev_config = {
            .bits_per_pixel = 16,
            .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
            .data_endian    = LCD_RGB_DATA_ENDIAN_LITTLE,
            .reset_gpio_num = -1,
            .vendor_config  = &vendor_config,
        };
        if (use_st7121) {
            ESP_ERROR_CHECK(esp_lcd_new_panel_st7121(io, &lcd_dev_config, &disp_panel));
        } else {
            ESP_ERROR_CHECK(esp_lcd_new_panel_st7123(io, &lcd_dev_config, &disp_panel));
        }
    } else {
        ESP_LOGI(TAG, "Install LCD driver of ili9881c");
        esp_lcd_dpi_panel_config_t dpi_config = {
            .virtual_channel    = 0,
            .dpi_clk_src        = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
            .dpi_clock_freq_mhz = BSP_LCD_MIPI_DSI_DPI_CLK_MHZ,
            .pixel_format       = LCD_COLOR_PIXEL_FORMAT_RGB565,
            .num_fbs            = 1,
            .video_timing = {
                .h_size            = BSP_LCD_H_RES,
                .v_size            = BSP_LCD_V_RES,
                .hsync_back_porch  = BSP_LCD_ILI9881_HSYNC_BACK_PORCH,
                .hsync_pulse_width = BSP_LCD_ILI9881_HSYNC_PULSE_WIDTH,
                .hsync_front_porch = BSP_LCD_ILI9881_HSYNC_FRONT_PORCH,
                .vsync_back_porch  = BSP_LCD_ILI9881_VSYNC_BACK_PORCH,
                .vsync_pulse_width = BSP_LCD_ILI9881_VSYNC_PULSE_WIDTH,
                .vsync_front_porch = BSP_LCD_ILI9881_VSYNC_FRONT_PORCH,
            },
            .flags.use_dma2d = true,
        };

        ili9881c_vendor_config_t vendor_config = {
            .init_cmds = tab5_lcd_ili9881c_specific_init_code_default,
            .init_cmds_size = sizeof(tab5_lcd_ili9881c_specific_init_code_default) /
            sizeof(tab5_lcd_ili9881c_specific_init_code_default[0]),
            .mipi_config    = {
                .dsi_bus    = mipi_dsi_bus,
                .dpi_config = &dpi_config,
                .lane_num   = BSP_LCD_MIPI_DSI_LANE_NUM,
            },
        };

        const esp_lcd_panel_dev_config_t lcd_dev_config = {
            .bits_per_pixel = 16,
            .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
            .reset_gpio_num = -1,
            .vendor_config  = &vendor_config,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9881c(io, &lcd_dev_config, &disp_panel));
    }
    ESP_ERROR_CHECK(esp_lcd_panel_reset(disp_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(disp_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(disp_panel, true));

    /* Return all handles */
    ret_handles->io = io;
    ret_handles->mipi_dsi_bus = mipi_dsi_bus;
    ret_handles->panel = disp_panel;
    ret_handles->control = NULL;
    ESP_LOGI(TAG, "Display initialized with resolution %dx%d", BSP_LCD_H_RES, BSP_LCD_V_RES);

    return ret;

err:
    if (disp_panel) {
        esp_lcd_panel_del(disp_panel);
    }
    if (io) {
        esp_lcd_panel_io_del(io);
    }
    if (mipi_dsi_bus) {
        esp_lcd_del_dsi_bus(mipi_dsi_bus);
    }
    return ret;
}

esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    /* Initilize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());

    /* Initialize touch */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
        .rst_gpio_num = BSP_LCD_TOUCH_RST,
        .int_gpio_num = BSP_LCD_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy  = 0,
            .mirror_x = (display_type == DISPLAY_ST7123),
            .mirror_y = (display_type == DISPLAY_ST7123),
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    bsp_detect_display_tp_type();
    if (tp_type == TP_ST7123 || tp_type == TP_ST7121) {
        esp_lcd_panel_io_i2c_config_t tp_io_config = {
            .dev_addr = BSP_ST7123_TOUCH_I2C_ADDRESS,
            .control_phase_bytes = 1,
            .dc_bit_offset = 0,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .scl_speed_hz = 400000,
        };
        ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle), TAG, "");
        return esp_lcd_touch_new_i2c_st7123(tp_io_handle, &tp_cfg, ret_touch);
    }

    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    tp_io_config.dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP;
    tp_io_config.scl_speed_hz = 400000;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, ret_touch);
}

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *bsp_display_lcd_init(const bsp_display_lcd_config_t *cfg)
{
    assert(cfg != NULL);
    bsp_lcd_handles_t lcd_panels;
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new_with_handles(NULL, &lcd_panels));

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle      = lcd_panels.io,
        .panel_handle   = lcd_panels.panel,
        .control_handle = lcd_panels.control,
        .buffer_size    = cfg->buffer_size,
        .double_buffer  = cfg->double_buffer,
        .hres           = BSP_LCD_H_RES,
        .vres           = BSP_LCD_V_RES,
        .monochrome     = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy  = false,
            .mirror_x = false,
            .mirror_y = false,
        },
#if LVGL_VERSION_MAJOR >= 9
        .color_format = LV_COLOR_FORMAT_RGB565,
#endif
        .flags = {
            .buff_dma    = cfg->flags.buff_dma,
            .buff_spiram = cfg->flags.buff_spiram,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = (BSP_LCD_BIGENDIAN ? true : false),
#endif
#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
            .sw_rotate = false, /* Avoid tearing is not supported for SW rotation */
#else
            .sw_rotate   = cfg->flags.sw_rotate, /* Only SW rotation is supported for 90° and 270° */
#endif
#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH
            .full_refresh = true,
#elif CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
            .direct_mode = true,
#endif
        }
    };

    const lvgl_port_display_dsi_cfg_t dpi_cfg = {.flags = {
#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
            .avoid_tearing = true,
#else
            .avoid_tearing = false,
#endif
        }
    };

    return lvgl_port_add_disp_dsi(&disp_cfg, &dpi_cfg);
}

esp_lcd_touch_handle_t bsp_display_get_touch_handle(void)
{
    return _tp;
}

static void lvgl_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    if (_tp == NULL) {
        data->state = LV_INDEV_STATE_REL;
        return;
    }

    uint16_t touch_x[5];
    uint16_t touch_y[5];
    uint16_t touch_strength[5];
    uint8_t touch_cnt = 0;

    esp_lcd_touch_read_data(_tp);
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(_tp, touch_x, touch_y, touch_strength, &touch_cnt, 5);

    if (!touchpad_pressed) {
        data->state = LV_INDEV_STATE_REL;
    } else {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = touch_x[0];
        data->point.y = touch_y[0];
        // ESP_LOGI(TAG, "touch cnt=%u", touch_cnt);
        // for (uint8_t i = 0; i < touch_cnt; ++i) {
        //     ESP_LOGI(TAG, "touch[%u]=(%u,%u) strength=%u", i, touch_x[i], touch_y[i], touch_strength[i]);
        // }
    }
}

static lv_indev_t *bsp_display_indev_init(lv_display_t *disp)
{
    BSP_ERROR_CHECK_RETURN_NULL(bsp_touch_new(NULL, &_tp));
    assert(_tp);
    esp_lcd_touch_exit_sleep(_tp);  // !!!

    if (tp_type == TP_GT911) {
        ESP_LOGI(TAG, "Add touch gt911 input (for selected screen)");
        disp_indev = lv_indev_create();
        lv_indev_set_type(disp_indev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(disp_indev, lvgl_read_cb);
        lv_indev_set_display(disp_indev, disp);
        return disp_indev;
    } else { // ST7121 or ST7123
        ESP_LOGI(TAG, "Add touch st7121/st7123 input (for selected screen)");
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = disp,
            .handle = _tp,
        };
        return lvgl_port_add_touch(&touch_cfg);
    }
}

lv_display_t *bsp_display_start(void)
{
    bsp_i2c_init();
    i2c_master_bus_handle_t i2c_bus_handle = bsp_i2c_get_handle();
    bsp_i2c_scan(i2c_bus_handle);
    bsp_io_expander_pi4ioe_init(i2c_bus_handle);

    bsp_reset_tp();
    vTaskDelay(pdMS_TO_TICKS(100));
    bsp_i2c_scan(i2c_bus_handle);
    vTaskDelay(pdMS_TO_TICKS(100));

    bsp_display_lcd_config_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
            .sw_rotate = true,
        }
    };
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_lcd_config_t *cfg)
{
    lv_display_t *disp;
    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());
    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);
    if (display_type == DISPLAY_ST7123) {
        ESP_LOGI(TAG, "Display rotation: 180");
        lv_disp_set_rotation(disp, LV_DISPLAY_ROTATION_180);
    } else {
        ESP_LOGI(TAG, "Display rotation: 0");
        lv_disp_set_rotation(disp, LV_DISPLAY_ROTATION_0);
    }
    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);
    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev;
}

void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}
#endif  // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
