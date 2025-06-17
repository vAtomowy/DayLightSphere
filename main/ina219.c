#include "ina219.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define I2C_MASTER_FREQ_HZ    100000
#define I2C_MASTER_TIMEOUT_MS 1000

esp_err_t i2c_master_init(i2c_port_t i2c_port, gpio_num_t sda, gpio_num_t scl)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));
    return i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
}


void i2c_scan(i2c_port_t port) {
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            printf("Found device at 0x%02X\n", addr);
        }
    }
}

esp_err_t ina219_write_register(i2c_port_t i2c_port, uint8_t reg, uint16_t value)
{
    uint8_t data[3];
    data[0] = reg;
    data[1] = (value >> 8) & 0xFF;
    data[2] = value & 0xFF;

    return i2c_master_write_to_device(i2c_port, INA219_I2C_ADDR,
                                      data, sizeof(data),
                                      pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

esp_err_t ina219_read_register(i2c_port_t i2c_port, uint8_t reg, uint16_t *value)
{
    uint8_t data[2] = {0};

    esp_err_t err = i2c_master_write_read_device(i2c_port, INA219_I2C_ADDR,
                                                 &reg, 1, data, 2,
                                                 pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err != ESP_OK) return err;

    *value = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}
