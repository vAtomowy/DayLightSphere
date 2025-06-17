#ifndef DRV_INA219_H
#define DRV_INA219_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>

#if __has_include("drv_config.h")
#include "drv_config.h"
#endif

#define INA219_I2C_ADDR      0x45

#define INA219_REG_CONFIG    0x00
#define INA219_REG_SHUNT_V   0x01
#define INA219_REG_BUS_V     0x02

void i2c_scan(i2c_port_t port);
esp_err_t i2c_master_init(i2c_port_t i2c_port, gpio_num_t sda, gpio_num_t scl);
esp_err_t ina219_write_register(i2c_port_t i2c_port, uint8_t reg, uint16_t value);
esp_err_t ina219_read_register(i2c_port_t i2c_port, uint8_t reg, uint16_t *value);

#ifdef __cplusplus
}
#endif

#endif // DRV_INA219_H
