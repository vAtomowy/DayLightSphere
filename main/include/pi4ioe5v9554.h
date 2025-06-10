#ifdef __cplusplus
extern "C" {
#endif

#ifndef DRV_PORT_EXPANDER_PI4IOE5V9554_H
#define DRV_PORT_EXPANDER_PI4IOE5V9554_H

#if __has_include ("drv_config.h")
#include "drv_config.h"
#endif

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

/**
 * @brief Maximum number of instances to create
 * 
 */
#ifndef RF_PI4IOE5V9554_MAX_INSTANCES
    #define RF_PI4IOE5V9554_MAX_INSTANCES 4
#endif

/**
 * @brief Create an instance (static)
 * 
 */
#define RF_PI4IOE5V9554_CONFIG_STATIC_INSTANCE(name, i2c, addr, isr, handler) \
    static rf_pi4ioe5v9554_cfg_t name = {                                     \
        .i2c_num = i2c,                                                       \
        .i2c_addr = addr,                                                     \
        .isr_pin = isr,                                                       \
        .isr_handler = handler                                                \
    }

/**
 * @brief Create an instance (no static)
 * 
 */
#define RF_PI4IOE5V9554_CONFIG_INSTANCE(name, i2c, addr, isr, handler) \
    rf_pi4ioe5v9554_cfg_t name = {                                     \
        .i2c_num = i2c,                                                \
        .i2c_addr = addr,                                              \
        .isr_pin = isr,                                                \
        .isr_handler = handler                                         \
    }

/**
 * @brief I2C Configuration
 * 
 */
#define RF_PI4IOE5V9554_CONFIG_I2C(i2c_num, sda, scl)                          \
    i2c_config_t pi4ioe5v9554_i2c_cfg = {                                      \
        .mode = I2C_MODE_MASTER,                                               \
        .sda_io_num = sda,                                                     \
        .scl_io_num = scl,                                                     \
        .sda_pullup_en = GPIO_PULLUP_ENABLE,                                   \
        .scl_pullup_en = GPIO_PULLUP_ENABLE,                                   \
        .master.clk_speed = 100000,                                            \
    };                                                                         \
    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &pi4ioe5v9554_i2c_cfg));         \
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, pi4ioe5v9554_i2c_cfg.mode, 0, 0, 0))

/**
 * @brief GPIO Configuration (interrupt)
 * 
 */
#define RF_PI4IOE5V9554_CONFIG_GPIO(pin)                 \
    gpio_config_t pi4ioe5v9554_gpio_cfg = {              \
        .intr_type = GPIO_INTR_NEGEDGE,                  \
        .pin_bit_mask = (1 << pin),                      \
        .mode = GPIO_MODE_INPUT                          \
    }                                                    \
    ESP_ERROR_CHECK(gpio_config(&pi4ioe5v9554_gpio_cfg))

/**
 * @brief The number of the I/O pin
 * 
 */
typedef enum rf_pi4ioe5v9554_pin_num_e
{
    RF_PI4IOE5V9554_PIN_NUM_0 = 0x01,
    RF_PI4IOE5V9554_PIN_NUM_1 = 0x02,
    RF_PI4IOE5V9554_PIN_NUM_2 = 0x04,
    RF_PI4IOE5V9554_PIN_NUM_3 = 0x08,
    RF_PI4IOE5V9554_PIN_NUM_4 = 0x10,
    RF_PI4IOE5V9554_PIN_NUM_5 = 0x20,
    RF_PI4IOE5V9554_PIN_NUM_6 = 0x40,
    RF_PI4IOE5V9554_PIN_NUM_7 = 0x80,
    RF_PI4IOE5V9554_PIN_NUM_ALL = 0xFF
} rf_pi4ioe5v9554_pin_num_t;

/**
 * @brief The mode of the I/O pin
 * 
 */
typedef enum rf_pi4ioe5v9554_pin_mode_e
{
    RF_PI4IOE5V9554_PIN_MODE_OUTPUT = 0x00,
    RF_PI4IOE5V9554_PIN_MODE_INPUT  = 0x01
} rf_pi4ioe5v9554_pin_mode_t;

/**
 * @brief The polarity of the input pin
 * 
 */
typedef enum rf_pi4ioe5v9554_pin_polarity_e
{
    RF_PI4IOE5V9554_PIN_POLARITY_RETAINED = 0x00,
    RF_PI4IOE5V9554_PIN_POLARITY_INVERTED = 0x01
} rf_pi4ioe5v9554_pin_polarity_t;

/**
 * @brief The level of the I/O pin
 * 
 */
typedef enum rf_pi4ioe5v9554_pin_level_e
{
    RF_PI4IOE5V9554_PIN_LEVEL_LOW  = 0x00,
    RF_PI4IOE5V9554_PIN_LEVEL_HIGH = 0x01
} rf_pi4ioe5v9554_pin_level_t;

/**
 * @brief Configuration structure
 * 
 */
typedef struct rf_pi4ioe5v9554_cfg_s
{
    i2c_port_t i2c_num; /*!< I2C port number */
    uint8_t i2c_addr;   /*!< I2C device's 7-bit address */
    gpio_num_t isr_pin; /*!< Interrupt GPIO number */
    void (*isr_handler)(void* arg); /*!< Interrupt handler */
} rf_pi4ioe5v9554_cfg_t;

/**
 * @brief Initialize Port Expander driver
 * 
 * @param instance_id Instance id. The maximum number of instances is defined by RF_PI4IOE5V9554_MAX_INSTANCES.
 * @param p_cfg Configuration structure pointer. Use RF_PI4IOE5V9554_CONFIG_STATIC_INSTANCE or RF_PI4IOE5V9554_CONFIG_INSTANCE to create configuration.
 * @return
 *     - ESP_ERR_NO_MEM Memory allocation failure.
 *     - ESP_ERR_INVALID_ARG Parameter error.
 *     - ESP_FAIL The instance is already in use.
 *     - ESP_OK Success.
 */
esp_err_t rf_pi4ioe5v9554_init(uint8_t instance_id, rf_pi4ioe5v9554_cfg_t* p_cfg);

/**
 * @brief Configuration of the pin mode
 * 
 * @param instance_id Instance id. The maximum number of instances is defined by RF_PI4IOE5V9554_MAX_INSTANCES.
 * @param pin_mask The pin mask for configuration (e.g. RF_PI4IOE5V9554_PIN_NUM_0 | RF_PI4IOE5V9554_PIN_NUM_1).
 * @param pin_mode The mode of the pin (rf_pi4ioe5v9554_pin_mode_e).
 * @param ticks_to_wait Maximum ticks to wait before issuing a timeout.
 * @return
 *     - ESP_ERR_NOT_FOUND Instance not initialized. Use the rf_pi4ioe5v9554_init funcion before.
 *     - ESP_ERR_TIMEOUT Operation timeout because the instance is using by another task.
 *     - ESP_OK Success.
 *     - Other - I2C driver error.
 */
esp_err_t rf_pi4ioe5v9554_pin_cfg_mode(uint8_t instance_id, uint8_t pin_mask, rf_pi4ioe5v9554_pin_mode_t pin_mode, TickType_t ticks_to_wait);

/**
 * @brief Configuration of the input pin polarity
 * 
 * @param instance_id Instance id. The maximum number of instances is defined by RF_PI4IOE5V9554_MAX_INSTANCES.
 * @param pin_mask The pin mask for configuration (e.g. RF_PI4IOE5V9554_PIN_NUM_0 | RF_PI4IOE5V9554_PIN_NUM_1).
 * @param pin_polarity The polarity of the pin (rf_pi4ioe5v9554_pin_polarity_e).
 * @param ticks_to_wait Maximum ticks to wait before issuing a timeout.
 * @return
 *     - ESP_ERR_NOT_FOUND Instance not initialized. Use the rf_pi4ioe5v9554_init funcion before.
 *     - ESP_ERR_TIMEOUT Operation timeout because the instance is using by another task.
 *     - ESP_OK Success.
 *     - Other - I2C driver error.
 */
esp_err_t rf_pi4ioe5v9554_pin_cfg_polarity(uint8_t instance_id, uint8_t pin_mask, rf_pi4ioe5v9554_pin_polarity_t pin_polarity, TickType_t ticks_to_wait);

/**
 * @brief Set level of the output pin
 * 
 * @param instance_id Instance id. The maximum number of instances is defined by RF_PI4IOE5V9554_MAX_INSTANCES.
 * @param pin_mask The pin mask for configuration (e.g. RF_PI4IOE5V9554_PIN_NUM_0 | RF_PI4IOE5V9554_PIN_NUM_1).
 * @param pin_level The level of the output pin (rf_pi4ioe5v9554_pin_level_e).
 * @param ticks_to_wait Maximum ticks to wait before issuing a timeout.
 * @return
 *     - ESP_ERR_NOT_FOUND Instance not initialized. Use the rf_pi4ioe5v9554_init funcion before.
 *     - ESP_ERR_TIMEOUT Operation timeout because the instance is using by another task.
 *     - ESP_OK Success.
 *     - Other - I2C driver error.
 */
esp_err_t rf_pi4ioe5v9554_pin_set_level(uint8_t instance_id, uint8_t pin_mask, rf_pi4ioe5v9554_pin_level_t pin_level, TickType_t ticks_to_wait);

/**
 * @brief Get level of the all input pin
 * 
 * @param instance_id Instance id. The maximum number of instances is defined by RF_PI4IOE5V9554_MAX_INSTANCES.
 * @param levels The value of the Input Port Register (datasheet: b. Register Description)
 * @param ticks_to_wait Maximum ticks to wait before issuing a timeout.
 * @return
 *     - ESP_ERR_NOT_FOUND Instance not initialized. Use the rf_pi4ioe5v9554_init funcion before.
 *     - ESP_ERR_TIMEOUT Operation timeout because the instance is using by another task.
 *     - ESP_OK Success.
 *     - Other - I2C driver error.
 */
esp_err_t rf_pi4ioe5v9554_pin_get_level(uint8_t instance_id, uint8_t* levels, TickType_t ticks_to_wait);

#endif //DRV_PORT_EXPANDER_PI4IOE5V9554_H
#ifdef __cplusplus
}
#endif