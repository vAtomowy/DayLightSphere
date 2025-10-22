#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_err.h"

#include "HWConfig.h"
#include "led.h"
#include "button.h"
#include "driver/i2c.h"

#include "esp_err.h"

#define LED_RED GPIO_NUM_6

static const char* TAG = "main";

static int s_led_state = 0; 

#define I2C_MASTER_FREQ_HZ    100000
#define I2C_MASTER_TIMEOUT_MS 1000

void i2c_scan() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_5,
        .scl_io_num = GPIO_NUM_4,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
        .clk_flags = 0
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_B_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_B_PORT, I2C_MODE_MASTER, 0, 0, 0));

    printf("I2C scan...\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_B_PORT, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            printf("Found device at 0x%02X\n", addr);
        }
    }
    printf("Scan done.\n");
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting LED blink example...");
    
    Led redLed2(YELLOW_LED, YELLOW_LED_TASK_NAME, YELLOW_LED_TASK_PRIORITY, YELLOW_LED_TASK_STACK_SIZE);
    redLed2.Init();

    Led redLed1(RED_LED, RED_LED_TASK_NAME, RED_LED_TASK_PRIORITY, RED_LED_TASK_STACK_SIZE);
    redLed1.Init();

    Button Key1Button(KEY1_GPIO, KEY1_TASK_NAME, KEY1_TASK_PRIORITY, KEY1_TASK_STACK_SIZE);
    Key1Button.Init();

    redLed2.SetLedState(Led::State::sOff);  
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    redLed2.SetLedState(Led::State::sOn); 
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    redLed2.SetLedState(Led::State::sOff); 
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    redLed1.SetLedState(Led::State::sOff);  
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    redLed1.SetLedState(Led::State::sOn); 
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    redLed1.SetLedState(Led::State::sOff); 
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    i2c_scan(); 

    redLed1.SetLedState(Led::State::sBlink);
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    redLed2.SetLedState(Led::State::sBlink); 
    while (1) {
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
