#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_err.h"

#include "led.h"
#include "button.h"

#define LED_RED GPIO_NUM_6

static const char* TAG = "main";

static int s_led_state = 0; 


static void blink_led(void)
{
    gpio_set_level(LED_RED, s_led_state); 
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(LED_RED);
    gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
}

extern "C" void app_main(void)
{
    configure_led();
    ESP_LOGI(TAG, "Starting LED blink example...");

    static int test_state = 0;
    [[maybe_unused]] static int test_state_2 = 0;
    
    LedMgr().Init();
    //ButtonMgr().Init();

    LedMgr().SetState(Led::State::sOff); 
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    LedMgr().SetState(Led::State::sOn); 
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    LedMgr().SetState(Led::State::sOff); 
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    LedMgr().SetState(Led::State::sBlink); 
    // LedMgr().SetBlinkFreq((Led::BlinkFreq)0); 
    // vTaskDelay(3000 / portTICK_PERIOD_MS);

    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON1" : "OFF");
        blink_led();
        s_led_state = !s_led_state;
        LedMgr().SetBlinkFreq((Led::BlinkFreq)test_state); 
        vTaskDelay(20000 / portTICK_PERIOD_MS);
        test_state++; 
        if(test_state > 4) test_state = 0; 
    }
}
