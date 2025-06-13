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
#include "driver.h"

#define LED_RED GPIO_NUM_6

static const char* TAG = "main";

static int s_led_state = 0; 

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting LED blink example...");

    static int test_state = 0;
    [[maybe_unused]] static int test_state_2 = 0;

    uint32_t duty = 0;
    uint32_t duty2 = 200;
    bool dir = false;
    bool dir2 = false;
    
    Led yelLed(YELLOW_LED, YELLOW_LED_TASK_NAME, YELLOW_LED_TASK_PRIORITY, YELLOW_LED_TASK_STACK_SIZE);
    yelLed.Init();

    Led redLed(RED_LED, RED_LED_TASK_NAME, RED_LED_TASK_PRIORITY, RED_LED_TASK_STACK_SIZE);
    redLed.Init();

    Button Key1Button(KEY1_GPIO, KEY1_TASK_NAME, KEY1_TASK_PRIORITY, KEY1_TASK_STACK_SIZE);
    Key1Button.Init();

    Button Key2Button(KEY2_GPIO, KEY2_TASK_NAME, KEY2_TASK_PRIORITY, KEY2_TASK_STACK_SIZE);
    Key2Button.Init();

    Driver Pwm1Driver(DRV1_GPIO, DRV1_GPIO_CHANNEL, DRV1_TASK_NAME, DRV1_TASK_PRIORITY, DRV1_TASK_STACK_SIZE);
    Pwm1Driver.Init();

    Driver Pwm2Driver(DRV2_GPIO, DRV2_GPIO_CHANNEL, DRV2_TASK_NAME, DRV2_TASK_PRIORITY, DRV2_TASK_STACK_SIZE);
    Pwm2Driver.Init();

    yelLed.SetLedState(Led::State::sOff);  
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    yelLed.SetLedState(Led::State::sOn); 
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    yelLed.SetLedState(Led::State::sOff); 
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    Pwm1Driver.SetDriverEnable(true);
    Pwm2Driver.SetDriverEnable(true);

    while (1) {

        // TEST 1 
        // ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON1" : "OFF");
        // s_led_state = !s_led_state;
        // yelLed.SetLedBlinkFreq((Led::BlinkFreq)test_state); 
        // redLed.SetLedBlinkFreq((Led::BlinkFreq)(test_state+1)); 
        // vTaskDelay(20000 / portTICK_PERIOD_MS);
        // test_state++; 
        // if(test_state > 4) test_state = 0; 

        // TEST 2
        static Button::State prevStateKey1 = Button::State::kIdle;
        static Button::State prevStateKey2 = Button::State::kIdle;
        Button::State currentStateKey1 = Key1Button.GetButtonState();
        Button::State currentStateKey2 = Key2Button.GetButtonState();

        if (currentStateKey1 == Button::State::kPressed && prevStateKey1 != Button::State::kPressed)
        {
            Led::State state = redLed.GetLedState();
            if (state == Led::State::sOff || state == Led::State::sBlink)
            {
                redLed.SetLedState(Led::State::sOn);
                ESP_LOGI(TAG, "KEY1_ON");
            }
            else
            {
                redLed.SetLedState(Led::State::sOff);
                ESP_LOGI(TAG, "KEY1_OFF");
            }
        }
        else if (currentStateKey1 == Button::State::kHeld && prevStateKey1 != Button::State::kHeld)
        {
            redLed.SetLedState(Led::State::sBlink);
            redLed.SetLedBlinkFreq((Led::BlinkFreq)0);
            ESP_LOGI(TAG, "KEY1_BLINK");
        }

        if (currentStateKey2 == Button::State::kPressed && prevStateKey2 != Button::State::kPressed)
        {
            Led::State state = yelLed.GetLedState();
            if (state == Led::State::sOff || state == Led::State::sBlink)
            {
                yelLed.SetLedState(Led::State::sOn);
                ESP_LOGI(TAG, "KEY2_ON");
            }
            else
            {
                yelLed.SetLedState(Led::State::sOff);
                ESP_LOGI(TAG, "KEY2_OFF");
            }
        }
        else if (currentStateKey2 == Button::State::kHeld && prevStateKey2 != Button::State::kHeld)
        {
            yelLed.SetLedState(Led::State::sBlink);
            yelLed.SetLedBlinkFreq((Led::BlinkFreq)0);
            ESP_LOGI(TAG, "KEY2_BLINK");
        }

        prevStateKey1 = currentStateKey1;
        prevStateKey2 = currentStateKey2;
        
        Pwm1Driver.SetPwm(duty);
        Pwm2Driver.SetPwm(duty2); // zimny led
        if (dir == false)
        { 
            duty= duty + 10;
        }
        else 
        { 
            duty = duty - 10; 
        }
        if(duty > 200 || duty < 0) 
        { 
            duty = 0;
            dir = !dir;
        }
        
        if (dir2 == false)
        { 
            duty2= duty2 + 10;
        }
        else 
        { 
            duty2 = duty2 - 10; 
        }
        if(duty2 > 200 || duty2 < 0) 
        { 
            duty2 = 0;
            dir2 = !dir2;
        }

        vTaskDelay(64 / portTICK_PERIOD_MS);

    }
}
