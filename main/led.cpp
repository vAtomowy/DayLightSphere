#include "led.h"
#include "esp_log.h"
#include "HWConfig.h"

static const char *TAG = "Led";

Led Led::sLed;

namespace
{
    TaskHandle_t sLedTaskHandle;
}

void Led::LedTask(void *pvParameter)
{
    Led *self = (Led *)pvParameter;
    int tickCounter = 0; 
    bool ledLevel = false;
    State lastState = State::sOff;

    ESP_LOGI(TAG, "LED task started");

    while (true)
    {
        State currentState = self->GetState();

        if (currentState != lastState)
        {
            if (currentState == State::sOff)
            {
                gpio_set_level(self->mGpioLed, 0);
                ledLevel = false;
                ESP_LOGI(TAG, "LED OFF");
            }
            else if (currentState == State::sOn)
            {
                gpio_set_level(self->mGpioLed, 1);
                ledLevel = true;
                ESP_LOGI(TAG, "LED ON");
            }
            else if (currentState == State::sBlink)
            { 
                tickCounter = 0;
            }
            lastState = currentState;
        }

        if (currentState == State::sBlink)
        {
            int blinkPeriodTicks = 0;
            switch (self->GetBlinkFreq())
            {
                case BlinkFreq::Freq250ms: blinkPeriodTicks = 1; break;
                case BlinkFreq::Freq500ms: blinkPeriodTicks = 2; break;
                case BlinkFreq::Freq1s:    blinkPeriodTicks = 4; break;
                case BlinkFreq::Freq2s:    blinkPeriodTicks = 8; break;
                case BlinkFreq::Freq10s:   blinkPeriodTicks = 40; break;
                default:                   blinkPeriodTicks = 4; break;
            }

            if (tickCounter % blinkPeriodTicks == 0)
            {
                ledLevel = !ledLevel;
                gpio_set_level(self->mGpioLed, ledLevel);
                ESP_LOGI(TAG, "BLINK TOGGLE [%d]: %d", tickCounter, ledLevel);
            }

            tickCounter = (tickCounter > 100) ? 0 : tickCounter + 1;
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

Led::State Led::GetState(void)
{
    return mState;
}

void Led::SetState(Led::State state)
{
    mState = state;
}

Led::BlinkFreq Led::GetBlinkFreq(void)
{ 
    return mBlinkFreq;  
}

void Led::SetBlinkFreq(Led::BlinkFreq freq)
{ 
    mBlinkFreq = freq;
}

void Led::init_led(gpio_num_t led)
{
    gpio_reset_pin(led);
    gpio_set_direction(led, GPIO_MODE_OUTPUT);
}

void Led::Init(void)
{
    esp_err_t err = ESP_OK;

    mGpioLed = LED_GPIO;

    init_led(mGpioLed);
    SetState(State::sBlink);
    SetBlinkFreq(BlinkFreq::Freq1s);

    if (ESP_OK == err)
    {
        // Create Task
        xTaskCreate(LedTask, BUTTON_TASK_NAME, BUTTON_TASK_STACK_SIZE, this, BUTTON_TASK_PRIORITY, &sLedTaskHandle);
    }
    else
    {
        ESP_LOGE(TAG, "Create button task failed !");
    }
}