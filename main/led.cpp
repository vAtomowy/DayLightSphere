#include "led.h"
#include "esp_log.h"
#include "HWConfig.h"

static const char *TAG = "Led";

void Led::LedTask(void *pvParameter)
{
    Led *self = (Led *)pvParameter;
    int tickCounter = 0; 
    bool ledLevel = false;
    State lastState = State::sOff;

    ESP_LOGI(TAG, "LED task started");

    while (true)
    {
        State currentState = self->GetLedState();

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
            switch (self->GetLedBlinkFreq())
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
            }

            tickCounter = (tickCounter > 100) ? 0 : tickCounter + 1;
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

Led::State Led::GetLedState(void)
{
    return mState;
}

void Led::SetLedState(Led::State state)
{
    mState = state;
}

Led::BlinkFreq Led::GetLedBlinkFreq(void)
{ 
    return mBlinkFreq;  
}

void Led::SetLedBlinkFreq(Led::BlinkFreq freq)
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
    init_led(mGpioLed);
    
    SetLedState(State::sOff);
    SetLedBlinkFreq(BlinkFreq::Freq1s);

    BaseType_t result = xTaskCreate(LedTask, mTaskName, mTaskStackSize, this, mTaskPriority, &mLedTaskHandle);

    if (result != pdPASS)
    {
        printf("Failed to create LED task for GPIO %d\n", mGpioLed);
        ESP_LOGE(TAG, "Failed to create LED task for GPIO %d\n", mGpioLed);
    }
}

Led::Led(gpio_num_t gpioLed, const char* taskName, UBaseType_t taskPriority, uint16_t taskStackSize): 
      mGpioLed(gpioLed),
      mTaskName(taskName),
      mTaskPriority(taskPriority),
      mTaskStackSize(taskStackSize),
      mLedTaskHandle(nullptr),
      mState(State::sOff),
      mBlinkFreq(BlinkFreq::Freq1s)
{
}