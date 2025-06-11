#include "button.h"
#include "esp_log.h"
#include "HWConfig.h"

static const char* TAG = "Button";

void Button::ButtonTask(void* pvParameter)
{
    Button *self = (Button *)pvParameter;

    ESP_LOGI(TAG, "Button task started");

    const TickType_t loopDelay = pdMS_TO_TICKS(30);
    const TickType_t holdThreshold = pdMS_TO_TICKS(1890);

    bool wasHeld = false;

    static uint8_t cnt; 
    static uint8_t state;

    while (true)
    {
        uint8_t level = self->ReadPinLevel();

        switch(self->GetButtonState())  
        { 
            case State::kIdle: 
            { 
                if (level == 0)
                {
                    self->SetButtonState(State::kDebounce);
                }
                else
                { 
                    // nothing
                }  
            }
            break; 

            case State::kDebounce: 
            { 
                if (level == 0)
                {
                    cnt = 0; 
                    self->SetButtonState(State::kPressed);
                }
                else
                { 
                    self->SetButtonState(State::kIdle);
                }  
            }
            break; 

            case State::kPressed: 
            { 
                if (level == 0)
                {
                    cnt++; 
                    if (cnt > 63) // ~1890ms / 30ms
                    {
                        self->SetButtonState(State::kHeld);
                        wasHeld = true;
                    }
                }
                else
                { 
                    self->SetButtonState(State::kReleased);
                }  
            }
            break;

            case State::kHeld:
            {
                if (level != 0)
                {
                    self->SetButtonState(State::kReleased);
                }
                // else: nadal trzymany – nic nie rób
            }
            break;

            case State::kReleased:
            {
                if (wasHeld)
                {
                    ESP_LOGI(TAG, "Button was held");
                    wasHeld = false;
                }
                else
                {
                    ESP_LOGI(TAG, "Button was clicked");
                }

                cnt = 0;
                self->SetButtonState(State::kIdle);
            }
            break;
        }

        vTaskDelay(loopDelay);
    }
}

uint8_t Button::ReadPinLevel()
{
    return gpio_get_level(mGpioButton);
}

Button::State Button::GetButtonState(void)
{
    return mState;
}

void Button::SetButtonState(Button::State state)
{
    mState = state;
}

void Button::Init(void)
{
    gpio_reset_pin(mGpioButton);
    gpio_set_direction(mGpioButton, GPIO_MODE_INPUT);
    gpio_pulldown_dis(mGpioButton);
    gpio_pullup_en(mGpioButton); 

    BaseType_t result = xTaskCreate(ButtonTask, mTaskName, mTaskStackSize, this, mTaskPriority, &mButtonTaskHandle);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create Button task for GPIO %d", mGpioButton);
    }
}

Button::Button(gpio_num_t gpioButton, const char* taskName, UBaseType_t taskPriority, uint16_t taskStackSize)
    : mGpioButton(gpioButton),
      mTaskName(taskName),
      mTaskPriority(taskPriority),
      mTaskStackSize(taskStackSize),
      mButtonTaskHandle(nullptr),
      mState(State::kReleased)
{
}
