#include "button.h"
#include "pi4ioe5v9554.h"
#include "esp_log.h"

static const char* TAG = "Button";

Button Button::sButton;

namespace {
    TaskHandle_t sButtonTaskHandle;
}

void Button::ButtonTask(void* pvParameter)
{
    Button* self  = (Button*)pvParameter;
    uint8_t lastLevel = 0;

    ESP_LOGI(TAG, "Button Task started");
    while (true)
    {
        if(self->ResetPinGetLevel() == 0)
        { 
            vTaskDelay(pdMS_TO_TICKS(50));
            if(self->ResetPinGetLevel() == 0)
            { 
                self->SetState(State::kPressed); 
                ESP_LOGD(TAG, "Button Pressed !");
            }
            else 
            { 
                self->SetState(State::kReleased); 
                ESP_LOGD(TAG, "Button Released !");
            }
        }
        else
        { 
            self->SetState(State::kReleased); 
            ESP_LOGD(TAG, "Button Released !");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

uint8_t Button::ResetPinGetLevel()
{
    uint8_t levels;
    rf_pi4ioe5v9554_pin_get_level(PORT_EXPANDER_INSTANCE_ID, &levels, pdMS_TO_TICKS(100));
                
    uint8_t mask = 1 << 6;
    return (levels & mask) != 0;
}

Button::State Button::GetState(void)
{
    return mState;
}

void Button::SetState(Button::State state)
{
    mState = state;
}

void Button::Init(void)
{
    esp_err_t err = ESP_OK; 

    mGpioButton = BUTTON_GPIO;

    ESP_ERROR_CHECK(rf_pi4ioe5v9554_pin_cfg_mode(PORT_EXPANDER_INSTANCE_ID, mGpioButton, RF_PI4IOE5V9554_PIN_MODE_INPUT, pdMS_TO_TICKS(100)));

    if (ESP_OK == err)
    {
        // Create Task
        xTaskCreate(ButtonTask, BUTTON_TASK_NAME, BUTTON_TASK_STACK_SIZE, this, BUTTON_TASK_PRIORITY, &sButtonTaskHandle);
    }
    else 
    { 
        ESP_LOGE(TAG, "Create button task failed !");
    }

}