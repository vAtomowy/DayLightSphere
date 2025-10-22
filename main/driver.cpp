#include "driver.h"
#include "esp_log.h"
#include "HWConfig.h"

static const char* TAG = "Driver";

void Driver::DriverTask(void* pvParameter)
{
    Driver *self = (Driver *)pvParameter;

    ESP_LOGI(TAG, "Driver task started on GPIO");

    while (true) 
    {
        if (self->DriverIsEnabled()) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, self->mPwmChannel, self->mDuty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, self->mPwmChannel);
        } else {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, self->mPwmChannel, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, self->mPwmChannel);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void Driver::SetDriverEnable(bool enable)
{
    mEnable = enable;
    ESP_LOGI(TAG, "Driver ENABLED: %s", enable ? "YES" : "NO");
}

bool Driver::DriverIsEnabled(void)
{
    return mEnable;
}

void Driver::SetPwm(uint32_t duty)
{
    if (duty > 1023) duty = 1023;
    mDuty = duty;
    ESP_LOGI(TAG, "PWM duty set to %lu", duty);
}

void Driver::InitPwm()
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_10_BIT, // 0-1023
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 50000,              // 50kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = mGpioDriver,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = mPwmChannel,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

void Driver::Init(void)
{
    InitPwm();

    BaseType_t result = xTaskCreate(DriverTask, mTaskName, mTaskStackSize, this, mTaskPriority, &mDriverTaskHandle);

    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create Driver task for GPIO %d", mGpioDriver);
    }
}

Driver::Driver(gpio_num_t gpioDriver, ledc_channel_t pwmChannel, const char* taskName, UBaseType_t taskPriority, uint16_t taskStackSize)
    : mGpioDriver(gpioDriver),
      mPwmChannel(pwmChannel),
      mTaskName(taskName),
      mTaskPriority(taskPriority),
      mTaskStackSize(taskStackSize),
      mDriverTaskHandle(nullptr),
      mEnable(false)
{
}
