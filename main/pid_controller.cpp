#include "pid_controller.h"
#include "esp_log.h"

static const char* TAG = "PID";

void PIDController::PidTask(void* pvParameter)
{
    PIDController* self = static_cast<PIDController*>(pvParameter);
    ESP_LOGI(TAG, "PID Task started");

    while (true)
    {
        float current = self->mCurrentSense.GetCurrent(); 
        float error = self->mTarget - current;

        self->mIntegral += error * 0.1f; 
        float derivative = (error - self->mPrevError) / 0.1f;

        float output = self->mKp * error + self->mKi * self->mIntegral + self->mKd * derivative;

        // Konwersja na PWM
        int pwm = static_cast<int>(output);
        if (pwm > 1023) pwm = 1023;
        if (pwm < 0) pwm = 0;

        self->mDriver.SetPwm(static_cast<uint32_t>(pwm));

        self->mPrevError = error;

        ESP_LOGI(TAG, "PID: Target=%.2f A, Current=%.2f A, PWM=%d", self->mTarget, current, pwm);

        vTaskDelay(kPidTaskPeriod);
    }
}

void PIDController::SetTarget(float target)
{
    mTarget = target;
}

void PIDController::Reset()
{
    mIntegral = 0.0f;
    mPrevError = 0.0f;
}

void PIDController::Init()
{
    BaseType_t result = xTaskCreate(PidTask, mTaskName, mTaskStackSize, this, mTaskPriority, &mPidTaskHandle);

    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create PID task");
    }
    else
    {
        ESP_LOGI(TAG, "PID task created");
    }
}

PIDController::PIDController(
    CurrentSense& currentSense,
    Driver& driver,
    const char* taskName,
    UBaseType_t taskPriority,
    uint16_t taskStackSize,
    float kp,
    float ki,
    float kd)
    : mCurrentSense(currentSense),
      mDriver(driver),
      mTaskName(taskName),
      mTaskPriority(taskPriority),
      mTaskStackSize(taskStackSize),
      mPidTaskHandle(nullptr),
      mKp(kp), mKi(ki), mKd(kd),
      mTarget(0.0f),
      mIntegral(0.0f),
      mPrevError(0.0f)
{
}
