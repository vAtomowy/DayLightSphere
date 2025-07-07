#include "pid_controller.h"
#include "esp_log.h"

#define TAG "PidController"

PidController::PidController()
    : mDriver(DRV1_GPIO, DRV1_GPIO_CHANNEL, DRV1_TASK_NAME, DRV1_TASK_PRIORITY, DRV1_TASK_STACK_SIZE),
      mCurrentSense(
          CURR_SENSE_COLD_ID_INSTANCE,
          CURR_SENSE_I2C_PORT,
          CURR_SENSE_COLD_I2C_ADDR,
          CURR_SENSE_SDA_PIN,
          CURR_SENSE_SCL_PIN,
          CURR_SENSE_COLD_TASK_NAME,
          CURR_SENSE_COLD_TASK_PRIORITY,
          CURR_SENSE_COLD_TASK_STACK_SIZE)
{
    mTargetCurrent = 0.0f;
    mLastUpdate = xTaskGetTickCount();
}

void PidController::Init()
{
    mDriver.Init();
    mDriver.SetDriverEnable(true);

    mCurrentSense.Init();
}

void PidController::SetTargetCurrent(float target)
{
    mTargetCurrent = target;
}

void PidController::UpdateControlLoop()
{
    float current = mCurrentSense.GetCurrent();
    float error = mTargetCurrent - current;

    TickType_t now = xTaskGetTickCount();
    float dt = (now - mLastUpdate) / 1000.0f;
    mLastUpdate = now;

    mIntegral += error * dt;
    float derivative = (error - mPrevError) / dt;
    mPrevError = error;

    float output = mKp * error + mKi * mIntegral + mKd * derivative;

    uint32_t pwmDuty = static_cast<uint32_t>(output * 1023.0f);
    if (pwmDuty > 1023) pwmDuty = 1023;
    if (pwmDuty < 0) pwmDuty = 0;

    mDriver.SetPwm(pwmDuty);

    ESP_LOGI(TAG, "Target: %.3f A, Measured: %.3f A, PWM: %u", mTargetCurrent, current, pwmDuty);
}

float PidController::GetVoltage() const
{
    return mCurrentSense.GetBusVoltage();
}

float PidController::GetCurrent() const
{
    return mCurrentSense.GetCurrent();
}
