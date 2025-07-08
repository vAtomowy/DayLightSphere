#pragma once

#include "driver.h"
#include "current_sense.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class PIDController
{
public:
    PIDController(
        CurrentSense& currentSense,
        Driver& driver,
        const char* taskName,
        UBaseType_t taskPriority,
        uint16_t taskStackSize,
        float kp,
        float ki,
        float kd
    );

    void Init();
    void SetTarget(float target);
    void Reset();

private:
    CurrentSense& mCurrentSense;
    Driver& mDriver;

    const char*     mTaskName;
    UBaseType_t     mTaskPriority;
    uint16_t        mTaskStackSize;
    TaskHandle_t    mPidTaskHandle;

    float mKp, mKi, mKd;
    float mTarget;
    float mIntegral;
    float mPrevError;

    static constexpr TickType_t kPidTaskPeriod = pdMS_TO_TICKS(100); 
    static void PidTask(void* pvParameter);
};
