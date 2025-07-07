#pragma once

#include "driver.h"
#include "current_sense.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class PidController
{
public:
    PidController();
    void Init();
    void UpdateControlLoop();  
    void SetTargetCurrent(float target);  
    float GetVoltage() const;
    float GetCurrent() const;

private:
    Driver mDriver;
    CurrentSense mCurrentSense;
    
    float mTargetCurrent;
    float mKp = 0.1f;  
    float mKi = 0.0f;
    float mKd = 0.0f;
    float mIntegral = 0.0f;
    float mPrevError = 0.0f;

    TickType_t mLastUpdate;
};
