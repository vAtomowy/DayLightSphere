#pragma once

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "HWConfig.h"
#include <atomic>

#include "driver/gpio.h"
#include "driver/ledc.h"

class Driver
{
public:

    Driver(gpio_num_t gpioDriver, const char* taskName, UBaseType_t taskPriority, uint16_t taskStackSize);

    void Init(void);

    bool DriverIsEnabled(void);
    void SetDriverEnable(bool enable);
    void SetPwm(uint32_t duty); 

private:
    gpio_num_t      mGpioDriver;
    const char*     mTaskName;
    UBaseType_t     mTaskPriority;
    uint16_t        mTaskStackSize;

    TaskHandle_t    mDriverTaskHandle;

    bool            mEnable; // 1 == On, 0 == Off
    uint32_t        mDuty;
    
    static void DriverTask(void* pvParameter);
    void InitPwm();
};
