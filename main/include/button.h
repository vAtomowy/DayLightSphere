#pragma once

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "HWConfig.h"
#include <atomic>

#include "driver/gpio.h"

class Button
{
public:
    enum class State : uint8_t
    {
        kIdle = 0,
        kDebounce,
        kPressed, 
        kHeld, 
        kReleased
    };

    Button(gpio_num_t gpioButton, const char* taskName, UBaseType_t taskPriority, uint16_t taskStackSize);

    void Init(void);

    State GetButtonState(void);
    uint8_t ReadPinLevel();

private:
    gpio_num_t      mGpioButton;
    const char*     mTaskName;
    UBaseType_t     mTaskPriority;
    uint16_t        mTaskStackSize;

    TaskHandle_t    mButtonTaskHandle;

    State           mState;

    void SetButtonState(State);
    static void ButtonTask(void* pvParameter);
};
