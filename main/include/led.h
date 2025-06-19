#pragma once

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "HWConfig.h"
#include <atomic>

#include "driver/gpio.h"

class Led
{
    public:
        enum class State : uint8_t
        {
            sOff = 0,
            sOn, 
            sBlink
        };
        enum class BlinkFreq : uint8_t 
        { 
            Freq250ms,
            Freq500ms,
            Freq1s,
            Freq2s,
            Freq10s
        };

        void Init(void);

        Led(gpio_num_t gpioLed, const char* taskName, UBaseType_t taskPriority, uint16_t taskStackSize); 

        State GetLedState(void);
        BlinkFreq GetLedBlinkFreq(void);
        void SetLedState(State);
        void SetLedBlinkFreq(BlinkFreq);
        
    private:
        gpio_num_t      mGpioLed;
        const char*     mTaskName;
        UBaseType_t     mTaskPriority;
        uint16_t        mTaskStackSize;

        TaskHandle_t    mLedTaskHandle;

        State           mState;
        BlinkFreq       mBlinkFreq;
        
        static void init_led(gpio_num_t led); 
        static void LedTask(void* pvParameter);
};