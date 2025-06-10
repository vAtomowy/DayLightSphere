#pragma once

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "HWConfig.h"
#include <atomic>

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
        State GetState(void);
        BlinkFreq GetBlinkFreq(void);
        void SetState(State);
        void SetBlinkFreq(BlinkFreq);
        
    private:
        friend Led & LedMgr(void);
        static Led sLed;

        gpio_num_t    mGpioLed;
        State         mState;
        BlinkFreq     mBlinkFreq;

        static void init_led(gpio_num_t led); 
        static void LedTask(void* pvParameter);
};

inline Led & LedMgr(void)
{
    return Led::sLed;
}