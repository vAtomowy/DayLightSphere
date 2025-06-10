#pragma once

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "HWConfig.h"
#include <atomic>

class Button
{
    public:
        enum class State : uint8_t
        {
            kPressed = 0,
            kReleased
        };

        void Init(void);
        uint8_t ResetPinGetLevel();
        State GetState(void);
        
    private:
        friend Button & ButtonMgr(void);
        static Button sButton;

        rf_pi4ioe5v9554_pin_num_t    mGpioButton;
        State                        mState;

        void SetState(State);
        static void ButtonTask(void* pvParameter);
};

inline Button & ButtonMgr(void)
{
    return Button::sButton;
}