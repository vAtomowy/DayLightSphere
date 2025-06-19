#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "ina219.h"

class CurrentSense
{
public:
    void Init();
    float GetBusVoltage(uint8_t sensor_id);
    float GetShuntVoltage(uint8_t sensor_id);
    float GetCurrent(uint8_t sensor_id); 

private:
    CurrentSense() = default;
    CurrentSense(const CurrentSense&) = delete;
    CurrentSense& operator=(const CurrentSense&) = delete;

    friend CurrentSense& CurrentSenseMgr();

    static CurrentSense sCurrentSense;

    uint8_t mINA219Handle[2];
    ina219_cfg_t mINA219Cfg[2];

    static void INA219Task(void* pvParameter); 
};

inline CurrentSense& CurrentSenseMgr()
{
    return CurrentSense::sCurrentSense;
}
