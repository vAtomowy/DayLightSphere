#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
extern "C" {
#include "ina219.h"
}
#include "HWConfig.h"

class CurrentSense
{
public:
    CurrentSense(
        uint8_t idInstance,
        i2c_port_t i2cPort,
        uint8_t i2cAddr,
        gpio_num_t sdaPin,
        gpio_num_t sclPin,
        const char* taskName,
        UBaseType_t taskPriority,
        uint16_t taskStackSize
    ); 

    void Init();
    float GetBusVoltage();
    float GetShuntVoltage();
    float GetCurrent(); 

private:
    // ---- Fixed INA219 settings (shared by all instances, not user-configurable). ---------
    static constexpr uint16_t kDefaultConfig =
        INA219_BVOLTAGERANGE_32V |
        INA219_GAIN_8_320MV |
        INA219_BADCRES_12BIT_128S |
        INA219_SADCRES_12BIT_128S |
        INA219_MODE_SANDBVOLT_CONTINUOUS;

    static constexpr uint16_t kDefaultCalibration = 0x6AA2;
    // --------------------------------------------END----------------------------------------
    static constexpr size_t kMaxInstances = INA219_MAX_INSTANCES;
    static i2c_port_t sI2cPortsInitialized[kMaxInstances];
    static size_t sI2cPortsCount;

    uint8_t         mIdInstance;
    i2c_port_t      mI2cPort;
    uint8_t         mI2cAddr;
    gpio_num_t      mSdaPin;
    gpio_num_t      mSclPin;

    const char*     mTaskName;
    UBaseType_t     mTaskPriority;
    uint16_t        mTaskStackSize; 

    TaskHandle_t mCurrentSenseTaskHandle;

    ina219_cfg_t mCurrentSenseCfg;

    static void CurrentSenseTask(void* pvParameter); 
};
