#include "current_sense.h"
#include "esp_log.h"
#include "HWConfig.h"

static const char* TAG = "CurrentSense";

CurrentSense CurrentSense::sCurrentSense;

void CurrentSense::Init()
{
    esp_err_t err;

    mINA219Cfg[0] = {
        .i2c_num  = I2C_NUM_0,
        .i2c_addr = 0x40
    };
    mINA219Cfg[1] = {
        .i2c_num  = I2C_NUM_0,
        .i2c_addr = 0x41
    };

    mINA219Handle[0] = INA219_COLD_INSTANCE; 
    mINA219Handle[1] = INA219_WARM_INSTANCE; 

    for (uint8_t i = 0; i < 2; i++)
    {
        err = ina219_init(mINA219Handle[i], &mINA219Cfg[i]);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "INA219[%d] init failed: %s", i, esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(TAG, "INA219[%d] initialized OK", i);
        }
    }
}

float CurrentSense::GetBusVoltage(uint8_t sensor_id)
{
    uint16_t reg_value = 0;
    float voltage = 0.0f;

    if (sensor_id < 2 &&
        ina219_read_register(mINA219Handle[sensor_id], INA219_REG_BUS_V, &reg_value, pdMS_TO_TICKS(10)) == ESP_OK)
    {
        reg_value >>= 3; // 13-bit resolution
        voltage = reg_value * 0.004f; // 4mV LSB
    }
    return voltage;
}

float CurrentSense::GetShuntVoltage(uint8_t sensor_id)
{
    int16_t reg_value = 0;
    float voltage = 0.0f;

    if (sensor_id < 2 &&
        ina219_read_register(mINA219Handle[sensor_id], INA219_REG_SHUNT_V, (uint16_t*)&reg_value, pdMS_TO_TICKS(10)) == ESP_OK)
    {
        voltage = reg_value * 0.00001f; // 10µV LSB
    }
    return voltage;
}

float CurrentSense::GetCurrent(uint8_t sensor_id)
{
    float shunt_voltage = GetShuntVoltage(sensor_id);
    return shunt_voltage / SHUNT_RESISTOR_OHMS;
}