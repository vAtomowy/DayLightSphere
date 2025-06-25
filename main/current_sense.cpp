#include "current_sense.h"
#include "esp_log.h"

static const char* TAG = "CurrentSense";

void CurrentSense::CurrentSenseTask(void* pvParameter)
{
    CurrentSense* self = (CurrentSense*)pvParameter;

    ESP_LOGI(TAG, "Current Sense Task was started");
    while (1)
    {
        //ESP_LOGI(TAG, "Task live or better Alive !");
        vTaskDelay(pdMS_TO_TICKS(25));
    } 
}


float CurrentSense::GetBusVoltage()
{
    uint16_t raw_value = 0;
    float voltage = 0.0f;

    if (ina219_read_bus_voltage(mIdInstance, &raw_value, pdMS_TO_TICKS(10)) == ESP_OK)
    {
        voltage = raw_value * 0.004f; // 4mV 
    }

    return voltage;
}

float CurrentSense::GetShuntVoltage()
{
    uint16_t raw_value = 0;
    float voltage = 0.0f;

    if (ina219_read_shunt_voltage(mIdInstance, &raw_value, pdMS_TO_TICKS(10)) == ESP_OK)
    {
        int16_t signed_value = static_cast<int16_t>(raw_value);
        voltage = signed_value * 0.00001f; // 10uV
    }

    return voltage;
}

float CurrentSense::GetCurrent()
{
    float shunt_voltage = GetShuntVoltage();
    return shunt_voltage / SHUNT_RESISTOR_OHMS;
}

void CurrentSense::Init()
{
    esp_err_t err = ESP_OK;

    mCurrentSenseCfg.i2c_num = mI2cPort;
    mCurrentSenseCfg.i2c_addr = mI2cAddr;

    i2c_config_t ina219_i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = mSdaPin,                  
        .scl_io_num = mSclPin,             
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master = {
            .clk_speed = INA219_DEFAULT_SPEED_HZ
        },
        .clk_flags = 0
    };

    i2c_driver_delete(mCurrentSenseCfg.i2c_num);

    ESP_ERROR_CHECK(i2c_param_config(mCurrentSenseCfg.i2c_num, &ina219_i2c_cfg));

    ESP_ERROR_CHECK(i2c_driver_install(mCurrentSenseCfg.i2c_num, ina219_i2c_cfg.mode, 0, 0, 0));

    err = ina219_init(mIdInstance, &mCurrentSenseCfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "INA219 init (instance %d) failed", mIdInstance);
    } else {
        ESP_LOGI(TAG, "INA219 init success (instance %d)", mIdInstance);
    }

    uint16_t config =
        INA219_BVOLTAGERANGE_32V      |  // 0x2000
        INA219_GAIN_8_320MV           |  // 0x1800
        INA219_BADCRES_12BIT_128S     |  // 0x0780
        INA219_SADCRES_12BIT_128S     |  // 0x0078
        INA219_MODE_SANDBVOLT_CONTINUOUS; // 0x0007

    ina219_configure(mIdInstance, config, pdMS_TO_TICKS(10)); 

    ina219_calibration(mIdInstance, 0x6AA2, pdMS_TO_TICKS(10)); 

    if (err == ESP_OK)
    {
       xTaskCreate(CurrentSenseTask, mTaskName, mTaskStackSize, this, mTaskPriority, &mCurrentSenseTaskHandle);
    }
    else 
    { 
        ESP_LOGE(TAG, "INA's 219 cannot create a TASK !");
    }

}

CurrentSense::CurrentSense( uint8_t idInstance,
                         i2c_port_t i2cPort,
                         uint8_t i2cAddr,
                         gpio_num_t sdaPin,
                         gpio_num_t sclPin,
                         const char* taskName,
                         UBaseType_t taskPriority,
                         uint16_t taskStackSize) :
    mIdInstance(idInstance),
    mI2cPort(i2cPort),
    mI2cAddr(i2cAddr),
    mSdaPin(sdaPin),
    mSclPin(sclPin),
    mTaskName(taskName),
    mTaskPriority(taskPriority),
    mTaskStackSize(taskStackSize),
    mCurrentSenseTaskHandle(nullptr)
{
    mCurrentSenseCfg.i2c_num = mI2cPort;
    mCurrentSenseCfg.i2c_addr = mI2cAddr;
}