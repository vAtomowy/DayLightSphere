#include "current_sense.h"
#include "esp_log.h"

static const char* TAG = "CurrentSense";

i2c_port_t CurrentSense::sI2cPortsInitialized[CurrentSense::kMaxInstances] = {};
size_t CurrentSense::sI2cPortsCount = 0;

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

    bool i2cAlreadyInitialized = false;

    mCurrentSenseCfg.i2c_num = mI2cPort;
    mCurrentSenseCfg.i2c_addr = mI2cAddr;

    ESP_LOGI(TAG, "Init started for I2C port %d", mCurrentSenseCfg.i2c_num);

    for (size_t i = 0; i < CurrentSense::sI2cPortsCount; i++) {
        ESP_LOGI(TAG, "Checking initialized port at index %d: %d", i, CurrentSense::sI2cPortsInitialized[i]);
        if (CurrentSense::sI2cPortsInitialized[i] == mCurrentSenseCfg.i2c_num) {
            ESP_LOGI(TAG, "I2C port %d already initialized, skipping driver install.", mCurrentSenseCfg.i2c_num);
            i2cAlreadyInitialized = true;
            break;
        }
    }

    if (!i2cAlreadyInitialized) {
        ESP_LOGI(TAG, "Configuring I2C driver for port %d", mCurrentSenseCfg.i2c_num);

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

        ESP_LOGI(TAG, "Setting I2C parameters");
        ESP_ERROR_CHECK(i2c_param_config(mCurrentSenseCfg.i2c_num, &ina219_i2c_cfg));

        ESP_LOGI(TAG, "Installing I2C driver");
        ESP_ERROR_CHECK(i2c_driver_install(mCurrentSenseCfg.i2c_num, ina219_i2c_cfg.mode, 0, 0, 0));

        if (CurrentSense::sI2cPortsCount < CurrentSense::kMaxInstances) {
            CurrentSense::sI2cPortsInitialized[CurrentSense::sI2cPortsCount++] = mCurrentSenseCfg.i2c_num;
            ESP_LOGI(TAG, "Port %d added to initialized list at index %d", mCurrentSenseCfg.i2c_num, CurrentSense::sI2cPortsCount - 1);
        } else {
            ESP_LOGW(TAG, "Too many INA219 instances, can't track I2C port %d", mCurrentSenseCfg.i2c_num);
        }
    }

    err = ina219_init(mIdInstance, &mCurrentSenseCfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "INA219 init (instance %d) failed", mIdInstance);
    } else {
        ESP_LOGI(TAG, "INA219 init success (instance %d)", mIdInstance);
    }

    ina219_configure(mIdInstance, kDefaultConfig, pdMS_TO_TICKS(10)); 

    ina219_calibration(mIdInstance, kDefaultCalibration, pdMS_TO_TICKS(10)); 

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