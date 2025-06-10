#include "pi4ioe5v9554.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *PI4IOE5V9554_TAG = "port expander pi4ioe5v9554";

#define PI4IOE5V9554_CFG_ERROR_STR         "pi4ioe5v9554 configuration pointer error"
#define PI4IOE5V9554_INSTANCE_ID_ERROR_STR "pi4ioe5v9554 instance id error"
#define PI4IOE5V9554_I2C_NUM_ERROR_STR     "pi4ioe5v9554 i2c number error"
#define PI4IOE5V9554_SEM_ERR_STR           "pi4ioe5v9554 semaphore error"

typedef enum pi4ioe5v9554_cmd_e
{
    PI4IOE5V9554_CMD_INPUT = 0,
    PI4IOE5V9554_CMD_OUTPUT,
    PI4IOE5V9554_CMD_POLARITY,
    PI4IOE5V9554_CMD_CONFIG
} pi4ioe5v9554_cmd_t;

//Object (config and data) of the pi4ioe5v9554
typedef struct rf_pi4ioe5v9554_obj_s
{
    rf_pi4ioe5v9554_cfg_t cfg;
    SemaphoreHandle_t mux;
    uint8_t instance_id;
} rf_pi4ioe5v9554_obj_t;

static rf_pi4ioe5v9554_obj_t* p_pi4ioe5v9554_obj[RF_PI4IOE5V9554_MAX_INSTANCES] = {0}; /*!< The table of the pi4ioe5v955 instance pointers */

static esp_err_t prv_pi4ioe5v9554_write_register(rf_pi4ioe5v9554_obj_t* p_obj, pi4ioe5v9554_cmd_t cmd, uint8_t value, TickType_t ticks_to_wait)
{
    uint8_t buff[2] = {0};

    buff[0] = cmd;
    buff[1] = value;

    return i2c_master_write_to_device(p_obj->cfg.i2c_num, 
                                      p_obj->cfg.i2c_addr, 
                                      buff, 2, 
                                      ticks_to_wait);
}

static esp_err_t prv_pi4ioe5v9554_read_register(rf_pi4ioe5v9554_obj_t* p_obj, pi4ioe5v9554_cmd_t cmd, uint8_t* value, TickType_t ticks_to_wait)
{
    return i2c_master_write_read_device(p_obj->cfg.i2c_num, 
                                        p_obj->cfg.i2c_addr, 
                                        (const uint8_t*)&cmd, 1, 
                                        value, 1, 
                                        ticks_to_wait);
}

esp_err_t rf_pi4ioe5v9554_init(uint8_t instance_id, rf_pi4ioe5v9554_cfg_t* p_cfg)
{
    esp_err_t err = ESP_OK;
    rf_pi4ioe5v9554_obj_t* p_obj = NULL;

    ESP_RETURN_ON_FALSE(p_cfg != NULL, ESP_ERR_INVALID_ARG, PI4IOE5V9554_TAG, PI4IOE5V9554_CFG_ERROR_STR);
    ESP_RETURN_ON_FALSE(instance_id < RF_PI4IOE5V9554_MAX_INSTANCES, ESP_ERR_INVALID_ARG, PI4IOE5V9554_TAG, PI4IOE5V9554_INSTANCE_ID_ERROR_STR);
    ESP_RETURN_ON_FALSE(p_cfg->i2c_num >= 0 && p_cfg->i2c_num < I2C_NUM_MAX, ESP_ERR_INVALID_ARG, PI4IOE5V9554_TAG, PI4IOE5V9554_I2C_NUM_ERROR_STR);

    if(p_pi4ioe5v9554_obj[instance_id] != NULL) return ESP_FAIL;

    p_obj = (rf_pi4ioe5v9554_obj_t*)calloc(1, sizeof(rf_pi4ioe5v9554_obj_t));
    if(!p_obj) return ESP_ERR_NO_MEM;

    *p_obj = (rf_pi4ioe5v9554_obj_t) {
        .cfg = *p_cfg,
        .instance_id = instance_id,
    };

    p_obj->mux = xSemaphoreCreateMutex();
    if (p_obj->mux == NULL) {
        ESP_LOGE(PI4IOE5V9554_TAG, PI4IOE5V9554_SEM_ERR_STR);
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    if (p_obj->cfg.isr_handler != NULL) {
        err = gpio_isr_handler_add(p_obj->cfg.isr_pin, p_obj->cfg.isr_handler, (void*)&p_obj->instance_id);
        if (err != ESP_OK) {
            goto cleanup;
        }
    }

    p_pi4ioe5v9554_obj[instance_id] = p_obj;
    return ESP_OK;

cleanup:
    if (p_obj->mux) {
        vSemaphoreDelete(p_obj->mux);
        p_obj->mux = NULL;
    }
    free(p_obj);
    return err;
}

esp_err_t rf_pi4ioe5v9554_pin_cfg_mode(uint8_t instance_id, uint8_t pin_mask, rf_pi4ioe5v9554_pin_mode_t pin_mode, TickType_t ticks_to_wait)
{
    esp_err_t err = ESP_OK;
    portBASE_TYPE res;
    uint8_t mode;

    if(p_pi4ioe5v9554_obj[instance_id] == NULL) return ESP_ERR_NOT_FOUND;

    res = xSemaphoreTake(p_pi4ioe5v9554_obj[instance_id]->mux, ticks_to_wait);
    if (res == pdFALSE) {
        return ESP_ERR_TIMEOUT;
    }

    err = prv_pi4ioe5v9554_read_register(p_pi4ioe5v9554_obj[instance_id], PI4IOE5V9554_CMD_CONFIG, &mode, ticks_to_wait);
    if (err == ESP_OK) 
    {
        for(uint8_t i=0; i<8; i++)
        {
            if (pin_mask & (1 << i)) {
                mode = (mode & ~(1 << i)) | (pin_mode << i);
            }
        }
        err = prv_pi4ioe5v9554_write_register(p_pi4ioe5v9554_obj[instance_id], PI4IOE5V9554_CMD_CONFIG, mode, ticks_to_wait);
    }

    xSemaphoreGive(p_pi4ioe5v9554_obj[instance_id]->mux);
    return err;
}

esp_err_t rf_pi4ioe5v9554_pin_cfg_polarity(uint8_t instance_id, uint8_t pin_mask, rf_pi4ioe5v9554_pin_polarity_t pin_polarity, TickType_t ticks_to_wait)
{
    esp_err_t err = ESP_OK;
    portBASE_TYPE res;
    uint8_t polarity;

    if(p_pi4ioe5v9554_obj[instance_id] == NULL) return ESP_ERR_NOT_FOUND;

    res = xSemaphoreTake(p_pi4ioe5v9554_obj[instance_id]->mux, ticks_to_wait);
    if (res == pdFALSE) {
        return ESP_ERR_TIMEOUT;
    }

    err = prv_pi4ioe5v9554_read_register(p_pi4ioe5v9554_obj[instance_id], PI4IOE5V9554_CMD_POLARITY, &polarity, ticks_to_wait);
    if (err == ESP_OK) 
    {
        for(uint8_t i=0; i<8; i++) 
        {
            if(pin_mask & (1 << i)) {
                polarity = (polarity & ~(1 << i)) | (pin_polarity << i);
            }
        }
        err = prv_pi4ioe5v9554_write_register(p_pi4ioe5v9554_obj[instance_id], PI4IOE5V9554_CMD_POLARITY, polarity, ticks_to_wait);
    }

    xSemaphoreGive(p_pi4ioe5v9554_obj[instance_id]->mux);
    return err;   
}

esp_err_t rf_pi4ioe5v9554_pin_set_level(uint8_t instance_id, uint8_t pin_mask, rf_pi4ioe5v9554_pin_level_t pin_level, TickType_t ticks_to_wait)
{
    esp_err_t err = ESP_OK;
    portBASE_TYPE res;
    uint8_t output;

    if(p_pi4ioe5v9554_obj[instance_id] == NULL) return ESP_ERR_NOT_FOUND;

    res = xSemaphoreTake(p_pi4ioe5v9554_obj[instance_id]->mux, ticks_to_wait);
    if (res == pdFALSE) {
        return ESP_ERR_TIMEOUT;
    }

    err = prv_pi4ioe5v9554_read_register(p_pi4ioe5v9554_obj[instance_id], PI4IOE5V9554_CMD_OUTPUT, &output, ticks_to_wait);
    if (err == ESP_OK)
    {
        for(uint8_t i=0; i<8; i++)
        {
            if(pin_mask & (1 << i)) {
                output = (output & ~(1 << i)) | (pin_level << i);
            }
        }
        err = prv_pi4ioe5v9554_write_register(p_pi4ioe5v9554_obj[instance_id], PI4IOE5V9554_CMD_OUTPUT, output, ticks_to_wait);        
    }

    xSemaphoreGive(p_pi4ioe5v9554_obj[instance_id]->mux);
    return err;
}

esp_err_t rf_pi4ioe5v9554_pin_get_level(uint8_t instance_id, uint8_t* levels, TickType_t ticks_to_wait)
{
    esp_err_t err = ESP_OK;
    portBASE_TYPE res;

    if(p_pi4ioe5v9554_obj[instance_id] == NULL) return ESP_ERR_NOT_FOUND;

    res = xSemaphoreTake(p_pi4ioe5v9554_obj[instance_id]->mux, ticks_to_wait);
    if (res == pdFALSE) {
        return ESP_ERR_TIMEOUT;
    }

    err = prv_pi4ioe5v9554_read_register(p_pi4ioe5v9554_obj[instance_id], PI4IOE5V9554_CMD_INPUT, levels, ticks_to_wait);

    xSemaphoreGive(p_pi4ioe5v9554_obj[instance_id]->mux);
    return err;
}