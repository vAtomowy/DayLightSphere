/**
 * @file HWConfig.h
 * @author Artur Bereit (abereit@softblue.pl)
 * @brief 
 * @version 0.1
 * @date 2024-06-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include "pi4ioe5v9554.h"

#define YELLOW_LED                      GPIO_NUM_7
#define YELLOW_LED_TASK_NAME            "Yellow_LED"
#define YELLOW_LED_TASK_STACK_SIZE      (4096)
#define YELLOW_LED_TASK_PRIORITY        (10)

#define RED_LED                         GPIO_NUM_6
#define RED_LED_TASK_NAME               "Red_LED"
#define RED_LED_TASK_STACK_SIZE         (4096)
#define RED_LED_TASK_PRIORITY           (10)

#define KEY1_GPIO                       GPIO_NUM_9
#define KEY1_TASK_NAME                  "Key1"
#define KEY1_TASK_STACK_SIZE            (4096)
#define KEY1_TASK_PRIORITY              (10)

#define KEY2_GPIO                       GPIO_NUM_8
#define KEY2_TASK_NAME                  "Key2"
#define KEY2_TASK_STACK_SIZE            (4096)
#define KEY2_TASK_PRIORITY              (10)

#define DRV1_GPIO                       GPIO_NUM_18    
#define DRV1_GPIO_CHANNEL               LEDC_CHANNEL_0       
#define DRV1_TASK_NAME                  "Driver1"
#define DRV1_TASK_STACK_SIZE            (4096)
#define DRV1_TASK_PRIORITY              (10)

#define DRV2_GPIO                       GPIO_NUM_19   
#define DRV2_GPIO_CHANNEL               LEDC_CHANNEL_1               
#define DRV2_TASK_NAME                  "Driver2"
#define DRV2_TASK_STACK_SIZE            (4096)
#define DRV2_TASK_PRIORITY              (10)


#define PORT_EXPANDER_INSTANCE_ID 0                                            // OK
#define PORT_EXPANDER_I2C_NUM     0                                            // OK
#define PORT_EXPANDER_I2C_ADDR    0x20                                         // OK
#define PORT_EXPANDER_I2C_SDA     MIXER_SDA_GPIO                               // OK
#define PORT_EXPANDER_I2C_SCL     MIXER_SCL_GPIO                               // OK


