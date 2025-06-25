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


#define CURR_SENSE_I2C_PORT             I2C_NUM_0
#define CURR_SENSE_SDA_PIN              GPIO_NUM_4
#define CURR_SENSE_SCL_PIN              GPIO_NUM_5

#define CURR_SENSE_COLD_ID_INSTANCE     0
#define CURR_SENSE_COLD_I2C_ADDR        0x40
#define CURR_SENSE_COLD_TASK_NAME       "CurrSenseCold"
#define CURR_SENSE_COLD_TASK_STACK_SIZE (4096)
#define CURR_SENSE_COLD_TASK_PRIORITY   (10)

#define CURR_SENSE_WARM_ID_INSTANCE     1
#define CURR_SENSE_WARM_I2C_ADDR        0x41
#define CURR_SENSE_WARM_TASK_NAME       "CurrSenseWarm"
#define CURR_SENSE_WARM_TASK_STACK_SIZE (4096)
#define CURR_SENSE_WARM_TASK_PRIORITY   (10)

#define SHUNT_RESISTOR_OHMS             0.15


