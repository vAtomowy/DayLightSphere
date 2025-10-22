/**
 * @file HWConfig.h
 * @author Artur Bereit
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

#define KEY1_GPIO                       GPIO_NUM_10
#define KEY1_TASK_NAME                  "Key1"
#define KEY1_TASK_STACK_SIZE            (4096)
#define KEY1_TASK_PRIORITY              (10)

#define I2C_A_PORT I2C_NUM_0
#define I2C_A_PIN_SDA                   GPIO_NUM_5
#define I2C_A_PIN_SCL                   GPIO_NUM_4
#define I2C_A_MASTER_SDA_IO             5
#define I2C_A_MASTER_SCL_IO             4
#define I2C_A_MASTER_FREQ_HZ            100000

// Found device at 0x60                                      
// Found device at 0x61

#define I2C_B_PORT I2C_NUM_0
#define I2C_B_PIN_SDA                   GPIO_NUM_2
#define I2C_B_PIN_SCL                   GPIO_NUM_3
#define I2C_B_MASTER_SDA_IO             2
#define I2C_B_MASTER_SCL_IO             3
#define I2C_B_MASTER_FREQ_HZ            100000

// Found device at 0x50                                      
// Found device at 0x51                                      
// Found device at 0x52                                      
// Found device at 0x53                                      
// Found device at 0x54                                                        
// Found device at 0x55
// Found device at 0x56
// Found device at 0x57
  
