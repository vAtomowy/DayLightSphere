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

#define BUTTON_GPIO             RF_PI4IOE5V9554_PIN_NUM_6
#define LED_GPIO                GPIO_NUM_7

#define PORT_EXPANDER_INSTANCE_ID 0                                            // OK
#define PORT_EXPANDER_I2C_NUM     0                                            // OK
#define PORT_EXPANDER_I2C_ADDR    0x20                                         // OK
#define PORT_EXPANDER_I2C_SDA     MIXER_SDA_GPIO                               // OK
#define PORT_EXPANDER_I2C_SCL     MIXER_SCL_GPIO                               // OK

#define BUTTON_TASK_NAME                "Button"
#define BUTTON_TASK_STACK_SIZE          (4096)
#define BUTTON_TASK_PRIORITY            (10)

#define ERROR_TASK_NAME                 "Error"
#define ERROR_TASK_STACK_SIZE           (4096) 
#define ERROR_TASK_PRIORITY             (10) 
