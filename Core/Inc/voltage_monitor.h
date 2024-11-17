/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pump_controller.h
  * @brief          : Header for pump_controller.c file.
  *                   This file contains the defines of function to drive the
  *                   water pump
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VOLTAGE_MONITOR_H
#define __VOLTAGE_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum {
    VOLTAGE_CRIT,
    VOLTAGE_WARN,
    VOLTAGE_LOW,
    VOLTAGE_OK,
    VOLTAGE_GOOD
} voltage_level_t;

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void init_voltage_monitor();
uint16_t measure_current_voltage(uint8_t repeat);
voltage_level_t get_line_voltage_level();
uint16_t get_current_voltage();

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __VOLTAGE_MONITOR_H */
