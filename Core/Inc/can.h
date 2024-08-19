/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : can.h
  * @brief          : Header for can.c file.
  *                   This file contains the CAN related routines.
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
#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct {
    uint16_t ts;
    union {
        uint16_t stdId;
        uint32_t extId;
    } id;
    uint8_t data[8];
    uint8_t fmi;
    uint8_t dlc:4;
    uint8_t ide:1;
    uint8_t rtr:1;
} CAN_Message;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */
void can_init();
void can_config_filter_bank0();
void can_start();
void can_enable_interrupt();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

#define CAN_ID_STD               0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */
