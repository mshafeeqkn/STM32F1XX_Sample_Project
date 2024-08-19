/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "led.h"
#include "can.h"

/**
 * @brief Configure the system clock as 8MHz using
 * external crystal oscillator.
 */
void config_sys_clock() {
    // Enable HSE (High-Speed External) oscillator
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0);  // Wait for HSE to be ready

    // Select HSE as the system clock source
    RCC->CFGR &= ~RCC_CFGR_SW;  // Clear SW bits
    RCC->CFGR |= RCC_CFGR_SW_HSE;  // Set SW bits to select HSE as system clock

    // Wait until HSE is used as the system clock source
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    config_sys_clock();
    can_init();
    can_config_filter_bank0();
    can_enable_interrupt();
    can_start();

    while(1) {
        TOGGLE_LED();
    }
}
