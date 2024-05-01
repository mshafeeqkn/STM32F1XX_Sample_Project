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

#include <FreeRTOS.h>
#include <task.h>

TaskHandle_t xTaskHandle1 = NULL;
TaskHandle_t xTaskHandle2 = NULL;

/* The task functions prototype*/
void vTask1(void *pvParameters);
void vTask2(void *pvParameters);

void delay(uint32_t ms) {
    // Simple delay function (not accurate, just for demonstration)
    for (volatile uint32_t i = 0; i < ms * 1000; ++i) {
        __NOP();  // No operation (compiler barrier)
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    // Enable clock for GPIOC peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    // Configure GPIO pin as output
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);  // Clear configuration
    GPIOC->CRH |= GPIO_CRH_MODE13_0;  // Set pin mode to general purpose output (max speed 10 MHz)
    /* Create one of the two tasks. */
    xTaskCreate(vTask1, "Task-1", configMINIMAL_STACK_SIZE, NULL, 2, &xTaskHandle1);
    /* Create second task */
    // xTaskCreate(vTask2, "Task-2", configMINIMAL_STACK_SIZE, NULL, 2, &xTaskHandle2);

    /* Start the scheduler */
    vTaskStartScheduler();

#if 0
    while (1) {
        // Toggle LED pin
        GPIOC->ODR ^= 0x2000;
        // Delay for some time
        // delay(1000);
        vTaskDelay(150000/portTICK_PERIOD_MS);
    }
#endif
}

void vTask1(void *pvParameters)
{
    /* As per most tasks, this task is implemented in an infinite loop. */
    while(1) {
        GPIOC->ODR ^= 0x2000;
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
/*-----------------------------------------------------------*/

void vTask2(void *pvParameters)
{
    /* As per most tasks, this task is implemented in an infinite loop. */
    while(1) {
        GPIOC->ODR ^= 0x2000;
        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}

