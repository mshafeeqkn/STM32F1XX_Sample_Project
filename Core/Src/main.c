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

#define TURN_ON_LED()            turn_led_on(TURN_ON)
#define TURN_OFF_LED()           turn_led_on(TURN_OFF)
#define TOGGLE_LED()             turn_led_on(TURN_TOGGLE)

typedef enum {
    TURN_OFF,
    TURN_ON,
    TURN_TOGGLE
} LedState_t;

void turn_led_on(LedState_t state);

void EXTI0_IRQHandler() {
    EXTI->PR |= EXTI_PR_PR0;
    TOGGLE_LED();
}

void turn_led_on(LedState_t state) {
    if(state == TURN_TOGGLE){
        GPIOC->ODR ^= GPIO_ODR_ODR13;
    } else if(state == TURN_ON) {
        GPIOC->ODR &= ~(GPIO_ODR_ODR13);
    } else {
        GPIOC->ODR |= GPIO_ODR_ODR13;
    }
}

void setup_exti0_interrupt() {
    // Set PB0 as input
    GPIOB->CRL &= ~GPIO_CRL_MODE0;

    // Configure PB0 as input pull-down/pull-up
    GPIOB->CRL &= ~GPIO_CRL_CNF0;
    GPIOB->CRL |= GPIO_CRL_CNF0_1;

    // Configure PB0 as pull-up
    GPIOB->ODR |= GPIO_ODR_ODR0;

    // Tie EXTI0 with PB0
    AFIO->EXTICR[0] &= ~AFIO_EXTICR1_EXTI0;
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PB;

    // Trigger on falling edge
    EXTI->FTSR |= EXTI_FTSR_TR0;

    // Mask EXTI0
    EXTI->IMR |= EXTI_IMR_MR0;

    // Clear the flag
    EXTI->PR |= EXTI_PR_PR0;

    // Enable the interrupt line at NVIC
    uint32_t prioritygroup = NVIC_GetPriorityGrouping();
    NVIC_SetPriority(EXTI0_IRQn, NVIC_EncodePriority(prioritygroup, 10, 0));
    NVIC_EnableIRQ(EXTI0_IRQn);
}

int main(void)
{
    // Enable clock for GPIOC peripheral
    RCC->APB2ENR |= (RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN);

    // Configure GPIO pin as output
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);  // Clear configuration
    GPIOC->CRH |= GPIO_CRH_MODE13_0;  // Set pin mode to general purpose output (max speed 10 MHz)

    setup_exti0_interrupt();

    while (1) {
    }
}

