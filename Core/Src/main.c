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

void delay(uint32_t ms) {
    // Simple delay function (not accurate, just for demonstration)
    for (volatile uint32_t i = 0; i < ms * 1000; ++i) {
        __NOP();  // No operation (compiler barrier)
    }
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

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    uint16_t adcDelay = 0xFFF;

    // Enable clock for GPIOC & ADC1 peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Configure GPIO pin as output
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);  // Clear configuration
    GPIOC->CRH |= GPIO_CRH_MODE13_0;  // Set pin mode to general purpose output (max speed 10 MHz)

    // GPIOA1 analog input mode
    GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);

    // Caliberate ADC after each power up
    ADC1->CR2 |= ADC_CR2_ADON;
    for(int i = 0; i < 3; i++);
    ADC1->CR2 |= ADC_CR2_CAL;
    while(ADC1->CR2 & ADC_CR2_CAL);
    ADC1->CR2 &= ~(ADC_CR2_ADON);
    // Enable scan mode
    // ADC->CR1 |= ADC_CR1_SCAN;

    // Continuos mode
    // ADC->CR2 |= ADC_CR2_CONT;

    // Right alignment
    // ADC->CR2 &= ~(ADC_CR2_ALIGN);

    // ADC channel 0
    ADC1->SQR3 = ADC_SQR3_SQ2;

    // Enable ADC
    ADC1->CR2 |= ADC_CR2_ADON;

    // Start Regular Conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    while(ADC1->SR & ADC_SR_EOC);
    adcDelay = ADC1->DR;

    while (1) {
        TOGGLE_LED();
        delay(adcDelay);
    }
}

