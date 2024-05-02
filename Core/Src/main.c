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
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#define TURN_ON_LED()            turn_led_on(TURN_ON)
#define TURN_OFF_LED()           turn_led_on(TURN_OFF)
#define TOGGLE_LED()             turn_led_on(TURN_TOGGLE)

typedef enum {
    TURN_OFF,
    TURN_ON,
    TURN_TOGGLE
} LedState_t;

void delay(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 100; ++i) {
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

void uart1_send_char(char ch) {
    TURN_ON_LED();
    while((USART1->SR & USART_SR_TXE) == 0) {}
    USART1->DR = ch;
    TURN_OFF_LED();
}

// Function that takes a format string and a variable number of arguments
void uart1_send_string(const char *format, ...) {
    va_list args;
    char buffer[64];
    size_t i, len;

    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);

    len = strlen(buffer);
    for(i = 0; i < len; i++) {
        uart1_send_char(buffer[i]);
    }

    // Wait until transmission complted
    while((USART1->SR & USART_SR_TC) == 0) {}
}

void uart1_setup() {
    // Enable clock to USART1, Alternate function IO and GPIOA
    RCC->APB2ENR |= (RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN);

    // Configure PA9 as output; 10MHz max; push pull
    GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    GPIOA->CRH |= (GPIO_CRH_MODE9_0 | GPIO_CRH_CNF9_1);

    // Baud rate 2400 @ 8MHz clock frequency
    USART1->BRR = 0xD05;

    // USART Tx enable and USART enable
    USART1->CR1 |= (USART_CR1_TE | USART_CR1_UE);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    int age = 34;

    uart1_setup();
    // Enable clock for GPIOC peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    // Configure GPIO pin as output
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);  // Clear configuration
    GPIOC->CRH |= GPIO_CRH_MODE13_0;  // Set pin mode to general purpose output (max speed 10 MHz)

    delay(1000);
    uart1_send_string("\r\nShafeeque; age = %d\r\nKunnath Naduthodi house\r\nPappinppara\r\nManjeri\r\nMalappuram\r\n", age);
    while (1) {
    }
}

