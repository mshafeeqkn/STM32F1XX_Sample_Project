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

#define UART_TX_ENABLE           0x01
#define UART_RX_ENABLE           0x02

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

/**
 * @brief Turn on/off/toggle the built in LED
 *
 * @param state - TURN_TOGGLE
 *                TURN_ON_LED
 *                TURN_OFF_LED
 */
void turn_led_on(LedState_t state) {
    if(state == TURN_TOGGLE){
        GPIOC->ODR ^= GPIO_ODR_ODR13;
    } else if(state == TURN_ON) {
        GPIOC->ODR &= ~(GPIO_ODR_ODR13);
    } else {
        GPIOC->ODR |= GPIO_ODR_ODR13;
    }
}

void EXTI0_IRQHandler(void)
{
    TOGGLE_LED();
}

/**
 * @brief Send a byte through the UART1
 *
 * @param byte to be sent
 */
void uart1_send_byte(uint8_t ch) {
    while((USART1->SR & USART_SR_TXE) == 0) {}
    USART1->DR = ch;
}

/**
 * @brief Send formatted string through UART1
 *
 * @param format - formatted string
 */
void uart1_send_string(const char *format, ...) {
    va_list args;
    char buffer[128];
    size_t i, len;

    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);

    len = strlen(buffer);
    for(i = 0; i < len; i++) {
        uart1_send_byte(buffer[i]);
    }

    // Wait until transmission complted
    while((USART1->SR & USART_SR_TC) == 0) {}
}

/**
 * @brief Setup the UART1 for transmit or recieve
 *
 * @param uart_mode - UART_TX_ENABLE, UART_RX_ENABLE
 *                    or both
 */
void uart1_setup(uint8_t uart_mode) {
    uint16_t uart1_cr1_flags = 0;

    // Enable clock to USART1, Alternate function IO and GPIOA
    RCC->APB2ENR |= (RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN);

    // Baud rate 2400 @ 8MHz clock frequency
    USART1->BRR = 0xD05;
    if(0 != uart_mode) {
        // Global UART enable if either Tx or Rx enabled.
        uart1_cr1_flags |= USART_CR1_UE;

        // Enable Transmit mode if required
        if(uart_mode & UART_TX_ENABLE) {
            uart1_cr1_flags |= USART_CR1_TE;

            // Configure PA9 as output; 10MHz max; push pull
            GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
            GPIOA->CRH |= (GPIO_CRH_MODE9_0 | GPIO_CRH_CNF9_1);
        }

        // Enable Receive mode if required
        if(uart_mode & UART_RX_ENABLE) {
            uart1_cr1_flags |= USART_CR1_RE;

            // Configure PA10 input pull-up/pull-down
            GPIOA->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
            GPIOA->CRH |= GPIO_CRH_CNF10_1;
        }

    }

    // Enable UART and required mode
    USART1->CR1 |= uart1_cr1_flags;
}

/**
 * @brief receive a byte from UART1
 */
uint8_t uart1_get_byte() {
    uint8_t data;

    // Wait until new data arrive
    while((USART1->SR & USART_SR_RXNE) == 0) {}

    // Take the data;
    data = USART1->DR;

    // CLear the received data flag
    USART1->SR &= ~USART_SR_RXNE;

    return data;
}

/**
 * @brief The function to read the line of data from UART1
 * The function will return if any \r char detected or
 * len number of charector received.
 *
 * @param buff - The pointer to the buffer
 * @param len - Maximum buffer length
 */
void uart1_get_string(char *buff, size_t len) {
    int i = 0;
    do {
        buff[i] = uart1_get_byte();
        // Receive till the enter key is pressed
        if('\r' == buff[i]) {
            break;
        }
        i++;
    // Loop until the buffer is full
    } while(i < len);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    int temp = 30;
    char buff[128] = {0};

    uart1_setup(UART_TX_ENABLE | UART_RX_ENABLE);

    // Enable clock for GPIOC peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    // Configure GPIO pin as output
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);  // Clear configuration
    GPIOC->CRH |= GPIO_CRH_MODE13_0;  // Set pin mode to general purpose output (max speed 10 MHz)

    delay(1000);
    uart1_send_string("\r\nShafeeque; temp = %d\r\nKunnath Naduthodi house\r\nPappinppara\r\nManjeri\r\nMalappuram\r\n", temp);
    while (1) {
        uart1_get_string(buff, 128);
        uart1_send_string(buff);
        uart1_send_byte('\n');
        memset(buff, 0, 128);
    }
}

