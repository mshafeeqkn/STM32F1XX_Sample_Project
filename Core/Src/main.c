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

// Macro defines
#define TURN_ON_LED()            turn_led_on(TURN_ON)
#define TURN_OFF_LED()           turn_led_on(TURN_OFF)
#define TOGGLE_LED()             turn_led_on(TURN_TOGGLE)
#define CHIP_ADDR                0x50

/**
 * Enumerations
 */
typedef enum {
    TURN_OFF,
    TURN_ON,
    TURN_TOGGLE
} LedState_t;

void delay(uint32_t tick) {
    for (volatile uint32_t i = 0; i < tick * 1; ++i) {
        __NOP();  // No operation (compiler barrier)
    }
}

/**
 * @brief Trun the built in LED on/off/toggle
 *
 * @param state - TURN_ON / TURN_OFF / TURN_TOGGLE
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

void i2c_init() {
    // Enable clock for GPIOB, Alternate IO
    // and I2C peripherals
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Configure PB6 and PB7 as output mode with max speed 10MHz
    // with alternate function as open-drain
    GPIOB->CRL |= (GPIO_CRL_CNF6 | GPIO_CRL_MODE6_0);
    GPIOB->CRL |= (GPIO_CRL_CNF7 | GPIO_CRL_MODE7_0);

    // Reset I2C module and wait until BUSY
    // flag get cleared
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;
    while((I2C1->SR2 & I2C_SR2_BUSY));

    // APB clock frequency as 8MHz
    I2C1->CR2 |= 0x08;

    // 100KHz normal mode; clock time period = 10uS
    // Clock speed 8MHz; time period = 125nS
    // Assuem i2c dutycycle 0.5, I2C high time = 5uS = 5000ns
    // Number of system clock required = 5000/125 = 40d = 0x28
    I2C1->CCR |= 0x28;

    // Rising edge maximum time = 1000ns;
    // Number of system clock 1000/125 + 1 = 8 + 1 = 9
    I2C1->TRISE |= 0x09;

    // Enable I2C peripheral
    I2C1->CR1 |= I2C_CR1_PE;
}

void i2c_start() {
    // Generate start condition, wait until the SB bit set
    // clear the SB flag by reading SR1 register
    I2C1->CR1 |= I2C_CR1_START;
    while((I2C1->SR1 & I2C_SR1_SB) == 0);
    (void)I2C1->SR1;
}

void i2c_addr(uint8_t addr, uint8_t is_read) {
    // Write address into the data register with write/read flag
    // Wait until the ADDR flag set and clear the ADDR flag by
    // reading SR1 and SR2.
    I2C1->DR = (addr << 1) | is_read;
    while(!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR1;
    (void)I2C1->SR2;
}

void i2c_send_byte(uint8_t byte) {
    // Wait until the transmit buffer empty
    while(!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = byte;
    // Wait until transmit finished
    while(!(I2C1->SR1 & I2C_SR1_BTF));
}

void i2c_stop() {
    // Generate stop condition
    I2C1->CR1 |= I2C_CR1_STOP;
}

int main(void) {
    config_sys_clock();

    i2c_init();
    i2c_start();
    i2c_addr(CHIP_ADDR, 0);
    i2c_send_byte(0x00);
    i2c_send_byte(0x01);
    i2c_send_byte(0x02);
    i2c_send_byte(0x03);
    i2c_send_byte(0x04);
    i2c_stop();

    while(1) {
    }
}

