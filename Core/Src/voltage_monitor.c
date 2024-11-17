/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pump_controller.c
  * @brief          : The driver functions related to the water pump
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
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "voltage_monitor.h"
#include "uart.h"

#define     NUM_INPUT_VOLT_SAMPLE        105
#define     VREF_VOLT                    3.3
#define     ADC_RESOLUTION               4095.0
#define     MULTIPLIER                   1200.0

volatile uint16_t adc_buff[NUM_INPUT_VOLT_SAMPLE];
static volatile uint16_t line_volt = 0;

static void timer_1_500us_cc1() {
    // Enable system clock to TIM1
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Set pre-scalar 8; so each count will take
    // 1us delay since we are using 8MHz clock
    TIM1->PSC = 71;

    // Count 0 - 499 (both inclusive - so total 500)
    TIM1->ARR = 499;

    // Clear the counter register as 0
    TIM1->CNT = 0;

    // Clear the counter reset flag and enable
    // the update interrupt which will trigger
    // on the counter reset.
    TIM1->SR &= ~(TIM_SR_UIF);
    TIM1->DIER |= TIM_DIER_UIE;
    TIM1->CCR1 = 499;
    TIM1->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0);

    // Capture compare enable
    TIM1->CCER |= TIM_CCER_CC1E;

    // Enable PA8 (OC) if corresponding OCxE is set in the CCER register
    TIM1->BDTR |= TIM_BDTR_MOE;
}


void init_voltage_monitor() {
    // Intialize the timer that trigger
    // every 500us
    timer_1_500us_cc1();

    // Enable the clock for ADC1, GPIOA and DMA1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // select the ADC channel 1
    ADC1->SQR3 = 1;

    // Configure ADC1 CR2 register, DMA, enable ADC
    // Start conversion on external trigger and the
    // external trigger is compare match CC1 of TIM1
    ADC1->CR2 |= ADC_CR2_DMA;
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_EXTTRIG;
    ADC1->CR2 &= ~ADC_CR2_EXTSEL;

    // Configure DMA1 Channel 1 for ADC1
    // Peripheral address (ADC data register)
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    // Memory address (adc_buff)
    DMA1_Channel1->CMAR = (uint32_t)adc_buff;

    // Increment memory address and peripheral addres
    // remain same. The peripheral and memory data sizes
    // are 16-bits and finally enable the DMA channel 1
    DMA1_Channel1->CCR |= DMA_CCR_MINC;
    DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;
	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    // Start ADC conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

static void start_adc_conversion(uint8_t count) {
    // Disable DMA channel, configure the data to be
    // transferred (here 100 samples) and then
    // enable the DMA
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CNDTR = count;
    DMA1_Channel1->CCR |= DMA_CCR_EN;
}

#if DEBUG_ENABLED // TODO: should be moved from here
void double2str(double value, char* str, int precision) {
    // Handle the sign
    if (value < 0) {
        *str++ = '-';
        value = -value;
    }

    // Extract integer part
    // the int_part should be long long; but here we used
    // long as a work around.
    long int_part = (long)value;
    double frac_part = value - int_part;

    // Convert integer part to string
    sprintf(str, "%ld", int_part);

    // Move the pointer to the end of the integer part
    while (*str != '\0') {
        str++;
    }

    // Add the decimal point
    *str++ = '.';

    // Handle the fractional part
    for (int i = 0; i < precision; i++) {
        frac_part *= 10;
        int digit = (int)frac_part;
        *str++ = '0' + digit;
        frac_part -= digit;
    }

    // Null-terminate the string
    *str = '\0';
}
#endif

#define     VOLTAGE_LEVEL_GOOD   220
#define     VOLTAGE_LEVEL_OK     200
#define     VOLTAGE_LEVEL_LOW    180
#define     VOLTAGE_LEVEL_WARN   160

uint16_t get_current_voltage() {
    return line_volt;
}

voltage_level_t get_line_voltage_level() {
    if(line_volt >= VOLTAGE_LEVEL_GOOD) {
        return VOLTAGE_GOOD;
    } else if(line_volt >= VOLTAGE_LEVEL_OK) {
        return VOLTAGE_OK;
    } else if(line_volt >= VOLTAGE_LEVEL_LOW) {
        return VOLTAGE_LOW;
    } else if(line_volt >= VOLTAGE_LEVEL_WARN) {
        return VOLTAGE_WARN;
    } else {
        return VOLTAGE_CRIT;
    }
}

uint16_t measure_current_voltage(uint8_t repeat) {
    uint16_t i, j = 0;
    uint32_t sum;
    uint32_t sq_sum;
    int16_t ac_voltage;
    uint16_t zero_point;
    double reading_voltage;
    double volt_sum = 0;
#ifdef DEBUG_ENABLED
    // char buff[16];
#endif

    if(0 == repeat)
        return 0;

    // Start timer 500us timer to covnert the ADC in every 500uS
    // See the timer_1_500us_cc1 function for the initialization
    TIM1->CR1 |= TIM_CR1_CEN;
    // Enable the DMA to transfer 'NUM_INPUT_VOLT_SAMPLE' converted
    // data into the memory
    start_adc_conversion(NUM_INPUT_VOLT_SAMPLE);

    while(1) {

        // When NUM_INPUT_VOLT_SAMPLE conversions are done the TCIF1
        // flag will be set int the DMA1_ISR
        if(DMA1->ISR & DMA_ISR_TCIF1) {
            DMA1->IFCR |= DMA_ISR_TCIF1;

            // Calculate the DC value (zero point) from which the AC
            // is oscillating
            sum = 0;
            sq_sum = 0;
            for(i = 0; i < NUM_INPUT_VOLT_SAMPLE; i++) {
                sum += adc_buff[i];
                // uart1_send_string("%d", adc_buff[i]);
            }
            zero_point = sum / NUM_INPUT_VOLT_SAMPLE;

            // Calculate the RMS value of the AC voltage
            // take the square of each sample taken in 500us delay
            // and devide it with maxium sample. Then convert it to
            // actual ADC voltage
            for(i = 0; i < NUM_INPUT_VOLT_SAMPLE; i++) {
                ac_voltage = adc_buff[i] - zero_point;
                sq_sum += (ac_voltage * ac_voltage);
            }

            reading_voltage = sqrt(sq_sum/NUM_INPUT_VOLT_SAMPLE) * VREF_VOLT * MULTIPLIER / ADC_RESOLUTION;
#ifdef DEBUG_ENABLED
            // Caution: Uncommenting this section will leads to I2C timeout
            char buff[16];
            double2str(reading_voltage, buff, 7);
            uart1_send_string("voltage = %u - %s\r\n", (uint16_t)reading_voltage, buff);
#endif
            // Try as many times as requested and take the avarage to 
            // get the actual line voltage
            volt_sum += reading_voltage;

            if(repeat > ++j) {
                start_adc_conversion(NUM_INPUT_VOLT_SAMPLE);
            } else {
                break;
            }

        }
    }

    line_volt = (uint16_t)(volt_sum/repeat);

#ifdef DEBUG_ENABLE1D
    uart1_send_string("Average voltage = %u\r\n", line_volt);
#endif
    return line_volt;
}
