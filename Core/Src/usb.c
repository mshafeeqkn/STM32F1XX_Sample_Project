/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb.c
  * @brief          : Program to configure the USB and communicate to a host PC
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
#include "usb.h"

volatile uint8_t reset_count = 5;

void USB_LP_CAN1_RX0_IRQHandler() {
    USB->ISTR &= ~USB_ISTR_RESET;
    reset_count++;
}

void init_usb(void) {

    // Enable clock for the USB
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;

    // Clear power down bit
    USB->CNTR &= ~USB_CNTR_PDWN;

    // Give a delay of tSTARTUP require 1uS, but
    // I am giving 1ms delay
    // 18000 loop * 4 clock per loop ~ 72000 clocks;
    for(uint16_t i = 0; i < 18000; i++);

    // Clear the reset and power down bits
    USB->CNTR &= ~USB_CNTR_FRES;

    // Clear interrupt flags if any
    USB->ISTR = 0;

    // Enable the USB interrupt
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    // Enable the USB reset interrupt
    USB->CNTR |= USB_CNTR_RESETM;
}
