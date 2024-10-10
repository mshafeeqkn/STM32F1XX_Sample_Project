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
#include "uart.h"
#include "string.h"

#define EPR_NON_TOGGLE_BITS         USB_EPREG_MASK
#define PMA_BASE_ADDR               0x40006000

typedef uint16_t                    PMAWord_t;

extern PMAWord_t _pma_end;

static void usb_reset(void) {
}

void USB_LP_CAN1_RX0_IRQHandler() {
    volatile uint16_t usb_status = USB->ISTR;

    if(usb_status & USB_ISTR_RESET) {
        usb_reset();
        USB->ISTR &= ~USB_ISTR_RESET;
    }

    if(usb_status & USB_ISTR_SOF) {
        USB->ISTR &= ~USB_ISTR_SOF;
    }

    if(usb_status & USB_ISTR_ESOF) {
        USB->ISTR &= ~USB_ISTR_ESOF;
    }

    if (usb_status & USB_ISTR_SUSP) {
        USB->ISTR &= ~USB_ISTR_SUSP;
    }
    
    if (usb_status & USB_ISTR_WKUP) {
        USB->ISTR &= ~USB_ISTR_WKUP;
    }

    if (usb_status & USB_ISTR_ERR) {
        USB->ISTR &= ~USB_ISTR_ERR;
    }

    if (usb_status & USB_ISTR_PMAOVR) {
        USB->ISTR &= ~USB_ISTR_PMAOVR;
    }

    while((usb_status = USB->ISTR) & USB_ISTR_CTR) {
        USB->EP0R = USB->EP0R & EPR_NON_TOGGLE_BITS & ~USB_EP_CTR_RX;
    }
}

void init_usb(void) {
}
