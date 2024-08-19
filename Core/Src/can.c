/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : can.c
  * @brief          : CAN protocol function definitions
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
#include "can.h"
#include "canMsgQueue.h"

void can_init() {
    // After the reset the CAN will be in the sleep mode
    // exit from sleep mode by clearing SLEEP bit. Wait until
    // the sleep acknowledgement.
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    while(0 != (CAN1->MSR & CAN_MSR_SLAK));

    // Request to enter into the initialization mode.
    // Wait until the CAN hardware acknowledge the init mode.
    CAN1->MCR |= CAN_MCR_INRQ;
    while(0 == (CAN1->MSR & CAN_MSR_INAK));

    // Disable the time triggered communication mode;
    // TODO - try by enabling.
    CAN1->MCR &= ~CAN_MCR_TTCM;

    // Disable automatic bus off management.
    // When too many errors detected by the CAN controller and
    // prevent further communication in through the bus. In
    // this case the CAN controller will cease all transmission.
    // This state is called Bus-off state. To recover from this
    // state; two options are there.
    // 1. by software intervention (ABOM = 0)
    // 2. Automatic (ABOM = 1)
    CAN1->MCR &= ~CAN_MCR_ABOM;

    // Disable the auto-wakeup; When ENABLED the CAN controller wakeup
    // automatically from sleep mode upon receiving the CAN message.
    // So the SLAK flag in the MSR and SLEEP bit in the MCR will be
    // cleared by hardware.
    CAN1->MCR &= ~CAN_MCR_AWUM;

    // Auto transmission disabled; the CAN controller will
    // be sending the data only once irrespective of sucessful,
    // error or arbitration lost.
    CAN1->MCR |= CAN_MCR_NART;

    // Don't lock receive FIFO on overrun. The new packet will
    // overwrite the previous message;
    CAN1->MCR &= ~CAN_MCR_RFLM;

    // Priority of transmit FIFO is based on the identifier
    // in the message.
    CAN1->MCR &= ~CAN_MCR_TXFP;

    // Run CAN in nomal mode; ie disable loopback and silent mode.
    CAN1->BTR &= ~(CAN_BTR_SILM | CAN_BTR_LBKM);

    // Set Resynchronization jump width bits as 0.
    // Total jump width = tq x (0 + 1) = tq. This is the number of
    // quanta in the CAN hardware is allowed to lengthen or shorten
    // to adjust the bit width for resynchronization.
    CAN1->BTR &= ~CAN_BTR_SJW;

    // TS1 = tq(b010 + 1) = 3tq; This include both progation segment
    // and phase segment 1. This segment defines the sampling point.
    // This segment can be lengthened if there is a positive phase drift
    // by the amount of maximum SJW bits.
    // Most importantly the sampling will happen after Phase segment 1;
    // This will ensure that logic level is sampled after bit trasmission
    // and propagation delay.
    CAN1->BTR |= CAN_BTR_TS1_1;

    // This is phase segment 2. This would be shorten when there is a
    // negative drift in the phase of CAN signal.
    CAN1->BTR |= (CAN_BTR_TS2_1 | CAN_BTR_TS2_0);

    // For 36MHz APB1, the prescalar is 36MHz/(8 + 1) = 4MHz;
    CAN1->BTR |= 8;
}

void can_config_filter_bank0() {
    // Deactivate filter corresponding to the selected bank
    // This is necessary to do before doing any modifications
    // in a CAN filter.
    CAN1->FA1R &= ~CAN_FA1R_FACT0;

    // Configure filter scale for bank0 for single 32 scale
    // configuration.
    CAN1->FS1R |= CAN_FS1R_FSC0;

    // Configure filter in mask mode
    CAN1->FM1R &= ~CAN_FM1R_FBM0;

    // Configure 32-bit identifier; we can configure any value here
    CAN1->sFilterRegister[0].FR1 = 0;

    // Configure 32-bit mask as 0, means every bits are don't care.
    // effectively the CAN will accept all the messages.
    CAN1->sFilterRegister[0].FR2 = 0;

    // Assign FIFO0 for the filter
    CAN1->FFA1R &= ~CAN_FFA1R_FFA0;

    // We have done the configuration. Enable the filter
    CAN1->FA1R |= CAN_FA1R_FACT0;

    // Switch to active filter mode
    CAN1->FMR &= ~CAN_FMR_FINIT;
}

void can_start() {
    // Exit from the initialization mode and enter into the
    // Normal mode of CAN module.
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while(CAN_MSR_INAK == (CAN1->MSR & CAN_MSR_INAK));

    CAN1->MCR &= ~CAN_MCR_TTCM;
}

void can_enable_interrupt() {
    // Generate interrupt when the message pending in FIFO0.
    CAN1->IER |= CAN_IER_FMPIE0;
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

void USB_LP_CAN1_RX0_IRQHandler(void) {
    CAN_Message msg;

    // Check that any data pending in FIFO0
    if(CAN1->RF0R & CAN_RF0R_FMP0) {
        // Read the header data

        // Get the standard or extended ID
        msg.ide = CAN1->sFIFOMailBox[0].RIR & CAN_RI0R_IDE;
        if(msg.ide == CAN_ID_STD) {
            msg.id.stdId = (CAN_RI0R_STID & CAN1->sFIFOMailBox[0].RIR) >> CAN_TI0R_STID_Pos;
        } else {
            msg.id.extId = ((CAN_RI0R_EXID|CAN_RI0R_STID) & CAN1->sFIFOMailBox[0].RIR) >> CAN_TI0R_EXID_Pos;
        }
        // Remote transmission request.
        msg.rtr = CAN_RI0R_RTR & CAN1->sFIFOMailBox[0].RIR;

        // Data length code - number of bytes in the frame (0 - 8); 0 means
        // this is a request from remote.
        msg.dlc = (CAN_RDT0R_DLC & CAN1->sFIFOMailBox[0].RDTR) >> CAN_RDT0R_DLC_Pos;
        // Filter match index
        msg.fmi = (CAN_RDT0R_FMI & CAN1->sFIFOMailBox[0].RDTR) >> CAN_RDT0R_FMI_Pos;
        // Message time stamp
        msg.ts = (CAN_RDT0R_TIME & CAN1->sFIFOMailBox[0].RDTR) >> CAN_RDT0R_TIME_Pos;

        // Get the data
        msg.data[0] = (CAN1->sFIFOMailBox[0].RDLR & CAN_RDL0R_DATA0) >> CAN_RDL0R_DATA0_Pos;
        msg.data[1] = (CAN1->sFIFOMailBox[0].RDLR & CAN_RDL0R_DATA1) >> CAN_RDL0R_DATA1_Pos;
        msg.data[2] = (CAN1->sFIFOMailBox[0].RDLR & CAN_RDL0R_DATA2) >> CAN_RDL0R_DATA2_Pos;
        msg.data[3] = (CAN1->sFIFOMailBox[0].RDLR & CAN_RDL0R_DATA3) >> CAN_RDL0R_DATA3_Pos;
        msg.data[4] = (CAN1->sFIFOMailBox[0].RDHR & CAN_RDH0R_DATA4) >> CAN_RDH0R_DATA4_Pos;
        msg.data[5] = (CAN1->sFIFOMailBox[0].RDHR & CAN_RDH0R_DATA5) >> CAN_RDH0R_DATA5_Pos;
        msg.data[6] = (CAN1->sFIFOMailBox[0].RDHR & CAN_RDH0R_DATA6) >> CAN_RDH0R_DATA6_Pos;
        msg.data[7] = (CAN1->sFIFOMailBox[0].RDHR & CAN_RDH0R_DATA7) >> CAN_RDH0R_DATA7_Pos;

        CMQ_AddMsg(&msg);

        // Release the FIFO
        CAN1->RF0R |= CAN_RF0R_RFOM0;
    }
}
