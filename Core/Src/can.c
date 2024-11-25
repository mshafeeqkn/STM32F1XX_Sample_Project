#include "can.h"

#define SEL_FILTER_BANK             0

void can_init() {
    uint32_t prio_grp;

    // Enable the clock for CAN peripheral
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    // CAN pins are PA11 and PA12, enable GIPOA clock
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // Configure PA11(CAN RX) as input floating
    // This is reset state, no need to change register value

    // Configure PA12 (CAN TX) as output alternate function push pull
    GPIOA->CRH |= GPIO_CRH_CNF12_1 | GPIO_CRH_MODE12_0;

    // Enable CAN interrupt
    prio_grp = NVIC_GetPriorityGrouping();
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, NVIC_EncodePriority(prio_grp, 0, 0));
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_SetPriority(CAN1_RX1_IRQn, NVIC_EncodePriority(prio_grp, 0, 0));
    NVIC_EnableIRQ(CAN1_RX1_IRQn);

    // Request CAN initialization and wait until 
    // the CAN complete initialization
    CAN1->MCR |= CAN_MCR_INRQ;
    while(0 == (CAN1->MSR & CAN_MSR_INAK));

    // Exit from sleep mode, and wait until the CAN exit
    // from the sleep mode
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    while(0 != (CAN1->MSR & CAN_MSR_SLAK));

    // Leaving the rest of configurations in
    // the MCR as disabled

    // Set bit timing register. Loopback, sync jump
    // width as 0, TS1 = 2TQ, and TS2 = 1TQ
    CAN1->BTR |= (uint32_t) (CAN_BTR_LBKM | CAN_BTR_TS1_0);
}

void can_config_filter() {
    const uint32_t message_id = 0x102 << CAN_F0R1_FB21_Pos;
    const uint32_t filter_mask = 0x103 << CAN_F0R1_FB21_Pos;

    // FINIT bit has to be set to initialize CAN filter bank
    // related registers
    CAN1->FMR |= CAN_FMR_FINIT;

    // We are using the filter bank SEL_FILTER_BANK to filter the
    // received packets. Inorder to configure filter bank 0,
    // deactivate the filter bank first.
    CAN1->FA1R &= ~(1 << SEL_FILTER_BANK);

    // Configure filter scale as single 32 bit for SEL_FILTER_BANK
    CAN1->FS1R |= (1 << SEL_FILTER_BANK);

    CAN1->sFilterRegister[SEL_FILTER_BANK].FR1 = message_id;
    CAN1->sFilterRegister[SEL_FILTER_BANK].FR2 = filter_mask;

    // Configure filter mode to use FR1 as ID and FR2 as Mask
    CAN1->FM1R &= ~(1 << SEL_FILTER_BANK);

    // Assign FIFO-0 for SEL_FILTER_BANK
    CAN1->FFA1R &= ~(1 << SEL_FILTER_BANK);

    // Finally re-enable filter bank 0
    CAN1->FA1R |= (1 << SEL_FILTER_BANK);

    // Leave from the initialization mode
    CAN1->FMR &= ~CAN_FMR_FINIT;
}
