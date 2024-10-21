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

#include <string.h>

// Section name, flags and sub-properties of the memory that is
// going to use by linkers
// .pma -> Name of the section where the variable should be placed
// aw -> a means it will be allocated just before program startup
//       w means the memory is writable
// %nobits -> This space doesn't need to occupy in the object or
//      binary files. ie, the linker won't expect any data associated.
//      this is useful for those variable which doesn't require
//      initialization data.
// used -> The variable shouldn't be removed during the optimization
//      even if there is no usage.
// aligned(8) -> The data will be aligned in 8byte memory address.
//      As per the reference manual page 627, the buffer descriptor
//      table start byte address should be 8-byte aligned.
//      (means multiple of 8). Since each member in this array have
//      size of 8 byte, all members becomes 8 byte aligned.
#define __PMA_SECTION__             ".pma,\"aw\",%nobits//"
#define __PMA_BDT_ATTR__            __attribute__((section(__PMA_SECTION__), used, aligned(8)))

#define PMA_BASE_ADDR               0x40006000
#define USB_LOCAL_ADDR(n)           (uint16_t)((uint32_t)(n) - PMA_BASE_ADDR)
#define PMA_ADDR_FROM_LOCAL(n)      (PMAWord_t*)((n) + PMA_BASE_ADDR)
#define APP_ADDR(n)                 (uint32_t*)((USB_LOCAL_ADDR(n) * 2) + PMA_BASE_ADDR)

#define CTRL_ENDPOINT_SIZE          64

#define USB_COUNT_RX_BLSIZE_Pos     (15U)
#define USB_COUNT_RX_BLSIZE_Msk     (0x1UL << USB_COUNT_RX_BLSIZE_Pos)     /*!< 0x80000000 */
#define USB_COUNT_RX_BLSIZE         USB_COUNT_RX_BLSIZE_Msk                /*!< BL_SIZE, bit 15 */
#define EPR_NON_TOGGLE_BITS         USB_EPREG_MASK

extern void turn_led_on(uint8_t on);

typedef uint16_t PMAWord_t;

typedef enum {
    ENDPOINT_BULK,
    ENDPOINT_CTRL,
    ENDPOINT_INTR
} USBEndpointType_t;

typedef enum {
    TRANS_NONE = 0,
    TRANS_NOZLP = 1 << 0
} USBTransferFlag_t;

typedef struct {
    void *addr;
    uint16_t len;
} USBTransferData_t;

typedef struct __attribute__((packed)) {
    PMAWord_t tx_addr;
    PMAWord_t tx_count;
    PMAWord_t rx_addr;
    PMAWord_t rx_count;
} USBBufferDescriptor_t;

typedef struct {
    union {
        uint16_t requestAndType;
        struct {
            uint8_t requestType;
            uint8_t request;
        };
    };
    uint16_t value;
    uint16_t index;
    uint16_t length;
} USBSetupPacket_t;

typedef struct {
    uint16_t size;
    USBTransferFlag_t flags;
    void *rx_buf;
    void *rx_pos;
    uint16_t rx_len;

    USBSetupPacket_t last_setup_pkt;
    // TODO: Complete this
} USBEndpointStatus_t;

typedef enum {
    USB_RX_WORKING = 0,
    USB_RX_DONE    = 1 << 0,
    USB_RX_SETUP   = 1 << 1
} USBRxStatus_t;

typedef enum {
    USB_CTRL_OK,
    USB_CTRL_STALL
} USBControlResult_t;

extern PMAWord_t _pma_end;

// Buffer descriptor table
static USBBufferDescriptor_t __PMA_BDT_ATTR__ bdt[8];

// Status of each endpoints
static USBEndpointStatus_t endpoint_status[8];

// Hold the next available address in the PMA
static PMAWord_t    *pma_ptr;

static uint8_t  buff_ep0[64];

static void usb_endpoint_setup(uint8_t endpoint,
                               uint16_t size,
                               USBEndpointType_t type,
                               USBTransferFlag_t flags) {

    // Sanity check
    if(endpoint > 7 || type > ENDPOINT_INTR) {
        return;
    }

    endpoint_status[endpoint].size = size;
    endpoint_status[endpoint].flags = flags;

    // TODO change this based on endpoint number
    USB->EP0R = USB_EP_CONTROL | (endpoint & 0xF);
}

static PMAWord_t* get_pma_buffer(uint16_t size) {
    PMAWord_t *buffer = pma_ptr;

    // The pma_ptr is PMAWord_t array.
    // Add extra space to avoid collision
    size = (size + 1)/sizeof(PMAWord_t);

    pma_ptr += size;
    return buffer;
}

static void set_usb_endpoint_status(uint8_t endpoint, uint16_t status, uint16_t mask) {
    // This is a bit tricky. because the bits we are going to set
    // is toggle. If we write 1, the value will toggle, and writing
    // 0 don't have any effect. So we need to write the data based
    // its current value.

    // Take the copy of current endpoint register
    // TODO: change this based on the endpoint number
    uint16_t val = USB->EP0R;

    // XOR the new status value with data read from end point register
    // As the result we will get val so that the corresponding bits that
    // needs to be toggled in endpoint register as 1 and other bits as 0.
    // Eg: if the EP0R register has 0x1000 and if we need to set 0x3000
    // after the following step we will get val = 0x2000. While writing
    // this data to EP0R, 13th bit will toggle and EP0R becomes 0x3000
    val ^= (status & mask);
    // TODO: change this based on the endpoint number
    USB->EP0R |= val;
}

static void usb_endpoint_begin_packet_rx(uint8_t endpoint) {
    uint16_t pkt_size = endpoint_status[endpoint].size;

    // Exit if reception has finished or not started yet
    if(endpoint_status[endpoint].rx_pos == 0 || pkt_size == 0) {
        return;
    }

    // Setup a space to recieve the incoming packets
    // Check if the PMA address is present; allocate
    // if it's not present
    if(bdt[endpoint].rx_addr == 0) {
        *APP_ADDR(&bdt[endpoint].rx_addr) = USB_LOCAL_ADDR(get_pma_buffer(pkt_size));

        // Table 77 in RM0008 explains how to define
        // Rx count in PMA
        uint16_t num_block;
        if(pkt_size > 62) {
            // BL_SIZE = 1 so that the counter value
            // become number of block * 64
            num_block = pkt_size / 64;
            *APP_ADDR(&bdt[endpoint].rx_count) = ((num_block << 10) | USB_COUNT_RX_BLSIZE);
            // bdt[endpoint].rx_count = ((num_block << 10) | USB_COUNT_RX_BLSIZE);
        } else {
            num_block = pkt_size / 2;
            *APP_ADDR(&bdt[endpoint].rx_count) = (num_block << 10);
            // bdt[endpoint].rx_count |= (num_block << 10);
        }
    }

    set_usb_endpoint_status(endpoint, USB_EP_RX_VALID, USB_EPRX_STAT);
}

static void usb_endpoint_receive(uint8_t endpoint, void *buff, uint16_t len) {
    if(buff) {
        endpoint_status[endpoint].rx_buf = buff;
        endpoint_status[endpoint].rx_pos = buff;
        endpoint_status[endpoint].rx_len = len;

        usb_endpoint_begin_packet_rx(endpoint);
    } else {
        endpoint_status[endpoint].rx_pos = 0;
        set_usb_endpoint_status(endpoint, USB_EP_RX_DIS, USB_EPRX_STAT);
    }
}

static void reset_endpoint_0() {
    // Setup endpoint 0 as a 64-bypte control endpoint.
    // Meaning, the maximum packet size that can be sent
    // or received in a single transfer is 64 byte.
    usb_endpoint_setup(0, CTRL_ENDPOINT_SIZE, ENDPOINT_CTRL, TRANS_NONE);

    // TODO: add comment why this
    usb_endpoint_receive(0, buff_ep0, sizeof(buff_ep0));
}

static void usb_reset(void) {
    // The packet memory area PMA of STM32F103C6T6
    // is located at the address 0x40006000 (Please
    // refer the table 3 in RM0008 document).
    //
    // The buffer descriptor table (bdt) located in PMA.
    // This table contain structure that describes
    // the buffer locations used for the 8 end points
    // in the PMA.

    USB->ISTR = 0;

    // The EBTABLE contain the address of buffer descriptor
    // table inside the PMA.
    USB->BTABLE = USB_LOCAL_ADDR(bdt);

    // Clear the buffer descriptor table and endpoint
    // status variables
    memset(bdt, 0, sizeof(bdt));
    memset(endpoint_status, 0, sizeof(endpoint_status));

    // Keep the the next location after the BDT
    pma_ptr = &_pma_end;

    // Setup the endpoint 0
    reset_endpoint_0();

    //enable correct transfer and reset interrupts
    USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SOFM
                              | USB_CNTR_ERRM | USB_CNTR_PMAOVRM;

    //Reset USB address to 0 with the device enabled
    USB->DADDR = USB_DADDR_EF;
}
static void copy_pma_to_sram(uint16_t *src, void *dst, uint16_t len) {
    PMAWord_t *dst_addr = (PMAWord_t*)dst;
    uint16_t *src_addr = (uint16_t*)APP_ADDR(src);

    for(uint16_t i = 0; i < len; i += sizeof(PMAWord_t)) {
        *dst_addr = *src_addr;
        dst_addr++;
        src_addr++;
    }
}

static USBRxStatus_t end_packet_rx(uint8_t endpoint) {
    uint16_t pkt_size = endpoint_status[endpoint].size;

    uint16_t rx_count = *APP_ADDR(&bdt[endpoint].rx_count) & 0x1FF;
    uint16_t rx_buff_addr = *APP_ADDR(&bdt[endpoint].rx_addr);
    uint16_t completed_len = endpoint_status[endpoint].rx_pos - endpoint_status[endpoint].rx_buf;

    if(USB->EP0R & USB_EP_SETUP) {
        // If the received packet is setup

        copy_pma_to_sram(PMA_ADDR_FROM_LOCAL(rx_buff_addr),
                         &endpoint_status[endpoint].last_setup_pkt, 8);

        // This is the only packet for the setup.
        // ie. reception is completed, we got what
        // we require.

        // turn_led_on(1);
        endpoint_status[endpoint].rx_len = completed_len;
        endpoint_status[endpoint].rx_pos = 0;

        return USB_RX_SETUP | USB_RX_DONE;
    }

    return 0;
}

#if 0
static USBControlResult_t endpoint_0_handle_setup_req(USBTransferData_t *nextData) {
    return USB_CTRL_STALL;
}

static void on_endpoint_0_setup_complete() {
    USBTransferData_t data = {0, 0};
    USBSetupPacket_t *setup_pkt __attribute__((unused)) = &endpoint_status[0].last_setup_pkt;

    if(USB_CTRL_STALL == endpoint_0_handle_setup_req(&data)) {
        goto usb_stall;
    }

    // cap the transfer length at the expected setup packet
    if(data.len > setup_pkt->length) {
        data.len = setup_pkt->length;
    }

usb_stall:
    // Stall USB here
}

static void on_endpoint_0_out_complete() {
}
#endif

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
        // An endpoint completed a valid transaction.

        // Find the endpoint ID and endpoint register value
        uint8_t endpoint = usb_status & USB_ISTR_EP_ID;
        uint16_t reg_val = USB->EP0R;

#if 1
        // uint8_t endpoint = 0;
        uart1_send_string("rx_pos = %x - rx_buf = %x\r\n",
                    endpoint_status[endpoint].rx_pos,
                    endpoint_status[endpoint].rx_buf);
#endif
        if(reg_val & USB_EP_CTR_RX) {
            // This is a receive transaction
            USBRxStatus_t ret = end_packet_rx(endpoint);
#if 0
            USB->EP0R = reg_val & EPR_NON_TOGGLE_BITS & ~USB_EP_CTR_RX;

            if(ret & USB_RX_SETUP) {
                on_endpoint_0_setup_complete();
            }

            if(ret & USB_RX_DONE) {
                on_endpoint_0_out_complete();
            }
#endif
        }
    }
}

void init_usb(void) {

    // Enable clock for the USB
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;

    // Clear the reset and power down bits
    USB->CNTR |= USB_CNTR_FRES;
    USB->CNTR = 0;
    // USB->CNTR &= ~USB_CNTR_FRES;

    // Clear interrupt flags if any
    USB->ISTR = 0;

    // Enable the USB interrupt
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    // Enable the USB reset interrupt
    USB->CNTR |= USB_CNTR_RESETM;
}
