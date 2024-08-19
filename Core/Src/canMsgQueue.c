#include <string.h>
#include "canMsgQueue.h"

#define MSG_LIST_SIZE   10

static CAN_Message msgList[MSG_LIST_SIZE];
static uint8_t write_idx = 0;
static uint8_t read_idx = 0;

int8_t CMQ_AddMsg(CAN_Message *msg) {
    if(read_idx == write_idx) {
        return -1;
    }

    memcpy(&msgList[write_idx], msg, sizeof(CAN_Message));
    write_idx++;
    if(write_idx == MSG_LIST_SIZE) {
        write_idx = 0;
    }
    return 0;
}

CAN_Message* CMQ_GetMsg() {
    CAN_Message *msg;

    if(read_idx == write_idx) {
        return NULL;
    }

    msg = &msgList[read_idx];
    read_idx++;
    if(read_idx == MSG_LIST_SIZE) {
        read_idx = 0;
    }

    return msg;
}
