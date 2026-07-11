#ifndef __FLASH_STORE_H
#define __FLASH_STORE_H

#include "main.h"
#include <stdint.h>

#define FLASH_STORE_MAX_MSGS 64U
#define FLASH_STORE_MSG_DATA_LEN 50U

typedef struct {
    uint8_t valid;
    uint8_t source_id;
    uint8_t length;
    char data[FLASH_STORE_MSG_DATA_LEN];
} FlashStore_MsgSlot;

void FlashStore_Init(void);
uint8_t FlashStore_IsReady(void);
uint8_t FlashStore_SaveMessage(const char *msg, uint8_t len);
uint8_t FlashStore_SaveMessageFrom(uint8_t source_id, const char *msg, uint8_t len);
void FlashStore_DeleteMessage(uint8_t index);
uint8_t FlashStore_GetCount(void);
uint8_t FlashStore_GetTotal(void);
const FlashStore_MsgSlot *FlashStore_GetMessage(uint8_t index);
void FlashStore_EraseAll(void);

#endif
