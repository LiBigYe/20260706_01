#ifndef __EDITOR_H
#define __EDITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define EDITOR_MAX_CHARS    48
#define T9_TIMEOUT_MS       2000
#define CURSOR_BLINK_MS     500

#define MODE_NUMBER  0
#define MODE_LOWER   1
#define MODE_UPPER   2
#define MODE_COUNT   3

void      Editor_Init(void);
void      Editor_HandleKey(uint8_t key);
void      Editor_Tick(void);
void      Editor_UpdateDisplay(void);
const char* Editor_GetBuffer(void);
uint8_t     Editor_GetLength(void);
uint8_t     Editor_IsSendRequested(void);
void        Editor_ClearSendRequest(void);
void        Editor_SetTxStatus(const char *status);

#ifdef __cplusplus
}
#endif

#endif /* __EDITOR_H */
