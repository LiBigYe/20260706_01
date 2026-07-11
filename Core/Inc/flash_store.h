/**
  ******************************************************************************
  * @file           : flash_store.h
  * @brief          : 内部Flash非易失消息存储 — 声语信使接收端
  *
  *   使用 STM32F411CEU6 内部 Flash Sector 3 (0x0800C000, 16KB) 存储最多
  *   5 条已接收短信息。断电后数据不丢失。
  *
  *   存储格式 (大端字节序, word 对齐写入):
  *     Word 0: Magic   = 0x564F4943 ("VOIC")
  *     Word 1: Version = 1 | (msg_count << 16)
  *     Word 2-14:  Message slot 0 (13 words = 52 bytes)
  *     Word 15-27: Message slot 1
  *     Word 28-40: Message slot 2
  *     Word 41-53: Message slot 3
  *     Word 54-66: Message slot 4
  *     总计: 67 words = 268 bytes, 远小于 16KB sector.
  *
  *   每条消息 slot (13 words):
  *     Word 0:  valid(8) | length(8) | data[0](8) | data[1](8)
  *     Word 1:  data[2..5]
  *     ...
  *     Word 12: data[46..49]
  *
  *   写入策略: 先擦除整个 sector, 再逐 word 写入全部 5 条消息.
  *   擦除时间: ~200-400ms (16KB sector).
  *
  *   注意: Flash 写入期间 CPU 会 stall (无法从 flash 取指执行).
  *   仅在接收完成后(RX_STATE_DONE)调用, 此时 DMA 已无活动数据.
  ******************************************************************************
  */

#ifndef __FLASH_STORE_H
#define __FLASH_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ========================================================================== */
/*  常量                                                                       */
/* ========================================================================== */

#define FLASH_STORE_SECTOR         FLASH_SECTOR_3
#define FLASH_STORE_ADDR           0x0800C000
#define FLASH_STORE_MAX_MSGS       5
#define FLASH_STORE_MSG_DATA_LEN   50      /* 最大 48 字符 + null + room */
#define FLASH_STORE_MAGIC          0x564F4943  /* "VOIC" */
#define FLASH_STORE_VERSION        2

/* 闪存写入电压范围 (STM32F411 Scale 1 → Voltage Range 3) */
#define FLASH_STORE_VOLTAGE_RANGE  FLASH_VOLTAGE_RANGE_3

/* ========================================================================== */
/*  RAM 中的消息镜像 (Flash 加载到 RAM 后再操作)                                */
/* ========================================================================== */

typedef struct {
    uint8_t valid;                          /* 1 = 有效消息 */
    uint8_t source_id;                      /* 来源终端 ID, 0=旧消息/未知 */
    uint8_t length;                         /* 消息长度 (0~48) */
    char    data[FLASH_STORE_MSG_DATA_LEN]; /* 消息内容 (null-terminated) */
} FlashStore_MsgSlot;

/* ========================================================================== */
/*  API                                                                        */
/* ========================================================================== */

/**
  * @brief  初始化存储系统
  *
  *   从 Flash Sector 3 读取已保存的消息到 RAM 镜像.
  *   若 magic 不匹配 (首次上电/擦除后), 初始化为空状态.
  *   应在 main() 早期调用, 在 RX_Init 之前.
  */
void FlashStore_Init(void);

/**
  * @brief  保存一条消息到 Flash (同时更新 RAM 镜像)
  * @param  msg:  消息字符串指针
  * @param  len:  消息长度 (0~48)
  * @retval 0~4 = 槽位索引, 0xFF = 存储已满或失败
  *
  *   若已有 5 条消息, 自动淘汰最旧的一条 (循环缓冲).
  *   擦除 sector → 写入全部 5 条 → 返回槽位索引.
  *   阻塞约 200-400ms (16KB sector erase).
  */
uint8_t FlashStore_SaveMessage(const char *msg, uint8_t len);
uint8_t FlashStore_SaveMessageFrom(uint8_t source_id, const char *msg, uint8_t len);

/**
  * @brief  删除指定槽位的消息
  * @param  index: 槽位索引 (0~4)
  *
  *   从 RAM 移除 → 擦除 sector → 重写剩余消息.
  */
void FlashStore_DeleteMessage(uint8_t index);

/**
  * @brief  获取已存储消息数量
  * @retval 0~5
  */
uint8_t FlashStore_GetCount(void);

/**
  * @brief  获取槽位总数 (含空闲槽位)
  * @retval 固定返回 5
  */
uint8_t FlashStore_GetTotal(void);

/**
  * @brief  获取指定索引的消息
  * @param  index: 0 ~ (count-1), 按存储时间排序
  * @retval 指向消息 slot 的指针, 或 NULL (索引无效)
  *
  *   返回的 slot 包含 valid, length, data 字段.
  *   调用者不应释放或修改返回的指针.
  */
const FlashStore_MsgSlot* FlashStore_GetMessage(uint8_t index);

/**
  * @brief  擦除所有存储的消息
  *
  *   擦除 sector → 重置 RAM 为空.
  */
void FlashStore_EraseAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_STORE_H */
