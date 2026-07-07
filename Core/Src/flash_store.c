/**
  ******************************************************************************
  * @file           : flash_store.c
  * @brief          : 内部Flash非易失消息存储实现 — 声语信使接收端
  *
  *   使用 STM32F411CEU6 内部 Flash Sector 3 (0x0800C000, 16KB) 存储.
  *
  *   消息槽位在 RAM 中维护 (按接收时间从旧到新排列),
  *   每次保存/删除时擦除 sector 并全量重写.
  *
  *   Flash 写入操作会短暂 stall CPU (16KB sector erase ~200-400ms).
  *   仅在接收完成 DONE 状态时调用, 此时无 DMA 活动.
  ******************************************************************************
  */

#include "flash_store.h"
#include <string.h>

/* ========================================================================== */
/*  Flash 布局常量                                                              */
/* ========================================================================== */

/* 每条消息在 flash 中占用的 word 数 */
#define FLASH_MSG_WORDS         13   /* 1 header + 12 data = 52 bytes */
/* 全部数据占用的总 word 数 */
#define FLASH_TOTAL_WORDS       (2 + FLASH_STORE_MAX_MSGS * FLASH_MSG_WORDS)
/* 每字 4 字节 */
#define FLASH_TOTAL_BYTES       (FLASH_TOTAL_WORDS * 4)

/* ========================================================================== */
/*  RAM 消息镜像                                                                */
/* ========================================================================== */

static FlashStore_MsgSlot msg_ram[FLASH_STORE_MAX_MSGS];
static uint8_t            msg_count;   /* 当前有效消息数 (0~5) */

/* ========================================================================== */
/*  内部辅助 — 闪存数据打包/解包                                                 */
/* ========================================================================== */

/**
  * @brief  将一条消息打包写入 flash
  * @param  base_addr: 此消息槽位的起始地址 (word-aligned)
  * @param  slot:      消息槽位指针
  * @retval HAL_OK / HAL_ERROR
  *
  *   写入 13 个 word:
  *     Word 0:  valid(31:24) | length(23:16) | data[0](15:8) | data[1](7:0)
  *     Word 1:  data[2..5]
  *     ...
  *     Word 12: data[46..49]
  *
  *   空槽位: valid=0, 其余为 0xFF (配合 flash 擦除后全 1 特性).
  */
static HAL_StatusTypeDef flash_write_slot(uint32_t addr, const FlashStore_MsgSlot *slot)
{
    HAL_StatusTypeDef status;

    if (!slot->valid) {
        /* 空槽位: 写入 13 个 0xFFFFFFFF (与擦除后状态一致, 可选跳过) */
        /* 实际不写, 因为 sector 刚擦除过, 内容就是 0xFFFFFFFF */
        return HAL_OK;
    }

    /* Word 0: valid | length | data[0] | data[1] */
    {
        uint32_t w0 = ((uint32_t)slot->valid  << 24)
                    | ((uint32_t)slot->length << 16)
                    | ((uint32_t)(uint8_t)slot->data[0] << 8)
                    | ((uint32_t)(uint8_t)slot->data[1]);
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, w0);
        if (status != HAL_OK) return status;
    }

    /* Words 1..11: 每字打包 4 字节数据 */
    for (uint8_t wi = 1; wi <= 11; wi++) {
        uint32_t w = 0;
        uint8_t  base = 2 + (wi - 1) * 4;
        for (uint8_t bi = 0; bi < 4; bi++) {
            uint8_t byte_val;
            if (base + bi < slot->length) {
                byte_val = (uint8_t)slot->data[base + bi];
            } else if (base + bi == slot->length) {
                byte_val = 0;  /* null terminator */
            } else {
                byte_val = 0xFF; /* padding */
            }
            w |= ((uint32_t)byte_val << (bi * 8));
        }
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                   addr + wi * 4, w);
        if (status != HAL_OK) return status;
    }

    /* Word 12: data[46..49] (最后 4 字节) */
    {
        uint32_t w12 = 0;
        for (uint8_t bi = 0; bi < 4; bi++) {
            uint8_t idx = 46 + bi;
            uint8_t byte_val;
            if (idx < slot->length) {
                byte_val = (uint8_t)slot->data[idx];
            } else if (idx == slot->length) {
                byte_val = 0;  /* null terminator */
            } else {
                byte_val = 0xFF;
            }
            w12 |= ((uint32_t)byte_val << (bi * 8));
        }
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                   addr + 12 * 4, w12);
        if (status != HAL_OK) return status;
    }

    return HAL_OK;
}

/**
  * @brief  从 flash 读取一条消息到 RAM
  * @param  base_addr: 此消息槽位的起始地址 (word-aligned)
  * @param  slot:      输出消息槽位指针
  */
static void flash_read_slot(uint32_t addr, FlashStore_MsgSlot *slot)
{
    /* Word 0: valid | length | data[0] | data[1] */
    uint32_t w0 = *(__IO uint32_t *)addr;
    slot->valid  = (uint8_t)((w0 >> 24) & 0xFF);
    slot->length = (uint8_t)((w0 >> 16) & 0xFF);
    slot->data[0] = (char)((w0 >> 8) & 0xFF);
    slot->data[1] = (char)(w0 & 0xFF);

    /* Words 1..11 */
    for (uint8_t wi = 1; wi <= 11; wi++) {
        uint32_t w = *(__IO uint32_t *)(addr + wi * 4);
        uint8_t  base = 2 + (wi - 1) * 4;
        for (uint8_t bi = 0; bi < 4; bi++) {
            slot->data[base + bi] = (char)((w >> (bi * 8)) & 0xFF);
        }
    }

    /* Word 12 */
    uint32_t w12 = *(__IO uint32_t *)(addr + 12 * 4);
    for (uint8_t bi = 0; bi < 4; bi++) {
        slot->data[46 + bi] = (char)((w12 >> (bi * 8)) & 0xFF);
    }

    /* 安全检查: 截断长度, 添加 null 终止符 */
    if (slot->length > 48) slot->length = 48;
    slot->data[slot->length] = '\0';

    /* 有效性检查: valid 必须为 1, 且 length > 0 或空消息也接受 */
    if (slot->valid != 1) {
        slot->valid = 0;
        memset(slot->data, 0, sizeof(slot->data));
        slot->length = 0;
    }
}

/* ========================================================================== */
/*  内部辅助 — 全量写入                                                          */
/* ========================================================================== */

/**
  * @brief  擦除 sector 并写入全部消息
  * @retval HAL_OK / HAL_ERROR / HAL_TIMEOUT
  *
  *   操作序列:
  *     1. 解锁 Flash
  *     2. 擦除 Sector 3 (200-400ms)
  *     3. 写入 header (2 words: magic + version|count)
  *     4. 逐槽位写入 5 条消息 (每槽 13 words)
  *     5. 锁定 Flash
  *
  *   注意: 擦除/写入期间 CPU 会 stall (无法从 flash 取指).
  */
static HAL_StatusTypeDef flash_write_all(void)
{
    HAL_StatusTypeDef status;

    /* 1. 解锁 Flash */
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return HAL_ERROR;
    }

    /* 2. 擦除 Sector 3 */
    FLASH_EraseInitTypeDef erase_cfg = {0};
    uint32_t sector_error = 0;

    erase_cfg.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase_cfg.Sector       = FLASH_STORE_SECTOR;
    erase_cfg.NbSectors    = 1;
    erase_cfg.VoltageRange = FLASH_STORE_VOLTAGE_RANGE;

    status = HAL_FLASHEx_Erase(&erase_cfg, &sector_error);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    /* 3. 写入 Word 0: Magic */
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                               FLASH_STORE_ADDR,
                               FLASH_STORE_MAGIC);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    /* 4. 写入 Word 1: Version | msg_count */
    {
        uint32_t w1 = ((uint32_t)FLASH_STORE_VERSION)
                    | ((uint32_t)msg_count << 16);
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                   FLASH_STORE_ADDR + 4, w1);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
    }

    /* 5. 逐槽位写入消息 */
    for (uint8_t i = 0; i < FLASH_STORE_MAX_MSGS; i++) {
        uint32_t slot_addr = FLASH_STORE_ADDR + 8 + (uint32_t)i * FLASH_MSG_WORDS * 4;
        status = flash_write_slot(slot_addr, &msg_ram[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
    }

    /* 6. 锁定 Flash */
    HAL_FLASH_Lock();

    return HAL_OK;
}

/* ========================================================================== */
/*  公开 API                                                                   */
/* ========================================================================== */

/**
  * @brief  初始化存储系统: 从 Flash 加载已有消息到 RAM
  */
void FlashStore_Init(void)
{
    /* 初始化 RAM 为空 */
    memset(msg_ram, 0, sizeof(msg_ram));
    msg_count = 0;

    /* 读取 Word 0: Magic 验证 */
    uint32_t magic = *(__IO uint32_t *)FLASH_STORE_ADDR;
    if (magic != FLASH_STORE_MAGIC) {
        /* 首次上电或 sector 被擦除, 保持空状态 */
        return;
    }

    /* 读取 Word 1: Version | msg_count */
    uint32_t w1 = *(__IO uint32_t *)(FLASH_STORE_ADDR + 4);
    uint32_t version = w1 & 0xFFFF;
    uint32_t flash_count = (w1 >> 16) & 0xFF;

    if (version != FLASH_STORE_VERSION) {
        /* 版本不兼容, 重置为空 */
        return;
    }

    if (flash_count > FLASH_STORE_MAX_MSGS) {
        flash_count = FLASH_STORE_MAX_MSGS;
    }

    /* 逐槽位读取消息 */
    for (uint8_t i = 0; i < FLASH_STORE_MAX_MSGS; i++) {
        uint32_t slot_addr = FLASH_STORE_ADDR + 8 + (uint32_t)i * FLASH_MSG_WORDS * 4;
        flash_read_slot(slot_addr, &msg_ram[i]);
        if (msg_ram[i].valid) {
            if (i >= msg_count) {
                msg_count = i + 1;
            }
        }
    }

    /* 一致性校验: 若 flash_count 与有效槽位数不符, 以实际槽位为准 */
    uint8_t actual_count = 0;
    for (uint8_t i = 0; i < FLASH_STORE_MAX_MSGS; i++) {
        if (msg_ram[i].valid) actual_count++;
    }
    if (flash_count != actual_count) {
        /* 数据不一致 (可能是写入中断), 以实际槽位为准.
           确保没有空隙 (compact) */
        uint8_t write_idx = 0;
        for (uint8_t read_idx = 0; read_idx < FLASH_STORE_MAX_MSGS; read_idx++) {
            if (msg_ram[read_idx].valid) {
                if (write_idx != read_idx) {
                    memcpy(&msg_ram[write_idx], &msg_ram[read_idx],
                           sizeof(FlashStore_MsgSlot));
                    memset(&msg_ram[read_idx], 0, sizeof(FlashStore_MsgSlot));
                }
                write_idx++;
            }
        }
        msg_count = write_idx;
    }
}

/**
  * @brief  保存一条消息到存储
  *
  *   循环缓冲: 满时淘汰最旧消息 (slot 0), 其余前移.
  */
uint8_t FlashStore_SaveMessage(const char *msg, uint8_t len)
{
    if (len > 48) len = 48;

    /* 如果已满 (count == 5), 淘汰最旧 (slot 0), 其余左移 */
    if (msg_count >= FLASH_STORE_MAX_MSGS) {
        /* 左移 4 条消息 */
        for (uint8_t i = 0; i < FLASH_STORE_MAX_MSGS - 1; i++) {
            memcpy(&msg_ram[i], &msg_ram[i + 1], sizeof(FlashStore_MsgSlot));
        }
        /* 新消息写入最后一个槽位 */
        msg_ram[FLASH_STORE_MAX_MSGS - 1].valid  = 1;
        msg_ram[FLASH_STORE_MAX_MSGS - 1].length = len;
        memcpy(msg_ram[FLASH_STORE_MAX_MSGS - 1].data, msg, len);
        msg_ram[FLASH_STORE_MAX_MSGS - 1].data[len] = '\0';
    } else {
        /* 未满: 直接追加到 count 位置 */
        uint8_t idx = msg_count;
        msg_ram[idx].valid  = 1;
        msg_ram[idx].length = len;
        memcpy(msg_ram[idx].data, msg, len);
        msg_ram[idx].data[len] = '\0';
        msg_count++;
    }

    /* 写入 Flash */
    if (flash_write_all() != HAL_OK) {
        /* 写入失败: 回退 RAM 状态 */
        /* (简单处理: 保留 RAM 状态, 下次上电重新从 flash 加载) */
        return 0xFF;
    }

    return (msg_count >= FLASH_STORE_MAX_MSGS)
           ? (FLASH_STORE_MAX_MSGS - 1)
           : (msg_count - 1);
}

/**
  * @brief  删除指定索引的消息
  */
void FlashStore_DeleteMessage(uint8_t index)
{
    if (index >= msg_count) return;
    if (msg_count == 0) return;

    /* 左移后续消息 */
    for (uint8_t i = index; i < msg_count - 1; i++) {
        memcpy(&msg_ram[i], &msg_ram[i + 1], sizeof(FlashStore_MsgSlot));
    }
    /* 清除最后一个槽位 */
    memset(&msg_ram[msg_count - 1], 0, sizeof(FlashStore_MsgSlot));
    msg_count--;

    /* 写入 Flash */
    flash_write_all();
}

/**
  * @brief  获取已存储消息数量
  */
uint8_t FlashStore_GetCount(void)
{
    return msg_count;
}

/**
  * @brief  获取槽位总数
  */
uint8_t FlashStore_GetTotal(void)
{
    return FLASH_STORE_MAX_MSGS;
}

/**
  * @brief  获取指定索引的消息
  */
const FlashStore_MsgSlot* FlashStore_GetMessage(uint8_t index)
{
    if (index >= msg_count) return NULL;
    if (!msg_ram[index].valid) return NULL;
    return &msg_ram[index];
}

/**
  * @brief  擦除所有存储的消息
  */
void FlashStore_EraseAll(void)
{
    memset(msg_ram, 0, sizeof(msg_ram));
    msg_count = 0;
    flash_write_all();
}
