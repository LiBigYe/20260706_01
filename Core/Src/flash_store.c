#include "flash_store.h"
#include "py25q64.h"
#include <stddef.h>
#include <string.h>

extern SPI_HandleTypeDef hspi1;

#define STORE_MAGIC 0x564F4943U
#define STORE_VERSION 3U
#define STORE_COPY_A 0x000000U
#define STORE_COPY_B 0x001000U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    uint32_t generation;
    uint32_t crc32;
    FlashStore_MsgSlot slots[FLASH_STORE_MAX_MSGS];
} StoreImage;

static StoreImage image;
static uint32_t active_address = STORE_COPY_A;
static uint8_t store_ready;

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t length)
{
    while (length-- > 0U) {
        crc ^= *data++;
        for (uint8_t bit = 0U; bit < 8U; bit++) crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
    return crc;
}

static uint32_t image_crc(const StoreImage *candidate)
{
    const uint8_t *start = (const uint8_t *)&candidate->slots[0];
    uint32_t crc = 0xFFFFFFFFU;
    crc = crc32_update(crc, (const uint8_t *)&candidate->version, sizeof(candidate->version));
    crc = crc32_update(crc, (const uint8_t *)&candidate->count, sizeof(candidate->count));
    crc = crc32_update(crc, (const uint8_t *)&candidate->generation, sizeof(candidate->generation));
    crc = crc32_update(crc, start, sizeof(candidate->slots));
    return ~crc;
}

static uint8_t image_valid(const StoreImage *candidate)
{
    return candidate->magic == STORE_MAGIC && candidate->version == STORE_VERSION &&
           candidate->count <= FLASH_STORE_MAX_MSGS && candidate->crc32 == image_crc(candidate);
}

static uint8_t generation_newer(uint32_t left, uint32_t right)
{
    return (int32_t)(left - right) > 0;
}

static HAL_StatusTypeDef commit(void)
{
    uint32_t target = active_address == STORE_COPY_A ? STORE_COPY_B : STORE_COPY_A;
    image.magic = STORE_MAGIC;
    image.version = STORE_VERSION;
    image.generation++;
    image.crc32 = image_crc(&image);
    HAL_StatusTypeDef status = PY25Q64_EraseSector(target);
    if (status == HAL_OK) status = PY25Q64_Write(target, &image, sizeof(image));
    if (status != HAL_OK) return status;
    StoreImage verify;
    status = PY25Q64_Read(target, &verify, sizeof(verify));
    if (status != HAL_OK || !image_valid(&verify) || verify.generation != image.generation) return HAL_ERROR;
    active_address = target;
    return HAL_OK;
}

void FlashStore_Init(void)
{
    memset(&image, 0, sizeof(image));
    store_ready = PY25Q64_Init(&hspi1) == HAL_OK;
    if (!store_ready) return;
    StoreImage copy_a;
    StoreImage copy_b;
    uint8_t valid_a = PY25Q64_Read(STORE_COPY_A, &copy_a, sizeof(copy_a)) == HAL_OK && image_valid(&copy_a);
    uint8_t valid_b = PY25Q64_Read(STORE_COPY_B, &copy_b, sizeof(copy_b)) == HAL_OK && image_valid(&copy_b);
    if (valid_a && (!valid_b || generation_newer(copy_a.generation, copy_b.generation))) {
        image = copy_a;
        active_address = STORE_COPY_A;
    } else if (valid_b) {
        image = copy_b;
        active_address = STORE_COPY_B;
    } else {
        active_address = STORE_COPY_B;
        image.magic = STORE_MAGIC;
        image.version = STORE_VERSION;
        image.generation = 0U;
        image.count = 0U;
        if (commit() != HAL_OK) store_ready = 0U;
    }
}

uint8_t FlashStore_IsReady(void) { return store_ready; }

uint8_t FlashStore_SaveMessage(const char *msg, uint8_t len)
{
    return FlashStore_SaveMessageFrom(0U, msg, len);
}

uint8_t FlashStore_SaveMessageFrom(uint8_t source_id, const char *msg, uint8_t len)
{
    if (!store_ready || msg == NULL || len == 0U) return 0xFFU;
    if (len >= FLASH_STORE_MSG_DATA_LEN) len = FLASH_STORE_MSG_DATA_LEN - 1U;
    StoreImage backup = image;
    uint8_t index;
    if (image.count < FLASH_STORE_MAX_MSGS) {
        index = (uint8_t)image.count++;
    } else {
        memmove(&image.slots[0], &image.slots[1], sizeof(image.slots[0]) * (FLASH_STORE_MAX_MSGS - 1U));
        index = FLASH_STORE_MAX_MSGS - 1U;
    }
    FlashStore_MsgSlot *slot = &image.slots[index];
    memset(slot, 0, sizeof(*slot));
    slot->valid = 1U;
    slot->source_id = source_id;
    slot->length = len;
    memcpy(slot->data, msg, len);
    slot->data[len] = '\0';
    if (commit() != HAL_OK) {
        image = backup;
        return 0xFFU;
    }
    return index;
}

void FlashStore_DeleteMessage(uint8_t index)
{
    if (!store_ready || index >= image.count) return;
    StoreImage backup = image;
    if (index + 1U < image.count) memmove(&image.slots[index], &image.slots[index + 1U], sizeof(image.slots[0]) * (image.count - index - 1U));
    image.count--;
    memset(&image.slots[image.count], 0, sizeof(image.slots[0]));
    if (commit() != HAL_OK) image = backup;
}

uint8_t FlashStore_GetCount(void) { return (uint8_t)image.count; }
uint8_t FlashStore_GetTotal(void) { return FLASH_STORE_MAX_MSGS; }

const FlashStore_MsgSlot *FlashStore_GetMessage(uint8_t index)
{
    if (!store_ready || index >= image.count || !image.slots[index].valid) return NULL;
    return &image.slots[index];
}

void FlashStore_EraseAll(void)
{
    if (!store_ready) return;
    StoreImage backup = image;
    memset(image.slots, 0, sizeof(image.slots));
    image.count = 0U;
    if (commit() != HAL_OK) image = backup;
}
