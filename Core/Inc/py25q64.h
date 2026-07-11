#ifndef PY25Q64_H
#define PY25Q64_H

#include "main.h"
#include <stdint.h>

#define PY25Q64_SIZE_BYTES 0x800000U
#define PY25Q64_PAGE_SIZE 256U
#define PY25Q64_SECTOR_SIZE 4096U
#define PY25Q64_JEDEC_ID 0x852017U

HAL_StatusTypeDef PY25Q64_Init(SPI_HandleTypeDef *spi);
uint32_t PY25Q64_ReadJedecId(void);
HAL_StatusTypeDef PY25Q64_Read(uint32_t address, void *data, uint32_t length);
HAL_StatusTypeDef PY25Q64_Write(uint32_t address, const void *data, uint32_t length);
HAL_StatusTypeDef PY25Q64_EraseSector(uint32_t address);
HAL_StatusTypeDef PY25Q64_EraseChip(void);

#endif
