#include "py25q64.h"

#define CMD_READ_DATA 0x03U
#define CMD_PAGE_PROGRAM 0x02U
#define CMD_WRITE_ENABLE 0x06U
#define CMD_READ_STATUS 0x05U
#define CMD_SECTOR_ERASE 0x20U
#define CMD_CHIP_ERASE 0xC7U
#define CMD_READ_JEDEC_ID 0x9FU
#define CMD_RELEASE_POWERDOWN 0xABU
#define STATUS_BUSY 0x01U
#define SPI_TIMEOUT_MS 100U
#define PROGRAM_TIMEOUT_MS 10U
#define ERASE_TIMEOUT_MS 1000U
#define CHIP_ERASE_TIMEOUT_MS 120000U

static SPI_HandleTypeDef *flash_spi;

static void cs_low(void) { HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_SET); }

static HAL_StatusTypeDef send_command(uint8_t value)
{
    cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(flash_spi, &value, 1U, SPI_TIMEOUT_MS);
    cs_high();
    return status;
}

static HAL_StatusTypeDef wait_ready(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t command = CMD_READ_STATUS;
    uint8_t status_byte = STATUS_BUSY;
    do {
        cs_low();
        HAL_StatusTypeDef status = HAL_SPI_Transmit(flash_spi, &command, 1U, SPI_TIMEOUT_MS);
        if (status == HAL_OK) status = HAL_SPI_Receive(flash_spi, &status_byte, 1U, SPI_TIMEOUT_MS);
        cs_high();
        if (status != HAL_OK) return status;
        if ((status_byte & STATUS_BUSY) == 0U) return HAL_OK;
    } while ((HAL_GetTick() - start) < timeout_ms);
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef write_enable(void)
{
    HAL_StatusTypeDef status = wait_ready(ERASE_TIMEOUT_MS);
    return status == HAL_OK ? send_command(CMD_WRITE_ENABLE) : status;
}

HAL_StatusTypeDef PY25Q64_Init(SPI_HandleTypeDef *spi)
{
    flash_spi = spi;
    cs_high();
    HAL_Delay(1U);
    if (send_command(CMD_RELEASE_POWERDOWN) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1U);
    return PY25Q64_ReadJedecId() == PY25Q64_JEDEC_ID ? HAL_OK : HAL_ERROR;
}

uint32_t PY25Q64_ReadJedecId(void)
{
    uint8_t command = CMD_READ_JEDEC_ID;
    uint8_t id[3] = {0U};
    if (flash_spi == NULL) return 0U;
    cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(flash_spi, &command, 1U, SPI_TIMEOUT_MS);
    if (status == HAL_OK) status = HAL_SPI_Receive(flash_spi, id, sizeof(id), SPI_TIMEOUT_MS);
    cs_high();
    if (status != HAL_OK) return 0U;
    return ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
}

HAL_StatusTypeDef PY25Q64_Read(uint32_t address, void *data, uint32_t length)
{
    if (flash_spi == NULL || data == NULL || address >= PY25Q64_SIZE_BYTES || length > PY25Q64_SIZE_BYTES - address) return HAL_ERROR;
    if (length == 0U) return HAL_OK;
    uint8_t header[4] = {CMD_READ_DATA, (uint8_t)(address >> 16), (uint8_t)(address >> 8), (uint8_t)address};
    cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(flash_spi, header, sizeof(header), SPI_TIMEOUT_MS);
    if (status == HAL_OK) status = HAL_SPI_Receive(flash_spi, data, (uint16_t)length, SPI_TIMEOUT_MS);
    cs_high();
    return status;
}

HAL_StatusTypeDef PY25Q64_Write(uint32_t address, const void *data, uint32_t length)
{
    if (flash_spi == NULL || data == NULL || address >= PY25Q64_SIZE_BYTES || length > PY25Q64_SIZE_BYTES - address) return HAL_ERROR;
    const uint8_t *source = data;
    while (length > 0U) {
        uint32_t remaining = PY25Q64_PAGE_SIZE - (address & (PY25Q64_PAGE_SIZE - 1U));
        uint16_t chunk = (uint16_t)(length < remaining ? length : remaining);
        HAL_StatusTypeDef status = write_enable();
        if (status != HAL_OK) return status;
        uint8_t header[4] = {CMD_PAGE_PROGRAM, (uint8_t)(address >> 16), (uint8_t)(address >> 8), (uint8_t)address};
        cs_low();
        status = HAL_SPI_Transmit(flash_spi, header, sizeof(header), SPI_TIMEOUT_MS);
        if (status == HAL_OK) status = HAL_SPI_Transmit(flash_spi, (uint8_t *)source, chunk, SPI_TIMEOUT_MS);
        cs_high();
        if (status != HAL_OK) return status;
        status = wait_ready(PROGRAM_TIMEOUT_MS);
        if (status != HAL_OK) return status;
        address += chunk;
        source += chunk;
        length -= chunk;
    }
    return HAL_OK;
}

HAL_StatusTypeDef PY25Q64_EraseSector(uint32_t address)
{
    if (address >= PY25Q64_SIZE_BYTES) return HAL_ERROR;
    address &= ~(PY25Q64_SECTOR_SIZE - 1U);
    HAL_StatusTypeDef status = write_enable();
    if (status != HAL_OK) return status;
    uint8_t packet[4] = {CMD_SECTOR_ERASE, (uint8_t)(address >> 16), (uint8_t)(address >> 8), (uint8_t)address};
    cs_low();
    status = HAL_SPI_Transmit(flash_spi, packet, sizeof(packet), SPI_TIMEOUT_MS);
    cs_high();
    return status == HAL_OK ? wait_ready(ERASE_TIMEOUT_MS) : status;
}

HAL_StatusTypeDef PY25Q64_EraseChip(void)
{
    HAL_StatusTypeDef status = write_enable();
    if (status != HAL_OK) return status;
    status = send_command(CMD_CHIP_ERASE);
    return status == HAL_OK ? wait_ready(CHIP_ERASE_TIMEOUT_MS) : status;
}
