#include "w25qxx.h"

static SPI_HandleTypeDef *spi; // Pointer to SPI handle used for flash

// Pull CS pin low (select chip)
static void CS_LOW()  { HAL_GPIO_WritePin(W25QXX_CS_GPIO_Port, W25QXX_CS_Pin, GPIO_PIN_RESET); }
// Pull CS pin high (deselect chip)
static void CS_HIGH() { HAL_GPIO_WritePin(W25QXX_CS_GPIO_Port, W25QXX_CS_Pin, GPIO_PIN_SET); }

// Initialize W25QXX with SPI handle
void W25QXX_Init(SPI_HandleTypeDef *hspi_instance) {
    spi = hspi_instance; // Store SPI handle
    CS_HIGH();           // Keep chip deselected initially
}

// Send Write Enable command
static void WriteEnable() {
    uint8_t cmd = W25Q_WRITE_ENABLE;
    CS_LOW();
    HAL_SPI_Transmit(spi, &cmd, 1, HAL_MAX_DELAY);
    CS_HIGH();
}

// Wait until flash is ready (WIP bit = 0)
static void WaitUntilReady() {
    uint8_t cmd = W25Q_READ_STATUS_REG1;
    uint8_t status;
    do {
        CS_LOW();
        HAL_SPI_Transmit(spi, &cmd, 1, HAL_MAX_DELAY);
        HAL_SPI_Receive(spi, &status, 1, HAL_MAX_DELAY);
        CS_HIGH();
    } while (status & 0x01); // Wait until Write In Progress bit clears
}

// Write data to flash (Page Program)
void W25QXX_Write(uint32_t addr, uint8_t *data, uint16_t len) {
    WriteEnable(); // Enable writing
    uint8_t cmd[4] = {
        W25Q_PAGE_PROGRAM,
        (addr >> 16) & 0xFF,
        (addr >> 8) & 0xFF,
        addr & 0xFF
    };
    CS_LOW();
    HAL_SPI_Transmit(spi, cmd, 4, HAL_MAX_DELAY);  // Send write command + address
    HAL_SPI_Transmit(spi, data, len, HAL_MAX_DELAY); // Send data
    CS_HIGH();
    WaitUntilReady(); // Wait until write finishes
}

// Read data from flash
void W25QXX_Read(uint32_t addr, uint8_t *data, uint16_t len) {
    uint8_t cmd[4] = {
        W25Q_READ_DATA,
        (addr >> 16) & 0xFF,
        (addr >> 8) & 0xFF,
        addr & 0xFF
    };
    CS_LOW();
    HAL_SPI_Transmit(spi, cmd, 4, HAL_MAX_DELAY);   // Send read command + address
    HAL_SPI_Receive(spi, data, len, HAL_MAX_DELAY); // Receive data
    CS_HIGH();
}

// Erase a 4KB sector at given address
void W25QXX_SectorErase(uint32_t address)
{
    uint8_t cmd[4] = {
        CMD_SECTOR_ERASE,
        (address >> 16) & 0xFF,
        (address >> 8) & 0xFF,
        address & 0xFF
    };
    WriteEnable(); // Enable 
    CS_LOW();
    HAL_SPI_Transmit(spi, cmd, 4, HAL_MAX_DELAY);
    CS_HIGH();
    WaitUntilReady(); // Wait until erase completes
}

// Erase entire chip
void W25QXX_ChipErase(void)
{
    uint8_t cmd = CMD_CHIP_ERASE;
    WriteEnable();
    CS_LOW();
    HAL_SPI_Transmit(spi, &cmd, 1, HAL_MAX_DELAY);
    CS_HIGH();
    WaitUntilReady(); // Wait until erase completes (can take seconds)
}
