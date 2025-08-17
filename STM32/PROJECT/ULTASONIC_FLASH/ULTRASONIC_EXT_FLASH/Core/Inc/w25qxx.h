#ifndef __W25QXX_H__   // Prevent multiple inclusion
#define __W25QXX_H__

#include "stm32l4xx_hal.h"  // HAL library for STM32L4 series

// Chip Select (CS) pin configuration for W25QXX
#define W25QXX_CS_GPIO_Port GPIOA
#define W25QXX_CS_Pin GPIO_PIN_4

// W25QXX command codes
#define W25Q_WRITE_ENABLE         0x06  // Enable write operations
#define W25Q_READ_STATUS_REG1     0x05  // Read status register 1
#define W25Q_PAGE_PROGRAM         0x02  // Write data to a page
#define W25Q_READ_DATA            0x03  // Read data from flash
#define CMD_SECTOR_ERASE          0x20  // Erase 4KB sector
#define CMD_CHIP_ERASE            0xC7  // Erase entire chip

// Function prototypes
void W25QXX_Init(SPI_HandleTypeDef *hspi);               // Initialize flash with SPI handle
void W25QXX_Write(uint32_t addr, uint8_t *data, uint16_t len);  // Write data to flash
void W25QXX_Read(uint32_t addr, uint8_t *data, uint16_t len);   // Read data from flash
void W25QXX_SectorErase(uint32_t address);                // Erase a 4KB sector

#endif
