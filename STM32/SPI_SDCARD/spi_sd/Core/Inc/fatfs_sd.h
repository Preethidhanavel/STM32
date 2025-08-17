
#ifndef __FATFS_SD_H__
#define __FATFS_SD_H__

#include "main.h"
#include "stm32l4xx_hal.h"
#include "fatfs.h"

// ------------------- User Configuration -------------------
#define SD_CS_GPIO_Port GPIOC
#define SD_CS_Pin       GPIO_PIN_4
#define SD_SPI_HANDLE   hspi1
// ----------------------------------------------------------

extern SPI_HandleTypeDef SD_SPI_HANDLE;

#define SD_CS_HIGH() HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET)
#define SD_CS_LOW()  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)

DSTATUS SD_disk_initialize(BYTE);
DSTATUS SD_disk_status(BYTE);
DRESULT SD_disk_read(BYTE* buff, BYTE sector, UINT count);
DRESULT SD_disk_write(const BYTE* buff, BYTE sector, UINT count);
DRESULT SD_disk_ioctl(BYTE cmd, void* buff);

#endif /* __FATFS_SD_H__ */
