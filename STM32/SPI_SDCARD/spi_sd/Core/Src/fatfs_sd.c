#define TRUE 1
#define FALSE 0
#define bool BYTE
#include "fatfs_sd.h"
#include "stm32l4xx_hal.h"
#include "string.h"

// SD card commands
#define CMD0  (0x40+0)
#define CMD8  (0x40+8)
#define CMD17 (0x40+17)
#define CMD24 (0x40+24)
#define CMD55 (0x40+55)
#define CMD58 (0x40+58)
#define ACMD41 (0x40+41)

extern SPI_HandleTypeDef SD_SPI_HANDLE;

static uint8_t SD_SPI_Transmit(uint8_t data) {
    uint8_t received;
    HAL_SPI_TransmitReceive(&SD_SPI_HANDLE, &data, &received, 1, HAL_MAX_DELAY);
    return received;
}

static void SD_SPI_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc) {
    uint8_t buf[6];
    buf[0] = cmd;
    buf[1] = (uint8_t)(arg >> 24);
    buf[2] = (uint8_t)(arg >> 16);
    buf[3] = (uint8_t)(arg >> 8);
    buf[4] = (uint8_t)(arg);
    buf[5] = crc;

    SD_CS_LOW();
    for (uint8_t i = 0; i < 6; i++) {
        SD_SPI_Transmit(buf[i]);
    }
}

DSTATUS SD_disk_initialize(BYTE pdrv) {
    HAL_Delay(1);
    SD_CS_HIGH();
    for (uint8_t i = 0; i < 10; i++) SD_SPI_Transmit(0xFF);

    SD_SPI_SendCommand(CMD0, 0, 0x95);
    for (uint8_t i = 0; i < 10; i++) {
        if (SD_SPI_Transmit(0xFF) == 0x01) break;
    }
    SD_CS_HIGH();
    SD_SPI_Transmit(0xFF);
    return 0;
}

DSTATUS SD_disk_status(BYTE pdrv) {
    return 0;
}

DRESULT SD_disk_read(BYTE *buff, BYTE sector, UINT count) {
    return RES_OK; // Stubbed
}

DRESULT SD_disk_write(const BYTE *buff, BYTE sector, UINT count) {
    return RES_OK; // Stubbed
}

DRESULT SD_disk_ioctl(BYTE cmd, void *buff) {
    return RES_OK; // Stubbed
}
