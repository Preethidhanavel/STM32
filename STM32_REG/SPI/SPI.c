#include <stdint.h>
#include "stm32l4xx.h"
#include "stm32l4xx_spi_driver.h"
#include "uart_driver.h"
#include <string.h>

// SD card commands
#define CMD0   (0x40+0)      // GO_IDLE_STATE
#define CMD17  (0x40+17)     // READ_SINGLE_BLOCK
#define CMD24  (0x40+24)     // WRITE_SINGLE_BLOCK
#define DUMMY  0xFF          // Dummy byte for SPI clocks

// SD card chip select pin
#define SD_CS_PORT GPIOA
#define SD_CS_PIN  GPIO_PIN_NO_4
#define SD_CS_LOW()  GPIO_WriteToOutputPin(SD_CS_PORT, SD_CS_PIN, RESET)
#define SD_CS_HIGH() GPIO_WriteToOutputPin(SD_CS_PORT, SD_CS_PIN, SET)

void string(uint8_t *s);  // Send string via UART

// SPI send & receive one byte
uint8_t SPI_Transfer(uint8_t data)
{
    uint8_t rx;
    SPI_SendData(SPI1, &data,  1);
    SPI_ReceiveData(SPI1, &rx, 1);
    return rx;
}

// Send command to SD card
void SD_Command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    SPI_Transfer(cmd | 0x40);   // command + start bit
    SPI_Transfer(arg >> 24);     // MSB
    SPI_Transfer(arg >> 16);
    SPI_Transfer(arg >> 8);
    SPI_Transfer(arg);           // LSB
    SPI_Transfer(crc);           // CRC
}

// Wait for SD card R1 response
uint8_t SD_WaitR1(void)
{
    uint8_t res;
    for (int i = 0; i < 100; i++)
    {
        res = SPI_Transfer(0xFF);
        if ((res & 0x80) == 0)
            return res;
    }
    return 0xFF;  // timeout
}

// Wait until SD card is not busy
void SD_WaitNotBusy(void)
{
    while (SPI_Transfer(0xFF) != 0xFF);
}

// Initialize SD card
int SD_Init(void)
{
    SD_CS_HIGH();            // CS idle high

    for (int i = 0; i < 10; i++)
        SPI_Transfer(0xFF); // Send 80 clock cycles

    SD_CS_LOW();
    SD_Command(0, 0, 0x95);  // CMD0: go idle
    if (SD_WaitR1() != 0x01) // should respond with 0x01
    {
        SD_CS_HIGH();
        return -1;
    }
    SD_CS_HIGH();
    SPI_Transfer(0xFF);
    return 0;
}

// Read 512-byte block from SD card
int SD_ReadBlock(uint8_t *buf, uint32_t block)
{
    uint8_t token;
    block *= 512;             // Convert block number to byte address

    SD_CS_LOW();
    SD_Command(CMD17, block, 0xFF); // CMD17: read single block

    if (SD_WaitR1() != 0x00) {     // wait for ready
        SD_CS_HIGH();
        return -1;
    }

    // Wait for start token 0xFE
    for (int i = 0; i < 1000; i++) {
        token = SPI_Transfer(0xFF);
        if (token == 0xFE) break;
    }
    if (token != 0xFE) {
        SD_CS_HIGH();
        return -2;
    }

    // Read 512 bytes
    for (int i = 0; i < 512; i++)
        buf[i] = SPI_Transfer(0xFF);

    SPI_Transfer(0xFF);  // dummy CRC
    SPI_Transfer(0xFF);

    SD_CS_HIGH();
    SPI_Transfer(0xFF);
    return 0;
}

// Write 512-byte block to SD card
uint8_t SD_WriteBlock(uint8_t *buf, uint32_t block)
{
    uint8_t response;
    block *= 512;              // convert block number to byte address

    SD_CS_LOW();
    SD_Command(CMD24, block, 0xFF); // CMD24: write single block

    if (SD_WaitR1() != 0x00) {
        SD_CS_HIGH();
        return 1;
    }

    SPI_Transfer(0xFE);        // data token
    for (int i = 0; i < 512; i++)
        SPI_Transfer(buf[i]);

    SPI_Transfer(0xFF);        // dummy CRC
    SPI_Transfer(0xFF);

    response = SPI_Transfer(0xFF);
    if ((response & 0x1F) != 0x05) {
        SD_CS_HIGH();
        return 2;
    }

    SD_WaitNotBusy();
    SD_CS_HIGH();
    SPI_Transfer(0xFF);
    return 0;
}

// Initialize SPI1 GPIO pins
void SPI1_GPIOInit(void)
{
    GPIO_Handle_t SPIPin;
    SPIPin.pGPIOx = GPIOA;
    SPIPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPin.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPIPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // SCLK
    SPIPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&SPIPin);

    // MOSI
    SPIPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Init(&SPIPin);

    // MISO
    SPIPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&SPIPin);

    // NSS (CS)
    SPIPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
    GPIO_Init(&SPIPin);
}

// Initialize SPI1 peripheral
void SPI1_Init(void)
{
    SPI_Handle_t SPI1Handle;
    SPI1Handle.pSPIx = SPI1;
    SPI1Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI1Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI1Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256;
    SPI1Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI1Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI1Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI1Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

    SPI_Init(&SPI1Handle);
}

// Initialize button GPIO
void GPIO_ButtonInit(void)
{
    GPIO_Handle_t GPIOBtn;
    memset(&GPIOBtn,0,sizeof(GPIOBtn));
    GPIOBtn.pGPIOx = GPIOC;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_PeriClockControl(GPIOC,ENABLE);
    GPIO_Init(&GPIOBtn);
}

USART_Handle_t usart2_handle;

// Initialize USART2 peripheral
void USART2_Init(void)
{
    usart2_handle.pUSARTx = USART2;
    usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
    usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
    usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
    USART_Init(&usart2_handle);
}

// Initialize USART2 GPIO pins
void USART2_GPIOInit(void)
{
    GPIO_Handle_t usart_gpios;
    usart_gpios.pGPIOx = GPIOA;
    usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode =7;
    GPIO_PeriClockControl(GPIOA, ENABLE);

    // TX pin
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_Init(&usart_gpios);

    // RX pin
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_Init(&usart_gpios);
}

// Simple delay
void delay(void)
{
    for(uint32_t i = 0 ; i < 250000 ; i++);
}

// Main function
int main(void)
{
    int i,j;
    uint8_t write_buf[512], read_buf[512];

    // Initialize peripherals
    GPIO_ButtonInit();
    SPI1_GPIOInit();
    SPI1_Init();
    USART2_GPIOInit();
    USART2_Init();
    USART_PeripheralControl(USART2,ENABLE);

    while(1)
    {
        // Wait for button press
        while((GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)==1));
        delay();

        SPI_PeripheralControl(SPI1,ENABLE);

        // Initialize SD card
        if (SD_Init())
        {
            string((uint8_t*)"FAILED TO OPEN SD\r\n");
            while (1);
        }

        string((uint8_t*)"WRITING INTO SD\r\n");

        // Prepare write buffer with A-Z
        for (i = 0,j=0; i < 512; i++,j++){
            write_buf[i] = 'A' + (j % 26);
            if(j==25)
                j=0;
        }
        write_buf[512]='\0';

        // Write block to SD
        if (SD_WriteBlock(write_buf, 0x00000000) != 0)
        {
            string((uint8_t*)"WRITE FAIL\r\n");
            while (1);
        }

        string((uint8_t*)"READING BLOCK...\r\n");

        // Read block from SD
        if (SD_ReadBlock(read_buf, 0x00000000) != 0)
        {
            string((uint8_t*)"READ FAIL\r\n");
            while (1);
        }
        read_buf[512]='\0';

        string(read_buf); // Send read data over UART
        while (1);
    }
}

// Send string via USART
void string(uint8_t *s)
{
    while(*s)
    {
        USART_SendData(&usart2_handle, s, 1);
        s++;
    }
}
