#include<stdio.h>
#include<string.h>
#include "stm32l4xx.h"
#include"stm32l4xx_i2c_driver.h"
#include"stm32l4xx_usart_driver.h"

#define SLAVE_ADDR  0x68   // I2C slave device address
#define MY_ADDR 0x61       // STM32 I2C own address

I2C_Handle_t I2C1Handle;

uint8_t some_data[] = "I2C";  // Data to send via I2C

// Initialize I2C1 GPIO pins (SCL and SDA)
void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;                         // Port B
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN; // Alternate function mode
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD; // Open-drain
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Pull-up
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;  // AF4 for I2C
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_PeriClockControl(GPIOB, ENABLE);

	// SCL pin PB6
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	// SDA pin PB7
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}

// Initialize I2C1 peripheral
void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE; // Enable ACK
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;      // STM32 own address

	I2C_Init(&I2C1Handle);
}

// Initialize button on PC13
void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;    // Input mode
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // No pull-up/pull-down
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIOBtn);
}

uint8_t msg[50] = "Transmit completed\r\n"; // Message to send via USART

USART_Handle_t usart2_handle;

// Simple delay function
void delay(void)
{
	for(uint32_t i = 0 ; i < 250000 ; i++);
}

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

// Initialize GPIO pins for USART2 (PA2 -> TX, PA3 -> RX)
void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart2_gpio;
	memset(&usart2_gpio, 0, sizeof(usart2_gpio));

	usart2_gpio.pGPIOx = GPIOA;
	usart2_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;  // Alternate function
	usart2_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // Push-pull
	usart2_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Pull-up
	usart2_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart2_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7; // AF7 for USART2

	// TX pin PA2
	usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIOA_PCLK_EN();
	GPIO_Init(&usart2_gpio);

	// RX pin PA3
	usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart2_gpio);
}

int main(void)
{
	// Initialize USART2 GPIO and peripheral
	USART2_GPIOInit();
	USART2_Init();
	USART_PeripheralControl(USART2, ENABLE);

	// Initialize button
	GPIO_ButtonInit();

	// Enable I2C1 peripheral clock, initialize GPIO and I2C1
	I2C_PeriClockControl(I2C1, ENABLE);
	I2C1_GPIOInits();
	I2C1_Inits();
	I2C_PeripheralControl(I2C1, ENABLE);

	while(1)
	{
		// Wait until button is pressed (PC13 = 0)
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == 1);
		delay(); // Debounce delay

		// Send data "I2C" to slave device via I2C
		I2C_MasterSendData(&I2C1Handle, some_data, 3, SLAVE_ADDR, 0);

		// Send confirmation message via USART2
		USART_SendData(&usart2_handle, msg, 20);
	}
}
