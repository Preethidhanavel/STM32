#include"stm32l4xx.h"
#include"stm32l4xx_tim.h"
#include"stm32l4xx_usart_driver.h"
#include"stm32l4xx_gpio_driver.h"
#include<stdio.h>
#include<string.h>

#define TX_BUFFER_SIZE 50
uint8_t tx_buffer[TX_BUFFER_SIZE] = "Hello DMA UART TX!\r\n";
USART_Handle_t usart2_handle;  //USART2 handle
// Initialize USART2 peripheral with required settings
void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;                                // Select USART2
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;  // Set baud rate to 115200
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE; // No hardware flow control
	usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;        // Enable both TX and RX
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1; // 1 stop bit
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS; // 8-bit data
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE; // No parity
	USART_Init(&usart2_handle); // Apply the configuration


}

// Configure GPIO pins for USART2 (PA2=TX, PA3=RX)
void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart2_gpio;
	memset(&usart2_gpio,0,sizeof(usart2_gpio)); // Clear structure

	usart2_gpio.pGPIOx = GPIOA;                                    // Use GPIO Port A
	usart2_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;     // Alternate function mode
	usart2_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;   // Push-pull output
	usart2_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;  // Pull-up resistor
	usart2_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;    // Fast speed
	usart2_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;             // AF7 = USART2

	// Configure PA2 as USART2_TX
	usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIOA_PCLK_EN();                // Enable clock for GPIOA
	GPIO_Init(&usart2_gpio);

	// Configure PA3 as USART2_RX
	usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart2_gpio);
}
//small software delay
void delay()
{
	for(uint32_t i=0;i<500000;i++);
}
// Redirect low-level printf output to USART2
int __io_putchar(int ch)
{
	// Send one character through USART2
	USART_SendData(&usart2_handle, (uint8_t*)&ch, 1);
	return(ch);   // Return the character sent
}
void uart2_dma_tx_init(void)
{
    /* Enable DMA1 clock */
    RCC->AHB1ENR |= (1<<0);

    /* Configure DMA1 Channel 7 for USART2_TX */
    DMA1_CSELR->CSELR &= ~(0xF << (4*(7-1)));   // Clear CH7 selection
    DMA1_CSELR->CSELR |=  (0X02000000);   // Map CH7 to USART2_TX (request=4)

    DMA1_Channel7->CCR &= ~(1<<0);          // Disable channel before config
    DMA1_Channel7->CPAR  = (uint32_t)&USART2->TDR; // Peripheral = USART2->TDR
    DMA1_Channel7->CMAR  = (uint32_t)tx_buffer;    // Memory buffer
    DMA1_Channel7->CNDTR = strlen((char*)tx_buffer); // Number of bytes to send
    DMA1_Channel7->CCR = (1<<4)| (1<<7)|(1<<13);    // memory-to-peripheral, memory increment,high priority

    /* Enable DMA transmission in USART2 */
    USART2->CR3 |= (1<<7);

    /* Finally enable DMA channel */
    DMA1_Channel7->CCR |= (1<<0);
}


int main()
{
	USART2_GPIOInit();                        // Initialize USART2 GPIO pins (PA2=TX, PA3=RX)
	USART2_Init();                            // Configure USART2
	USART_PeripheralControl(USART2, ENABLE);  // Enable USART2 peripheral
	printf("Hello World\r\n");  // uart printing
	uart2_dma_tx_init();             //intialise DMA transfer

	 while(1)
	    {
	        // Wait until transfer complete
	        while((DMA1_Channel7->CNDTR) != 0)
	        	;

		 	DMA1_Channel7->CCR &= ~(1<<0);
	        // Reload data for next transfer
	        strcpy((char*)tx_buffer, "DMA TX AGAIN\r\n");
	        DMA1_Channel7->CNDTR = strlen((char*)tx_buffer);

	        // Restart DMA
	        DMA1_Channel7->CCR |= (1<<0);
	        delay();
	    }


}
