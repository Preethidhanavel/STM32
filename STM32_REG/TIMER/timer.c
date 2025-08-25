#include"stm32l4xx.h"
#include"stm32l4xx_tim.h"
#include"stm32l4xx_usart_driver.h"
#include"stm32l4xx_gpio_driver.h"
#include<stdio.h>
#include<string.h>

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

	usart2_gpio.pGPIOx = GPIOA;                           // Use GPIO Port A
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
GPIO_Handle_t GpioLed;   // GPIO handle structures for LED and Button (Button unused here)

void Led_Init(void)
{

		// Clear the structures (initialize to zero)
		memset(&GpioLed,0,sizeof(GpioLed));

		// Configure LED pin (PA5) as output
		GpioLed.pGPIOx = GPIOA;                           // Use GPIO Port A
		GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;   // Select pin 5
		GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;     // Set as output mode
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;  // Set speed to fast
		GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // Output type = Push-pull
		GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;  // Enable internal pull-up

		// Enable clock for GPIOA
		GPIO_PeriClockControl(GPIOA, ENABLE);

		// Initialize GPIO with above configuration
		GPIO_Init(&GpioLed);

}
int main()
{
	USART2_GPIOInit();                        // Initialize USART2 GPIO pins (PA2=TX, PA3=RX)
	USART2_Init();                            // Configure USART2
	USART_PeripheralControl(USART2, ENABLE);  // Enable USART2 peripheral
	Led_Init();                 // Initialize the LED
	timer2_1hz_init();          // Initialize Timer2 to generate 1 Hz update events

	while(1)
	{
	    // Wait until the update interrupt flag (UIF) is set
	    while(!(TIM2->SR & SR_UIF));

	    // Clear the update interrupt flag
	    TIM2->SR &= ~(SR_UIF);

	    // Toggle the LED (connected to GPIO pin 5)
	    GpioLed.pGPIOx->ODR ^= (1 << 5);

	    // Print a debug message
	    printf("Timer2-Delay\r\n");
	}

}
