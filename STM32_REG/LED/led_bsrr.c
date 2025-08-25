#include "stm32l4xx.h"
#include "stm32l4xx_gpio_driver.h"
#include <string.h>

// Simple software delay function
void delay()
{
	for(uint32_t i=0; i<50000; i++);
}

int main()
{
	GPIO_Handle_t GpioLed;   // GPIO handle structures for LED and Button (Button unused here)

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

	// Infinite loop to blink LED
	while(1)
	{
		GpioLed.pGPIOx->BSRR = (1<<5U);      // Set PA5 HIGH (LED ON)
		delay();                             // Delay
		GpioLed.pGPIOx->BSRR = (1<<(5+16));  // Reset PA5 LOW (LED OFF)
		delay();                             // Delay
	}
}
