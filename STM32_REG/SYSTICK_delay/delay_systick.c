#include"stm32l4xx.h"
#include"stm32l4xx_systick.h"
#include<string.h>

int main()
{
		GPIO_Handle_t GpioLed;
		memset(&GpioLed,0,sizeof(GpioLed));              // Clear the GPIO handle structure
		GpioLed.pGPIOx =GPIOA;                          // Use GPIO Port A
		GpioLed.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_5;   // Select Pin 5
		GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;      // Set pin as Output
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;   // Set output speed to Fast
		GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;  // Configure output as Push-Pull
		GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;  // Enable Pull-Up resistor

		GPIO_PeriClockControl(GPIOA, ENABLE);          // Enable clock for GPIOA
		GPIO_Init(&GpioLed);                           // Initialize GPIOA Pin 5 with given settings

		while(1)
		{
			GpioLed.pGPIOx->ODR = (1<<5);              // Set Pin 5 HIGH (LED ON)
			systickDelayMs(500);                       // Delay
			GpioLed.pGPIOx->ODR &=~(1<<5);             // Clear Pin 5 (LED OFF)
			systickDelayMs(500);                       // Delay
		}


}
