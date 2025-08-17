#include<stdint.h>
#include"stm32l4xx.h"
#include"stm32l4xx_gpio_driver.h"

// Simple delay function
void delay()
{
	for(uint32_t i = 0; i < 250000; i++);
}

int main(void)
{
    /* Declare GPIO handles for LED and Button */
	GPIO_Handle_t GpioLed, GpioBtn;

    // --- LED Configuration ---
	GpioLed.pGPIOx = GPIOA;                          // LED on Port A
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5; // Pin 5
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;   // Output mode
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST; // Fast speed
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // Push-pull output
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // No pull-up/pull-down
	GPIO_PeriClockControl(GPIOA, ENABLE);           // Enable GPIOA clock
	GPIO_Init(&GpioLed);                            // Initialize LED GPIO

    // --- Button Configuration ---
	GpioBtn.pGPIOx = GPIOC;                          // Button on Port C
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13; // Pin 13
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;     // Input mode
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST; // Fast speed
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // No pull-up/pull-down
	GPIO_PeriClockControl(GPIOC, ENABLE);           // Enable GPIOC clock
	GPIO_Init(&GpioBtn);                            // Initialize Button GPIO

	while(1)
	{
		// Check if button is pressed (PC13 = 0)
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == 0)
		{
			delay(); // Debounce delay
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5); // Toggle LED
		}
	}
}
