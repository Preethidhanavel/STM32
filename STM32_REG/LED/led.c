#include <stdint.h>
#include"stm32l4xx.h"
#include"stm32l4xx_gpio_driver.h"

// Warning if FPU is enabled but not initialized
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

// Simple delay function
void delay()
{
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
    /* Configure LED GPIO */
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOA;                        // Port A
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5; // Pin 5
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;   // Output mode
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST; // Fast speed
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD; // Open-drain output
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Pull-up enabled

	GPIO_PeriClockControl(GPIOA, ENABLE); // Enable clock for GPIOA
	GPIO_Init(&GpioLed);                  // Initialize LED GPIO

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5); // Toggle LED state
		delay();                                     // Delay for visible blinking
	}
}
