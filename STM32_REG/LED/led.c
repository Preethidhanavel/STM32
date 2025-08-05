

#include <stdint.h>
#include"stm32l4xx.h"
#include"stm32l4xx_gpio_driver.h"
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif
void delay()
{
	for(uint32_t i=0;i<500000;i++);
}
int main(void)
{
    /* Loop forever */
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx =GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	//GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;  // OUTPUT is configure as push-pull
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD;    // OUTPUT is configured as open-drain
	//GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);
	//GPIO_WriteToOutputPin(GPIOA,GPIO_PIN_NO_5, GPIO_PIN_SET);
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}

}
