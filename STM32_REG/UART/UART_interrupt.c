

#include<stdio.h>
#include<string.h>
#include "stm32l4xx.h"
#include"stm32l4xx_usart_driver.h"
//#include"syscalls.c"


char msg[30] = "UART_INTERRUPT";




USART_Handle_t usart2_handle;



volatile uint8_t rxCmplt = RESET;
volatile uint32_t rx_index = 0;
char rx_buf[1024];


uint8_t g_data = 0;

extern void initialise_monitor_handles();



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

void 	USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode =7;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpios);

	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpios);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;

	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOC,ENABLE);
	GPIO_Init(&GPIOBtn);

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioLed);

}

void delay(void)
{
	for(uint32_t i = 0 ; i < 250000 ; i ++);
}
int main(void)
{

	initialise_monitor_handles();

	USART2_GPIOInit();
    USART2_Init();
    GPIO_ButtonInit();
    USART_IRQInterruptConfig(IRQ_NO_USART2,ENABLE);
    USART_PeripheralControl(USART2,ENABLE);
    printf("Application is running\n");

    while(1)
    {
	//	while(  GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) );
		delay();
		USART_SendData(&usart2_handle,(uint8_t*)msg,strlen(msg));
		USART_ReceiveDataIT(&usart2_handle,(uint8_t *)&rx_buf[rx_index],1);

    	while (1)
    	{


    	    if (rxCmplt == SET)
    	    {
    	       printf("You typed: %s\n", rx_buf);
    	        rxCmplt = RESET;
    	        USART_ReceiveDataIT(&usart2_handle, (uint8_t *)&rx_buf[rx_index], 1);  // Restart
    	    }

    	}
    }


	return 0;
}


void USART2_IRQHandler(void)
{
	USART_IRQHandling(&usart2_handle);
}


void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApEv)
{
	if (ApEv == USART_EVENT_RX_CMPLT)
	{
		printf("%c ",rx_buf[rx_index]);
		if(rx_buf[rx_index]=='a')
		{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();
		}
		USART_SendData(pUSARTHandle, (uint8_t *)&rx_buf[rx_index], 1);

		if (rx_buf[rx_index] == '\r' || rx_buf[rx_index] == '\n')
		{
			rx_buf[rx_index] = '\0';
			rxCmplt = SET;
			rx_index = 0;
		}
		else
		{
			rx_index++;
			USART_ReceiveDataIT(pUSARTHandle, (uint8_t *)&rx_buf[rx_index], 1);
		}
	}
}




