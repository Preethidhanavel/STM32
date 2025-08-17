#include<stdio.h>
#include "rtc.h"
#include"stm32l4xx_usart_driver.h"
#include "lcd.h"
#include<string.h>

#define SYSTICK_TIM_CLK   16000000UL  // SysTick clock frequency
#define PRINT_LCD                     // Enable LCD printing

// Initialize SysTick timer to generate interrupt at tick_hz frequency
void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014; // SysTick Reload Value Register
	uint32_t *pSCSR = (uint32_t*)0xE000E010; // SysTick Control/Status Register
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    *pSRVR &= ~(0x00FFFFFFFF); // Clear reload value
    *pSRVR |= count_value;      // Set new reload value
    *pSCSR |= ( 1 << 1);        // Enable SysTick interrupt
    *pSCSR |= ( 1 << 2);        // Select processor clock
    *pSCSR |= ( 1 << 0);        // Enable SysTick timer
}

// Return day of week as string
char* get_day_of_week(uint8_t i)
{
	char* days[] = { "Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};
	return days[i-1];
}

// Convert number to 2-character string
void number_to_string(uint8_t num , char* buf)
{
	if(num < 10){
		buf[0] = '0';
		buf[1] = num + 48; // ASCII conversion
	}else if(num >= 10 && num < 99){
		buf[0] = (num/10) + 48;
		buf[1]= (num % 10) + 48;
	}
}

// Convert RTC_time_t to string "HH:MM:SS"
char* time_to_string(RTC_time_t *rtc_time)
{
	static char buf[9];

	buf[2]= ':';
	buf[5]= ':';

	number_to_string(rtc_time->hours, buf);
	number_to_string(rtc_time->minutes, &buf[3]);
	number_to_string(rtc_time->seconds, &buf[6]);

	buf[8] = '\0';
	return buf;
}

// Convert RTC_date_t to string "DD/MM/YY"
char* date_to_string(RTC_date_t *rtc_date)
{
	static char buf[9];

	buf[2]= '/';
	buf[5]= '/';

	number_to_string(rtc_date->date, buf);
	number_to_string(rtc_date->month, &buf[3]);
	number_to_string(rtc_date->year, &buf[6]);

	buf[8]= '\0';
	return buf;
}

// Simple millisecond delay
static void mdelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000); i++);
}

// USART2 handle
USART_Handle_t usart2_handle;

// Initialize USART2 for debugging
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

// Initialize GPIO pins for USART2
void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart2_gpio;
	memset(&usart2_gpio,0,sizeof(usart2_gpio));
	usart2_gpio.pGPIOx = GPIOA;
	usart2_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart2_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart2_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart2_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart2_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	// TX pin
	usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIOA_PCLK_EN();
	GPIO_Init(&usart2_gpio);

	// RX pin
	usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart2_gpio);
}

int main(void)
{
	// Initialize USART2 and LCD
	USART2_GPIOInit();
	USART2_Init();
	USART_PeripheralControl(USART2, ENABLE);

	RTC_time_t current_time;
	RTC_date_t current_date;

#ifndef PRINT_LCD
	printf("RTC test\n");
#else
	lcd_init();
	lcd_print_string("RTC Test...");
	USART_SendData(&usart2_handle, (uint8_t*)"RTC Test..\r\n",12);
	mdelay(2000);
	lcd_display_clear();
	lcd_display_return_home();
#endif

	// Initialize DS1307 RTC
	if(ds1307_init()){
		printf("RTC init has failed\n");
		USART_SendData(&usart2_handle, (uint8_t *)"Init FAIL1\r\n",12);
		while(1);
	}

	init_systick_timer(1); // 1Hz SysTick

	// Set initial date and time
	current_date.day = FRIDAY;
	current_date.date = 12;
	current_date.month = 8;
	current_date.year = 25;

	current_time.hours = 18;
	current_time.minutes = 00;
	current_time.seconds = 30;
	current_time.time_format = TIME_FORMAT_12HRS_PM;

	ds1307_set_current_date(&current_date);
	ds1307_set_current_time(&current_time);

	// Read back current time and date
	ds1307_get_current_time(&current_time);
	ds1307_get_current_date(&current_date);

	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS)
	{
		am_pm = (current_time.time_format) ? "PM" : "AM";
#ifndef PRINT_LCD
		printf("Current time = %s %s\n", time_to_string(&current_time), am_pm);
#else
		lcd_print_string(time_to_string(&current_time));
		lcd_print_string(am_pm);
#endif
		USART_SendData(&usart2_handle, (uint8_t *)"current_time\r\n",14);
	}
	else
	{
#ifndef PRINT_LCD
		printf("Current time = %s\n", time_to_string(&current_time));
#else
		lcd_print_string(time_to_string(&current_time));
#endif
		USART_SendData(&usart2_handle, (uint8_t *)"current_time\r\n",14);
	}

#ifndef PRINT_LCD
	printf("Current date = %s <%s>\n", date_to_string(&current_date), get_day_of_week(current_date.day));
#else
	lcd_set_cursor(2, 1);
	lcd_print_string(date_to_string(&current_date));
#endif
	USART_SendData(&usart2_handle, (uint8_t *)"current_date_day\r\n",18);

	while(1); // Infinite loop

	return 0;
}

// SysTick interrupt handler, updates LCD or prints current date/time
void SysTick_Handler(void)
{
	RTC_time_t current_time;
	RTC_date_t current_date;

	ds1307_get_current_time(&current_time);

	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS){
		am_pm = (current_time.time_format) ? "PM" : "AM";
#ifndef PRINT_LCD
		printf("Current time = %s %s\n", time_to_string(&current_time), am_pm);
#else
		lcd_set_cursor(1, 1);
		lcd_print_string(time_to_string(&current_time));
		lcd_print_string(am_pm);
#endif
		USART_SendData(&usart2_handle, (uint8_t *)"current_time\r\n",14);
	}
	else
	{
#ifndef PRINT_LCD
		printf("Current time = %s\n", time_to_string(&current_time));
#else
		lcd_set_cursor(1, 1);
		lcd_print_string(time_to_string(&current_time));
#endif
		USART_SendData(&usart2_handle, (uint8_t *)"current_time\r\n",14);
	}

	ds1307_get_current_date(&current_date);

#ifndef PRINT_LCD
	printf("Current date = %s <%s>\n", date_to_string(&current_date), get_day_of_week(current_date.day));
#else
	lcd_set_cursor(2, 1);
	lcd_print_string(date_to_string(&current_date));
	lcd_print_char('<');
	lcd_print_string(get_day_of_week(current_date.day));
	lcd_print_char('>');
#endif
	USART_SendData(&usart2_handle, (uint8_t *)"current_date_day\r\n",18);
}
