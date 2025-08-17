#ifndef RTC_H_
#define RTC_H_

#include "stm32l4xx.h"

// DS1307 I2C peripheral and GPIO configuration
#define DS1307_I2C  			I2C1                // I2C peripheral used
#define DS1307_I2C_GPIO_PORT    GPIOB               // GPIO port for SDA and SCL
#define DS1307_I2C_SDA_PIN 		GPIO_PIN_NO_7       // SDA pin
#define DS1307_I2C_SCL_PIN 		GPIO_PIN_NO_6       // SCL pin
#define DS1307_I2C_SPEED 		I2C_SCL_SPEED_SM    // Standard mode speed
#define DS1307_I2C_PUPD			GPIO_PIN_PU         // Pull-up configuration

// DS1307 register addresses
#define DS1307_ADDR_SEC 		0x00
#define DS1307_ADDR_MIN 		0x01
#define DS1307_ADDR_HRS			0x02
#define DS1307_ADDR_DAY			0x03
#define DS1307_ADDR_DATE		0x04
#define DS1307_ADDR_MONTH		0x05
#define DS1307_ADDR_YEAR		0x06

// Time formats
#define TIME_FORMAT_12HRS_AM 	0   // 12-hour AM
#define TIME_FORMAT_12HRS_PM 	1   // 12-hour PM
#define TIME_FORMAT_24HRS 		2   // 24-hour format

// DS1307 I2C device address
#define DS1307_I2C_ADDRESS      0x68

// Days of the week
#define SUNDAY  	1
#define MONDAY  	2
#define TUESDAY  	3
#define WEDNESDAY   4
#define THURSDAY  	5
#define FRIDAY  	6
#define SATURDAY  	7

// Structure to store date
typedef struct
{
	uint8_t date;   // Day of month (1-31)
	uint8_t month;  // Month (1-12)
	uint8_t year;   // Year (0-99)
	uint8_t day;    // Day of week (1-7)
} RTC_date_t;

// Structure to store time
typedef struct
{
	uint8_t seconds;      // Seconds (0-59)
	uint8_t minutes;      // Minutes (0-59)
	uint8_t hours;        // Hours (0-23 or 1-12)
	uint8_t time_format;  // 12-hour or 24-hour format
} RTC_time_t;

// Function prototypes
uint8_t ds1307_init(void);                       // Initialize DS1307 RTC

void ds1307_set_current_time(RTC_time_t *);      // Set current time
void ds1307_get_current_time(RTC_time_t *);      // Get current time
void ds1307_set_current_date(RTC_date_t *);      // Set current date
void ds1307_get_current_date(RTC_date_t *);      // Get current date

#endif /* RTC_H_ */
