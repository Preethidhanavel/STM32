#ifndef LCD_H_
#define LCD_H_

#include "stm32l4xx.h"

// --- LCD Function Prototypes ---
// Initialize the LCD
void lcd_init(void);

// Send a command to the LCD
void lcd_send_command(uint8_t cmd);

// Send a single character to the LCD
void lcd_print_char(uint8_t data);

// Clear the LCD display
void lcd_display_clear(void);

// Return cursor to home position
void lcd_display_return_home(void);

// Print a string on the LCD
void lcd_print_string(char*);

// Set cursor to specific row and column (1-based)
void lcd_set_cursor(uint8_t row, uint8_t column);

// --- LCD GPIO Pin Configuration ---
#define LCD_GPIO_PORT  GPIOD   // Port where LCD is connected
#define LCD_GPIO_RS    GPIO_PIN_NO_0  // Register Select pin
#define LCD_GPIO_RW    GPIO_PIN_NO_1  // Read/Write pin
#define LCD_GPIO_EN    GPIO_PIN_NO_2  // Enable pin
#define LCD_GPIO_D4    GPIO_PIN_NO_3  // Data pin 4
#define LCD_GPIO_D5    GPIO_PIN_NO_4  // Data pin 5
#define LCD_GPIO_D6    GPIO_PIN_NO_5  // Data pin 6
#define LCD_GPIO_D7    GPIO_PIN_NO_6  // Data pin 7

// --- LCD Command Macros ---
#define LCD_CMD_4DL_2N_5X8F        0x28  // 4-bit, 2 lines, 5x8 font
#define LCD_CMD_DON_CURON           0x0E  // Display ON, Cursor ON
#define LCD_CMD_INCADD              0x06  // Increment cursor position
#define LCD_CMD_DIS_CLEAR           0x01  // Clear display
#define LCD_CMD_DIS_RETURN_HOME     0x02  // Return cursor to home position

#endif /* LCD_H_ */
