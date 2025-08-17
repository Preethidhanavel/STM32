#include "lcd.h"

// Internal helper functions
static void write_4_bits(uint8_t value);
static void lcd_enable(void);
static void mdelay(uint32_t cnt);
static void udelay(uint32_t cnt);

// Send a command to LCD
void lcd_send_command(uint8_t cmd)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET); // RS = 0 for command
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET); // RW = 0 for write

	write_4_bits(cmd >> 4);  // Send higher nibble
	write_4_bits(cmd & 0x0F); // Send lower nibble
}

// Send a single character to LCD
void lcd_print_char(uint8_t data)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET); // RS = 1 for data
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET); // RW = 0 for write

	write_4_bits(data >> 4); // Higher nibble
	write_4_bits(data & 0x0F); // Lower nibble
}

// Print string on LCD
void lcd_print_string(char *message)
{
    do
    {
        lcd_print_char((uint8_t)*message++); // Print one character at a time
    }
    while (*message != '\0');
}

// Initialize LCD
void lcd_init(void)
{
	GPIO_Handle_t lcd_signal;

	// Configure LCD pins as output
	lcd_signal.pGPIOx = LCD_GPIO_PORT;
	lcd_signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	lcd_signal.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	lcd_signal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	lcd_signal.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// Initialize control pins
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
	GPIO_Init(&lcd_signal);
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
	GPIO_Init(&lcd_signal);
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	GPIO_Init(&lcd_signal);

	// Initialize data pins D4-D7
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4; GPIO_Init(&lcd_signal);
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5; GPIO_Init(&lcd_signal);
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6; GPIO_Init(&lcd_signal);
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7; GPIO_Init(&lcd_signal);

	// Reset all pins
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	mdelay(40); // Wait for LCD to power up

	// LCD initialization sequence
	write_4_bits(0x3); mdelay(5);
	write_4_bits(0x3); udelay(150);
	write_4_bits(0x3);
	write_4_bits(0x2);

	lcd_send_command(LCD_CMD_4DL_2N_5X8F); // 4-bit, 2 lines, 5x8 font
	lcd_send_command(LCD_CMD_DON_CURON);    // Display ON, cursor ON

	lcd_display_clear();                    // Clear display
	lcd_send_command(LCD_CMD_INCADD);       // Entry mode
}

// Write 4 bits to data pins
static void write_4_bits(uint8_t value)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, (value >> 0) & 0x1);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, (value >> 1) & 0x1);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, (value >> 2) & 0x1);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, (value >> 3) & 0x1);

	lcd_enable(); // Generate enable pulse
}

// Clear LCD display
void lcd_display_clear(void)
{
	lcd_send_command(LCD_CMD_DIS_CLEAR);
	mdelay(2);
}

// Return cursor to home position
void lcd_display_return_home(void)
{
	lcd_send_command(LCD_CMD_DIS_RETURN_HOME);
	mdelay(2);
}

// Set cursor to specific row and column
void lcd_set_cursor(uint8_t row, uint8_t column)
{
  column--;
  switch (row)
  {
    case 1:
      lcd_send_command((column |= 0x80)); // First row address
      break;
    case 2:
      lcd_send_command((column |= 0xC0)); // Second row address
      break;
    default:
      break;
  }
}

// Generate enable pulse
static void lcd_enable(void)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	udelay(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	udelay(100);
}

// Millisecond delay
static void mdelay(uint32_t cnt)
{
	for(uint32_t i = 0; i < (cnt * 1000); i++);
}

// Microsecond delay
static void udelay(uint32_t cnt)
{
	for(uint32_t i = 0; i < cnt; i++);
}
