#include "stm32l4xx.h"
#include"stm32l4xx_usart_driver.h"
#include<stdio.h>
#include<string.h>
#define RCC_AHB2ENR_ADCEN       (1U << 13)  // Bit 13: ADC clock enable
#define ADC_CR_ADEN             (1UL << 0U)      // ADC enable

#define ADC_CR_ADDIS            (1UL << 1U)     // ADC disable
#define ADC_CR_ADSTART          (1UL << 2U)   // Start conversion
#define ADC_CR_ADCAL            (1UL << 31U)     // Calibration
#define ADC_CR_ADVREGEN         (1UL << 28U)  // Regulator enable
#define ADC_CR_DEEPPWD          (1UL << 29U)   // Deep-power-down
#define ADC_CCR_CKMODE_Msk      (0x3UL << 16U)
#define ADC_CCR_CKMODE          ADC_CCR_CKMODE_Msk

#define ADC_CCR_CKMODE_DIV1     (0x1UL << 16U) // HCLK / 1
#define ADC_CCR_CKMODE_DIV2     (0x2UL << 16U) // HCLK / 2
#define ADC_ISR_ADRDY           (1UL << 0U)    // ADC ready
#define ADC_ISR_EOC             (1UL << 2U)      // End of conversion
#define ADC_SQR1_L_Msk          (0xFUL << 0U)
#define ADC_SQR1_L              ADC_SQR1_L_Msk                // Sequence length


void delay(void)
{
	for (volatile int i = 0; i < 1000; i++); // Delay ~20us
}
void adc_init(void)
{
    /*** 1. Enable clocks ***/
    GPIOA_PCLK_EN();
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;     // ADC clock

    /*** 2. Configure GPIO ***/
    // ADC1_IN5 = PA0
    GPIOA->MODER |= (3U << (0 * 2));       // PA0 analog
    GPIOA->PUPDR &= ~(3U << (0 * 2));      // No pull
    GPIOA->ASCR  |= (1U << 0);             // Connect analog switch

    /*** 3. Configure ADC clock source ***/
    ADCOM->CCR &= ~ADC_CCR_CKMODE;
    ADCOM->CCR |= ADC_CCR_CKMODE_DIV1; // HCLK / 1

    /*** 4. Power-up ADC ***/
    ADC1->CR &= ~ADC_CR_DEEPPWD;       // Exit deep power-down
    ADC1->CR |= ADC_CR_ADVREGEN;       // Enable regulator
    delay();

    /*** 5. Calibrate ***/
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL);

    /*** 6. Sampling time for channel 5 ***/
    ADC1->SMPR1 &= ~(0x7U << (5 * 3));
    ADC1->SMPR1 |=  (0x2U << (5 * 3)); // 12.5 cycles

    /*** 7. Sequence length & channel ***/
    ADC1->SQR1 &= ~ADC_SQR1_L;                   // 1 conversion
    ADC1->SQR1 &= ~(0x1FU << 6U);  // Clear SQ1
    ADC1->SQR1 |=  (5U << 6U);     // SQ1 = channel 5

    /*** 8. Enable ADC ***/
    ADC1->ISR |= ADC_ISR_ADRDY;        // Clear ADRDY
    ADC1->CR  |= ADC_CR_ADEN;          // Enable ADC
    while (!(ADC1->ISR & ADC_ISR_ADRDY));
}

/*** Function to read one ADC conversion ***/
uint16_t adc_read(void)
{
    ADC1->CR |= ADC_CR_ADSTART;            // Start conversion
    while (!(ADC1->ISR & ADC_ISR_EOC));    // Wait end of conversion
    return (uint16_t)(ADC1->DR);           // Return 12-bit result
}
USART_Handle_t usart2_handle;  //USART2 handle
// Initialize USART2 peripheral with required settings
void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;                                // Select USART2
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;  // Set baud rate to 115200
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE; // No hardware flow control
	usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;        // Enable both TX and RX
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1; // 1 stop bit
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS; // 8-bit data
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE; // No parity
	USART_Init(&usart2_handle); // Apply the configuration
}

// Configure GPIO pins for USART2 (PA2=TX, PA3=RX)
void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart2_gpio;
	memset(&usart2_gpio,0,sizeof(usart2_gpio)); // Clear structure

	usart2_gpio.pGPIOx = GPIOA;                           // Use GPIO Port A
	usart2_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;     // Alternate function mode
	usart2_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;   // Push-pull output
	usart2_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;  // Pull-up resistor
	usart2_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;    // Fast speed
	usart2_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;             // AF7 = USART2

	// Configure PA2 as USART2_TX
	usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIOA_PCLK_EN();                // Enable clock for GPIOA
	GPIO_Init(&usart2_gpio);

	// Configure PA3 as USART2_RX
	usart2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart2_gpio);
}

// Small delay function (software delay loop)
void delay1(void)
{
	for(uint32_t i = 0 ; i < 250000 ; i ++);
}

int main(void)
{
	char msg[30];   // Buffer to store formatted string
    float adc_val;  // Variable to hold ADC result

    USART2_GPIOInit();                        // Initialize USART2 GPIO pins
    USART2_Init();                            // Initialize USART2 peripheral
    USART_PeripheralControl(USART2, ENABLE);  // Enable USART2
    adc_init();                               // Initialize ADC

    while (1)
    {
        adc_val = adc_read(); // Read ADC value (0 – 4095 for 12-bit ADC)

        // Format the ADC value as a string
        sprintf(msg,"ADC-Value: %.2f\r\n",adc_val);

        // Transmit the string over USART2
        USART_SendData(&usart2_handle, (uint8_t*) msg, strlen(msg));

        delay1(); // Small delay between transmissions
    }
}
