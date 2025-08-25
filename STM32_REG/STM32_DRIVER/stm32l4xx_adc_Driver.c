#include "stm32l4xx.h"

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


void delay1(void)
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
    delay1();

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
void start_conversion(void)
{
    /* Enable continuous conversion mode */
    ADC1->CFGR |= (1<<13);

    /* Start ADC conversion */
    ADC1->CR |= (1<<2);
}

uint32_t adc_read(void)
{
    /* Wait until end of conversion */
    while(!(ADC1->ISR & (1<<2)));

    /* Return converted value */
    return (ADC1->DR);
}
