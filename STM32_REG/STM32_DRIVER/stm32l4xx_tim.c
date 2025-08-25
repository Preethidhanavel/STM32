#include "stm32l4xx.h"
#include"stm32l4xx_tim.h"
#define TIM2EN      (1U<<0)   // APB1ENR1: TIM2 clock enable
#define TIM3EN      (1U<<1)   // APB1ENR1: TIM3 clock enable
#define CR1_CEN     (1U<<0)
#define GPIOA_EN    (1U<<0)   // AHB2ENR: GPIOA clock enable

void timer2_1hz_init(void)
{
    /* Enable clock access to TIM2 */
    RCC->APB1ENR1 |= TIM2EN;

    /* Set prescaler */
    TIM2->PSC = 16000 - 1;   // 16 MHz / 16000 = 1 kHz

    /* Set auto-reload value */
    TIM2->ARR = 1000 - 1;    // 1000 / 1000 Hz = 1 Hz

    /* Clear counter */
    TIM2->CNT = 0;

    /* Enable timer */
    TIM2->CR1 = CR1_CEN;
}

void timer2_pa5_output_compare(void)
{
    /* Enable GPIOA clock */
    RCC->AHB2ENR |= GPIOA_EN;

    /* Set PA5 to alternate function mode */
    GPIOA->MODER &= ~(3U << 10);   // clear MODER5
    GPIOA->MODER |=  (2U << 10);   // AF mode

    /* Set alternate function AF1 (TIM2_CH1 on PA5) */
    GPIOA->AFR[0] &= ~(0xF << 20);
    GPIOA->AFR[0] |=  (1U << 20);   // AF1

    /* Enable clock access to TIM2 */
    RCC->APB1ENR1 |= TIM2EN;

    /* Set prescaler */
    TIM2->PSC = 16000 - 1;   // 16 MHz / 16000 = 1 kHz

    /* Set auto reload value */
    TIM2->ARR = 1000 - 1;    // 1 Hz

    /* Configure output compare toggle mode on channel 1 */
    TIM2->CCMR1 &= ~(7U << 4);   // clear OC1M bits
    TIM2->CCMR1 |=  (3U << 4);   // toggle mode (011)

    /* Enable channel 1 output */
    TIM2->CCER |= (1U << 0);

    /* Clear counter */
    TIM2->CNT = 0;

    /* Enable timer */
    TIM2->CR1 = CR1_CEN;
}

void timer3_pa6_input_capture(void)
{
    /* Enable GPIOA clock */
    RCC->AHB2ENR |= GPIOA_EN;

    /* Set PA6 to alternate function mode */
    GPIOA->MODER &= ~(3U << 12);
    GPIOA->MODER |=  (2U << 12);

    /* Set AF2 (TIM3_CH1 on PA6) */
    GPIOA->AFR[0] &= ~(0xF << 24);
    GPIOA->AFR[0] |=  (2U << 24);

    /* Enable clock access to TIM3 */
    RCC->APB1ENR1 |= TIM3EN;

    /* Prescaler */
    TIM3->PSC = 16 - 1;   // 16 MHz / 16 = 1 MHz timer clock

    /* Set CH1 to input capture mode */
    TIM3->CCMR1 &= ~(3U << 0);
    TIM3->CCMR1 |=  (1U << 0);   // CC1S = 01 -> IC1 mapped on TI1

    /* Capture on rising edge */
    TIM3->CCER &= ~(1U << 1);    // CC1P = 0 (rising)

    /* Enable capture from channel 1 */
    TIM3->CCER |= (1U << 0);

    /* Enable timer */
    TIM3->CR1 = CR1_CEN;
}
