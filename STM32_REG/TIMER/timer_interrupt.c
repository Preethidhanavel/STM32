#include "stm32l4xx.h"

#define RCC_AHB2ENR_GPIOAEN    (1U << 0)   // Enable GPIOA clock
#define RCC_APB1ENR1_TIM2EN    (1U << 0)   // Enable TIM2 clock
#define TIM_CR1_CEN   (0x1U)
#define TIM_DIER_UIE   (0x1U)
#define TIM_SR_UIF   (0x1U)

static void gpio_led_init(void)
{
    // Enable GPIOA clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // Configure PA5 as output (01)
    GPIOA->MODER &= ~(3U << (5U * 2U));
    GPIOA->MODER |=  (1U << (5U * 2U));
}

static void tim2_init(void)
{
    // Enable TIM2 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // TIM2 runs at APB1 clock
    TIM2->PSC = 4000 - 1;   // Prescaler
    TIM2->ARR = 500 - 1;    // Auto-reload

    TIM2->DIER |= TIM_DIER_UIE;  // Enable update interrupt
    TIM2->CR1  |= TIM_CR1_CEN;   // Enable TIM2 counter

    // Enable TIM2 IRQ in NVIC
   //GPIO_PRIORITY_CONFIG(28,0Xf);
   GPIO_IRQInterruptConfig(28, ENABLE);
}

// TIM2 Interrupt Service Routine
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)   // Check update interrupt flag
    {
        TIM2->SR &= ~TIM_SR_UIF; // Clear flag

        // Toggle LED (PA5)
        GPIOA->ODR ^= (1U << 5);
    }
}

int main(void)
{
    gpio_led_init();
    tim2_init();

    while (1)
    {
        // Main loop does nothing, LED toggles in ISR
    }
}
