#include "stm32l4xx.h"

// Global millisecond counter incremented in SysTick_Handler
static volatile uint32_t g_ms = 0;

// RCC AHB2 peripheral enable register: bit0 enables GPIOA clock
#define RCC_AHB2ENR_GPIOAEN 			(1U << 0)



// Initialize PA5
static void gpio_led_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;   // Enable GPIOA peripheral clock

    // Configure PA5 as general-purpose output
    GPIOA->MODER &= ~(3U << (5U * 2U));    // Clear mode bits
    GPIOA->MODER |=  (1U << (5U * 2U));    // 01 = output mode

    GPIOA->OTYPER &= ~(1U << 5);           // Push-pull output
    GPIOA->OSPEEDR &= ~(3U << (5U * 2U));  // Low speed
    GPIOA->PUPDR &= ~(3U << (5U * 2U));    // No pull-up/pull-down
}


// Initialize SysTick for 1ms tick
static void systick_init_1ms(void)
{
    uint32_t reload = (4000000 / 1000UL) - 1UL; // Assuming clock - 1ms
    SysTick->RVR  = reload;   // Reload value
    SysTick->CVR   = 0;       // Clear current value register

    // SysTick interrupt priority function
     GPIO_PRIORITY_CONFIG(-1, 0X0F);

    // Enable SysTick with:
    // - Core clock as source
    // - Interrupt enabled
    // - Counter enabled
    SysTick->CSR = (1<<2)| (1<<1)| (1<<0);

}


// SysTick interrupt handler (runs every 1ms)
void SysTick_Handler(void)
{
    g_ms++;   // Increment millisecond counter
}


// delay function
static void delay_ms(uint32_t ms)
{
    uint32_t start = g_ms; // Capture start time
    while ((g_ms - start) < ms) {
        for (volatile int i = 0; i < 1; i++); // Small wait (prevents compiler optimizations)
    }
}


// Main program
int main(void)
{
    gpio_led_init();       // Configure LED pin
    systick_init_1ms();    // Configure SysTick timer

    while (1)
    {
        GPIOA->ODR ^= (1U << 5); // Toggle PA5 (LED)
        delay_ms(500);           // Delay 500ms
    }
}
