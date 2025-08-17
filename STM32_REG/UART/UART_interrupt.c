#include <stdio.h>
#include <string.h>
#include "stm32l4xx.h"
#include "stm32l4xx_usart_driver.h"


// Message to send continuously over UART
char msg[30] = "UART_INTERRUPT";

// USART2 handle
USART_Handle_t usart2_handle;

// Variables for interrupt-driven RX
volatile uint8_t rxCmplt = RESET;     // Flag to indicate reception complete
volatile uint32_t rx_index = 0;       // Index for storing received data
char rx_buf[1024];                    // Buffer to store received data

uint8_t g_data = 0;                   // Temporary variable for received data

extern void initialise_monitor_handles();  // For printf over SWO/debug

/*---------------------------------------------------
 * USART2 Initialization
 *---------------------------------------------------*/
void USART2_Init(void)
{
    usart2_handle.pUSARTx = USART2;
    usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;  // 115200 baud
    usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;        // TX and RX
    usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
    USART_Init(&usart2_handle);  // Initialize USART2 peripheral
}

/*---------------------------------------------------
 * GPIO Initialization for USART2
 * PA2 -> TX, PA3 -> RX
 *---------------------------------------------------*/
void USART2_GPIOInit(void)
{
    GPIO_Handle_t usart_gpios;

    usart_gpios.pGPIOx = GPIOA;
    usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;      // Alternate function
    usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;     // Push-pull
    usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;   // Pull-up
    usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;     // Fast speed
    usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode = 7;              // AF7 for USART2

    // Configure TX pin PA2
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_PeriClockControl(GPIOA, ENABLE);  // Enable GPIOA clock
    GPIO_Init(&usart_gpios);

    // Configure RX pin PA3
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_Init(&usart_gpios);
}

/*---------------------------------------------------
 * GPIO Initialization for Button and LED
 * PC13 -> Button, PA5 -> LED
 *---------------------------------------------------*/
void GPIO_ButtonInit(void)
{
    GPIO_Handle_t GPIOBtn, GpioLed;

    // Button input
    GPIOBtn.pGPIOx = GPIOC;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN; 
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_PeriClockControl(GPIOC, ENABLE);
    GPIO_Init(&GPIOBtn);

    // LED output
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_Init(&GpioLed);
}

/*---------------------------------------------------
 * Simple delay function
 *---------------------------------------------------*/
void delay(void)
{
    for(uint32_t i = 0 ; i < 250000 ; i++);
}

/*---------------------------------------------------
 * Main function
 *---------------------------------------------------*/
int main(void)
{
    // Initialize SWO/debug for printf
    initialise_monitor_handles();

    // Initialize UART and GPIO
    USART2_GPIOInit();
    USART2_Init();
    GPIO_ButtonInit();

    // Enable USART2 interrupts
    USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);

    // Enable USART2 peripheral
    USART_PeripheralControl(USART2, ENABLE);

    printf("Application is running\n");

    while(1)
    {
        delay();

        // Send message continuously
        USART_SendData(&usart2_handle, (uint8_t*)msg, strlen(msg));

        // Start interrupt-based reception of 1 byte
        USART_ReceiveDataIT(&usart2_handle, (uint8_t *)&rx_buf[rx_index], 1);

        // Wait for reception complete
        while (1)
        {
            if (rxCmplt == SET)
            {
                printf("You typed: %s\n", rx_buf);   // Print received string
                rxCmplt = RESET;                     // Reset flag
                USART_ReceiveDataIT(&usart2_handle, (uint8_t *)&rx_buf[rx_index], 1); // Restart reception
            }
        }
    }
    return 0;
}

/*---------------------------------------------------
 * USART2 IRQ Handler
 *---------------------------------------------------*/
void USART2_IRQHandler(void)
{
    USART_IRQHandling(&usart2_handle);  // Handle USART interrupts
}

/*---------------------------------------------------
 * USART application callback function
 * Handles RX complete event
 *---------------------------------------------------*/
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApEv)
{
    if (ApEv == USART_EVENT_RX_CMPLT)
    {
        // Print received character
        printf("%c ", rx_buf[rx_index]);

        // If received character is 'a', toggle LED
        if(rx_buf[rx_index] == 'a')
        {
            GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
            delay();
        }

        // Echo back received character
        USART_SendData(pUSARTHandle, (uint8_t *)&rx_buf[rx_index], 1);

        // Check for end of string (Enter key)
        if (rx_buf[rx_index] == '\r' || rx_buf[rx_index] == '\n')
        {
            rx_buf[rx_index] = '\0';  // terminate string
            rxCmplt = SET;            // set reception complete flag
            rx_index = 0;             // reset buffer index
        }
        else
        {
            rx_index++;   // move to next buffer index
            // Continue interrupt-based reception
            USART_ReceiveDataIT(pUSARTHandle, (uint8_t *)&rx_buf[rx_index], 1);
        }
    }
}
