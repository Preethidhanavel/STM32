

#ifndef INC_STM32L4XX_H_
#define INC_STM32L4XX_H_

#include<stdint.h>

#define __vo volatile
/*base address*/

#define FLASH_BASE_ADDR				0x08000000UL
#define SRAM1_BASE_ADDR				0x20000000UL
#define SRAM2_BASE_ADDR				0x10000000
#define ROM							0x1FFF0000
#define SRAM 						SRAM1_BASE_ADDR

/* peripheral bus base address*/
#define PERIPHER_BASE				 0x40000000UL
#define AHB1_BASE_ADDR				 0x40020000UL
#define AHB2_BASE_ADDR				 0x48000000UL
#define APB1_BASE_ADDR				 PERIPHER_BASE
#define APB2_BASE_ADDR				 0x40010000UL

/*peripheral base address on AHB2 bus*/
#define GPIOA_BASE_ADDR				 (AHB2_BASE_ADDR + 0X0000)
#define GPIOB_BASE_ADDR				 (AHB2_BASE_ADDR + 0X0400)
#define GPIOC_BASE_ADDR				 (AHB2_BASE_ADDR + 0X0800)
#define GPIOD_BASE_ADDR				 (AHB2_BASE_ADDR + 0X0C00)
#define GPIOE_BASE_ADDR				 (AHB2_BASE_ADDR + 0X1000)
#define GPIOF_BASE_ADDR				 (AHB2_BASE_ADDR + 0X1400)
#define GPIOG_BASE_ADDR				 (AHB2_BASE_ADDR + 0X1800)
#define GPIOH_BASE_ADDR				 (AHB2_BASE_ADDR + 0X1C00)

#define OTG_FS_BASE_ADDR			 0x50000000UL
#define ADC_BASE_ADDR				 0x50040000UL
#define AES_BASE_ADDR				 0x50060000UL
#define RNG_BASE_ADDR				 0x50060800UL

/*base address of peripheral on AHB1 bus*/
#define DMA1_BASE_ADDR				 (AHB1_BASE_ADDR + 0X0000)
#define DMA2_BASE_ADDR				 (AHB1_BASE_ADDR + 0X0400)
#define RCC_BASE_ADDR				 (AHB1_BASE_ADDR + 0X1000)
#define FLASHR_BASE_ADDR			 (AHB1_BASE_ADDR + 0X2000)
#define CRC_BASE_ADDR				 (AHB1_BASE_ADDR + 0X3000)
#define TSC_BASE_ADDR                (AHB1_BASE_ADDR + 0X4000)
/*end of AHB1  */

/*peripheral on APB1 bus */
#define TIM2_BASE_ADDR				 (APB1_BASE_ADDR + 0X0000)
#define TIM3_BASE_ADDR               (APB1_BASE_ADDR + 0X0400)
#define TIM4_BASE_ADDR               (APB1_BASE_ADDR + 0X0800)
#define TIM5_BASE_ADDR               (APB1_BASE_ADDR + 0X0C00)
#define TIM6_BASE_ADDR               (APB1_BASE_ADDR + 0X1000)
#define TIM7_BASE_ADDR               (APB1_BASE_ADDR + 0X1400)
#define LCD_BASE_ADDR                (APB1_BASE_ADDR + 0X2400)
#define RTC_BASE_ADDR                (APB1_BASE_ADDR + 0X2800)
#define WWDG_BASE_ADDR               (APB1_BASE_ADDR + 0X2C00)
#define IWDG_BASE_ADDR               (APB1_BASE_ADDR + 0X3000)
#define SPI2_BASE_ADDR               (APB1_BASE_ADDR + 0X3800)
#define SPI3_BASE_ADDR               (APB1_BASE_ADDR + 0X3C00)
#define USART2_BASE_ADDR             (APB1_BASE_ADDR + 0X4400)
#define USART3_BASE_ADDR             (APB1_BASE_ADDR + 0X4800)
#define UART4_BASE_ADDR              (APB1_BASE_ADDR + 0X4C00)
#define UART5_BASE_ADDR              (APB1_BASE_ADDR + 0X5000)
#define I2C1_BASE_ADDR               (APB1_BASE_ADDR + 0X5400)
#define I2C2_BASE_ADDR               (APB1_BASE_ADDR + 0X5800)
#define I2C3_BASE_ADDR               (APB1_BASE_ADDR + 0X5C00)
#define CAN1_BASE_ADDR               (APB1_BASE_ADDR + 0X6400)
#define PWR_BASE_ADDR                (APB1_BASE_ADDR + 0X7000)
#define DAC1_BASE_ADDR               (APB1_BASE_ADDR + 0X7400)
#define OPAMP_BASE_ADDR              (APB1_BASE_ADDR + 0X7800)
#define LPTIM1_BASE_ADDR             (APB1_BASE_ADDR + 0X7C00)
#define LPUART1_BASE_ADDR            (APB1_BASE_ADDR + 0X8000)
#define SWPMI1_BASE_ADDR             (APB1_BASE_ADDR + 0X8800)
#define LPTIM2_BASE_ADDR			 (APB1_BASE_ADDR + 0X9400)
/*end of  APB1*/

/* peripheral base address on APB2 bus */
#define SYSCFG_BASE_ADDR 			 (APB2_BASE_ADDR + 0X0000)
#define VREFBUF_BASE_ADDR 			 (APB2_BASE_ADDR + 0X0030)
#define COMP_BASE_ADDR 			 	 (APB2_BASE_ADDR + 0X0200)
#define EXTI_BASE_ADDR 			 	 (APB2_BASE_ADDR + 0X0400)
#define FIREWALL_BASE_ADDR 			 (APB2_BASE_ADDR + 0X1C00)
#define SDMMC1_BASE_ADDR 			 (APB2_BASE_ADDR + 0X2800)
#define TIM1_BASE_ADDR 			 	 (APB2_BASE_ADDR + 0X2C00)
#define SPI1_BASE_ADDR 			 	 (APB2_BASE_ADDR + 0X3000)
#define TIM8_BASE_ADDR 			 	 (APB2_BASE_ADDR + 0X3400)
#define USART1_BASE_ADDR 			 (APB2_BASE_ADDR + 0X3800)
#define TIM15_BASE_ADDR 			 (APB2_BASE_ADDR + 0X4000)
#define TIM16_BASE_ADDR 			 (APB2_BASE_ADDR + 0X4400)
#define TIM17_BASE_ADDR 			 (APB2_BASE_ADDR + 0X4800)
#define SAI1_BASE_ADDR 			 	 (APB2_BASE_ADDR + 0X5400)
#define SAI2_BASE_ADDR 			     (APB2_BASE_ADDR + 0X5800)
#define DFSDM1_BASE_ADDR 			 (APB2_BASE_ADDR + 0X6000)
/*end of  APB2 bus*/

/* GPIO structure definition  */
typedef struct{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
	__vo uint32_t BRR;
	__vo uint32_t ASCR;
}GPIO_Regdef_t;
/*end of definition */

/*RCC structure definition  */
typedef struct{
	__vo uint32_t CR;
	__vo uint32_t ICSCR;
	__vo uint32_t CFGR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t PLLSAI1CFGR;
	__vo uint32_t PLLSAI2CFGR;
	__vo uint32_t CIER;
	__vo uint32_t CIFR;
	__vo uint32_t CICR;
	__vo uint32_t RESERVED0;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED1;
	__vo uint32_t APB1RSTR1;
	__vo uint32_t APB1RSTR2;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED2;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED3;
	__vo uint32_t APB1ENR1;
	__vo uint32_t APB1ENR2;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t AHB1SMENR;
	__vo uint32_t AHB2SMENR;
	__vo uint32_t AHB3SMENR;
	__vo uint32_t RESERVED5;
	__vo uint32_t APB1SMENR1;
	__vo uint32_t APB1SMENR2;
	__vo uint32_t APB2SMENR;
	__vo uint32_t RESERVED6;
	__vo uint32_t CCIPR;
	__vo uint32_t RESERVED7;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t CRRCR;
	__vo uint32_t CCIPR2;
}RCC_Regdef_t;
/*end of definition */

/* structure definition of EXTI */
typedef struct{
	__vo uint32_t IMR1;
	__vo uint32_t EMR1;
	__vo uint32_t RTSR1;
	__vo uint32_t FTSR1;
	__vo uint32_t SWIER1;
	__vo uint32_t PR1;
	__vo uint32_t reserved[2];
	__vo uint32_t IMR2;
	__vo uint32_t EMR2;
	__vo uint32_t RTSR2;
	__vo uint32_t FTSR2;
	__vo uint32_t SWIER2;
	__vo uint32_t PR2;
}EXTI_RegDef_t;
/*end of EXTI defintion*/

/* syscfg definition */
typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t CFGR1;
	__vo uint32_t EXTICR[4];
	__vo uint32_t SCSR;
	__vo uint32_t CFGR2;
	__vo uint32_t SWPR;
	__vo uint32_t SKR;
	uint8_t reserved0;
	__vo uint32_t SWPR2;

}SYSCFG_RegDef_t;
/*GPIO peripheral definition*/
#define GPIOA 						((GPIO_Regdef_t *)GPIOA_BASE_ADDR)
#define GPIOB 						((GPIO_Regdef_t *)GPIOB_BASE_ADDR)
#define GPIOC 						((GPIO_Regdef_t *)GPIOC_BASE_ADDR)
#define GPIOD 						((GPIO_Regdef_t *)GPIOD_BASE_ADDR)
#define GPIOE 						((GPIO_Regdef_t *)GPIOE_BASE_ADDR)
#define GPIOF 						((GPIO_Regdef_t *)GPIOF_BASE_ADDR)
#define GPIOG 						((GPIO_Regdef_t *)GPIOG_BASE_ADDR)
#define GPIOH 						((GPIO_Regdef_t *)GPIOH_BASE_ADDR)
/* end of GPIO definition*/
/* RCC peripheral definition*/
#define RCC							((RCC_Regdef_t *)RCC_BASE_ADDR)
/*end of peripheral definiton*/
#define EXTI						((EXTI_RegDef_t *)EXTI_BASE_ADDR)
#define SYSCFG                      ((SYSCFG_RegDef_t *)SYSCFG_BASE_ADDR)
/*clock enable for GPIO peripheral*/
#define GPIOA_PCLK_EN()             (RCC -> AHB2ENR |= 1<<0)
#define GPIOB_PCLK_EN()             (RCC -> AHB2ENR |= 1<<1)
#define GPIOC_PCLK_EN()             (RCC -> AHB2ENR |= 1<<2)
#define GPIOD_PCLK_EN()             (RCC -> AHB2ENR |= 1<<3)
#define GPIOE_PCLK_EN()             (RCC -> AHB2ENR |= 1<<4)
#define GPIOF_PCLK_EN()             (RCC -> AHB2ENR |= 1<<5)
#define GPIOG_PCLK_EN()             (RCC -> AHB2ENR |= 1<<6)
#define GPIOH_PCLK_EN()             (RCC -> AHB2ENR |= 1<<7)
/*end of GPIO clk enable*/

/* Clock enable for I2C peripheral*/
#define I2C1_PCLK_EN()             (RCC -> APB1ENR  |= 1<<21)
#define I2C2_PCLK_EN()             (RCC -> APB1ENR1 |= 1<<22)
#define I2C3_PCLK_EN()             (RCC -> APB1ENR1 |= 1<<23)
/* end of I2C clk enable*/

/* clock enable for SPI peripheral*/
#define SPI1_PCLK_EN()             (RCC -> APB2ENR  |= 1<<12)
#define SPI2_PCLK_EN()             (RCC -> APB1ENR1 |= 1<<14)
#define SPI3_PCLK_EN()             (RCC -> APB1ENR1 |= 1<<15)
/*end of SPI clk enable*/

/*clock enable for UART peripheral */
#define USART1_PCLK_EN()       	   (RCC -> APB2ENR  |= 1<<14)
#define USART2_PCLK_EN()		   (RCC -> APB1ENR1 |= 1<<17)
#define USART3_PCLK_EN()		   (RCC -> APB1ENR1 |= 1<<18)
#define UART4_PCLK_EN()		       (RCC -> APB1ENR1 |= 1<<19)
#define UART5_PCLK_EN()		       (RCC -> APB1ENR1 |= 1<<20)
/*end of uart clk enable*/

/* clock enable for SYSCFG  */

#define SYSCFG_PCLK_EN()           (RCC -> APB2ENR |=1<<0)

/* end of SYSCFG clk enable*/

/*clock disable for GPIO peripheral*/
#define GPIOA_PCLK_DI()             (RCC -> AHB2ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()             (RCC -> AHB2ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()             (RCC -> AHB2ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()             (RCC -> AHB2ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()             (RCC -> AHB2ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()             (RCC -> AHB2ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()             (RCC -> AHB2ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()             (RCC -> AHB2ENR &= ~(1<<7))
/*end of GPIO clk disable*/

/*peripheral clokc reset */
#define GPIOA_REG_RESET()			do{(RCC -> AHB2RSTR |= (1<<0)); RCC -> AHB2RSTR &= ~(1<<0);}while(0)
#define GPIOB_REG_RESET()			do{(RCC -> AHB2RSTR |= (1<<1)); RCC -> AHB2RSTR &= ~(1<<1);}while(0)
#define GPIOC_REG_RESET()			do{(RCC -> AHB2RSTR |= (1<<2)); RCC -> AHB2RSTR &= ~(1<<2);}while(0)
#define GPIOD_REG_RESET()			do{(RCC -> AHB2RSTR |= (1<<3)); RCC -> AHB2RSTR &= ~(1<<3);}while(0)
#define GPIOE_REG_RESET()			do{(RCC -> AHB2RSTR |= (1<<4)); RCC -> AHB2RSTR &= ~(1<<4);}while(0)
#define GPIOF_REG_RESET()			do{(RCC -> AHB2RSTR |= (1<<5)); RCC -> AHB2RSTR &= ~(1<<5);}while(0)
#define GPIOG_REG_RESET()			do{(RCC -> AHB2RSTR |= (1<<6)); RCC -> AHB2RSTR &= ~(1<<6);}while(0)
#define GPIOH_REG_RESET()			do{(RCC -> AHB2RSTR |= (1<<7)); RCC -> AHB2RSTR &= ~(1<<7);}while(0)
/*end of pclk reset */
#define GPIO_BASEADDR_TO_CODE(x)    (x==GPIOA) ?0 :\
									(x==GPIOB) ?1 :\
									(x==GPIOC) ?2 :\
									(x==GPIOD) ?3 :\
									(x==GPIOE) ?4 :\
									(x==GPIOF) ?5 :\
									(x==GPIOG) ?6 :\
									(x==GPIOH) ?7 :0

/* Clock disable for I2C peripheral*/
#define I2C1_PCLK_DI()             (RCC -> APB1ENR  &= ~(1<<21))
#define I2C2_PCLK_DI()             (RCC -> APB1ENR1 &= ~(1<<22))
#define I2C3_PCLK_DI()             (RCC -> APB1ENR1 &= ~(1<<23))
/* end of I2C clk disable*/

/* clock disable for SPI peripheral*/
#define SPI1_PCLK_DI()             (RCC -> APB2ENR  &= ~(1<<12))
#define SPI2_PCLK_DI()             (RCC -> APB1ENR1 &= ~(1<<14))
#define SPI3_PCLK_DI()             (RCC -> APB1ENR1 &= ~(1<<15))
/*end of SPI clk disable*/

/*clock disable for UART peripheral */
#define USART1_PCLK_DI()       	   (RCC -> APB2ENR  &= ~(1<<14))
#define USART2_PCLK_DI()		   (RCC -> APB1ENR1 &= ~(1<<17))
#define USART3_PCLK_DI()		   (RCC -> APB1ENR1 &= ~(1<<18))
#define UART4_PCLK_DI()		       (RCC -> APB1ENR1 &= ~(1<<19))
#define UART5_PCLK_DI()		       (RCC -> APB1ENR1 &= ~(1<<20))
/*end of uart clk disable*/

/* clock disable for SYSCFG  */

#define SYSCFG_PCLK_DI()           (RCC -> APB2ENR &=~(1<<0))

/* end of SYSCFG clk disable*/

/*macros for enable or disable*/
#define ENABLE         1
#define DISABLE        0
#define SET            ENABLE
#define RESET          DISABLE
#define GPIO_PIN_SET   SET
#define GPIO_PIN_RESET RESET

#define IRQ_NO_EXTI0   6
#define IRQ_NO_EXTI1   7
#define IRQ_NO_EXTI2   8
#define IRQ_NO_EXTI3   9
#define IRQ_NO_EXTI4   10
#define IRQ_NO_EXTI9_5   23
#define IRQ_NO_EXTI15_10   40


#define NVIC_ISER0     ( (__vo uint32_t *) 0xE000E100)
#define NVIC_ISER1     ( (__vo uint32_t *) 0xE000E104)
#define NVIC_ISER2     ( (__vo uint32_t *) 0xE000E108)
#define NVIC_ISER3     ( (__vo uint32_t *) 0xE000E10C)
#define NVIC_ISER4     ( (__vo uint32_t *) 0xE000E110)
#define NVIC_ISER5     ( (__vo uint32_t *) 0xE000E114)


#define NVIC_ICER0     ( (__vo uint32_t *) 0XE000E180)
#define NVIC_ICER1     ( (__vo uint32_t *) 0xE000E184)
#define NVIC_ICER2     ( (__vo uint32_t *) 0xE000E188)
#define NVIC_ICER3     ( (__vo uint32_t *) 0xE000E18C)
#define NVIC_ICER4     ( (__vo uint32_t *) 0xE000E190)
#define NVIC_ICER5     ( (__vo uint32_t *) 0xE000E194)

#define NVIC_PR_BASE_ADDR  ( (__vo uint32_t *)  0xE000E400)

#define NO_PR_BITS_IMPLEMENTED   4


#endif /* INC_STM32L4XX_H_ */
