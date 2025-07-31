/*
 * stm32l4xx.h
 *
 *  Created on: Jul 31, 2025
 *      Author: VINOTH
 */

#ifndef INC_STM32L4XX_H_
#define INC_STM32L4XX_H_

#include<stdint.h>

#define __vo volatile
/*base address*/

#define FLASH_BASE_ADDR				0x0800 0000UL
#define SRAM1_BASE_ADDR				0x2000 0000UL
#define SRAM2_BASE_ADDR				0x1000 0000
#define ROM							0x1FFF 0000
#define SRAM 						SRAM1_BASE_ADDR

/* peripheral bus base address*/
#define PERIPHER_BASE				 0x4000 0000UL
#define AHB1_BASE_ADDR				 0x4002 0000UL
#define AHB2_BASE_ADDR				 0x4800 0000UL
#define APB1_BASE_ADDR				 PERIPHER_BASE
#define APB2_BASE_ADDR				 0x4001 0000UL

/*peripheral base address on AHB2 bus*/
#define GPIOA_BASE_ADDR				 (AHB2_BASE_ADDR + 0X0000)
#define GPIOB_BASE_ADDR				 (AHB2_BASE_ADDR + 0X0400)
#define GPIOC_BASE_ADDR				 (AHB2_BASE_ADDR + 0X0800)
#define GPIOD_BASE_ADDR				 (AHB2_BASE_ADDR + 0X0C00)
#define GPIOE_BASE_ADDR				 (AHB2_BASE_ADDR + 0X1000)
#define GPIOF_BASE_ADDR				 (AHB2_BASE_ADDR + 0X1400)
#define GPIOG_BASE_ADDR				 (AHB2_BASE_ADDR + 0X1800)
#define GPIOH_BASE_ADDR				 (AHB2_BASE_ADDR + 0X1C00)

#define OTG_FS_BASE_ADDR			 0x5000 0000UL
#define ADC_BASE_ADDR				 0x5004 0000UL
#define AES_BASE_ADDR				 0x5006 0000UL
#define RNG_BASE_ADDR				 0x5006 0800UL

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

/*clock enable for GPIO peripheral*/
#define GPIOA_PCLK_EN()             (RCC -> AHB2ENR | = 1<<0)
#define GPIOB_PCLK_EN()             (RCC -> AHB2ENR | = 1<<1)
#define GPIOC_PCLK_EN()             (RCC -> AHB2ENR | = 1<<2)
#define GPIOD_PCLK_EN()             (RCC -> AHB2ENR | = 1<<3)
#define GPIOE_PCLK_EN()             (RCC -> AHB2ENR | = 1<<4)
#define GPIOF_PCLK_EN()             (RCC -> AHB2ENR | = 1<<5)
#define GPIOG_PCLK_EN()             (RCC -> AHB2ENR | = 1<<6)
#define GPIOH_PCLK_EN()             (RCC -> AHB2ENR | = 1<<7)
/*end of GPIO clk enable*/

/* Clock enable for I2C peripheral*/
#define I2C1_PCLK_EN()             (RCC -> APB1ENR  | = 1<<21)
#define I2C2_PCLK_EN()             (RCC -> APB1ENR1 | = 1<<22)
#define I2C3_PCLK_EN()             (RCC -> APB1ENR1 | = 1<<23)
/* end of I2C clk enable*/

/* clock enable for SPI peripheral*/
#define SPI1_PCLK_EN()             (RCC -> APB2ENR  | = 1<<12)
#define SPI2_PCLK_EN()             (RCC -> APB1ENR1 | = 1<<14)
#define SPI3_PCLK_EN()             (RCC -> APB1ENR1 | = 1<<15)
/*end of SPI clk enable*/

/*clock enable for UART peripheral */
#define USART1_PCLK_EN()       	   (RCC -> APB2ENR  | = 1<<14)
#define USART2_PCLK_EN()		   (RCC -> APB1ENR1 | = 1<<17)
#define USART3_PCLK_EN()		   (RCC -> APB1ENR1 | = 1<<18)
#define UART4_PCLK_EN()		       (RCC -> APB1ENR1 | = 1<<19)
#define UART5_PCLK_EN()		       (RCC -> APB1ENR1 | = 1<<20)
/*end of uart clk enable*/

/* clock enable for SYSCFG  */

#define SYSCFG_PCLK_EN()           (RCC -> APB2ENR | =1<<0)

/* end of SYSCFG clk enable*/

/*clock disable for GPIO peripheral*/
#define GPIOA_PCLK_DI()             (RCC -> AHB2ENR & = ~(1<<0))
#define GPIOB_PCLK_DI()             (RCC -> AHB2ENR & = ~(1<<1))
#define GPIOC_PCLK_DI()             (RCC -> AHB2ENR & = ~(1<<2))
#define GPIOD_PCLK_DI()             (RCC -> AHB2ENR & = ~(1<<3))
#define GPIOE_PCLK_DI()             (RCC -> AHB2ENR & = ~(1<<4))
#define GPIOF_PCLK_DI()             (RCC -> AHB2ENR & = ~(1<<5))
#define GPIOG_PCLK_DI()             (RCC -> AHB2ENR & = ~(1<<6))
#define GPIOH_PCLK_DI()             (RCC -> AHB2ENR & = ~(1<<7))
/*end of GPIO clk disable*/

/* Clock disable for I2C peripheral*/
#define I2C1_PCLK_DI()             (RCC -> APB1ENR  & = ~(1<<21))
#define I2C2_PCLK_DI()             (RCC -> APB1ENR1 & = ~(1<<22))
#define I2C3_PCLK_DI()             (RCC -> APB1ENR1 & = ~(1<<23))
/* end of I2C clk disable*/

/* clock disable for SPI peripheral*/
#define SPI1_PCLK_DI()             (RCC -> APB2ENR  & = ~(1<<12))
#define SPI2_PCLK_DI()             (RCC -> APB1ENR1 & = ~(1<<14))
#define SPI3_PCLK_DI()             (RCC -> APB1ENR1 & = ~(1<<15))
/*end of SPI clk disable*/

/*clock disable for UART peripheral */
#define USART1_PCLK_DI()       	   (RCC -> APB2ENR  & = ~(1<<14))
#define USART2_PCLK_DI()		   (RCC -> APB1ENR1 & = ~(1<<17))
#define USART3_PCLK_DI()		   (RCC -> APB1ENR1 & = ~(1<<18))
#define UART4_PCLK_DI()		       (RCC -> APB1ENR1 & = ~(1<<19))
#define UART5_PCLK_DI()		       (RCC -> APB1ENR1 & = ~(1<<20))
/*end of uart clk disable*/

/* clock disable for SYSCFG  */

#define SYSCFG_PCLK_DI()           (RCC -> APB2ENR & =~(1<<0))

/* end of SYSCFG clk disable*/



#endif /* INC_STM32L4XX_H_ */
