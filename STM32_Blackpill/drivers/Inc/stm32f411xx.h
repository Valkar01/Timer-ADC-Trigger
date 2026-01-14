/*
 * stm32f411xx.h
 *
 *  Created on: 06-Jan-2026
 *      Author: Karthik
 */

/************************************** PROCESSOR SPECIFIC DETAILS ********************************************/
/*
 * ARM M4 Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0                        ((volatile uint32_t *)0XE000E100)
#define NVIC_ISER1                        ((volatile uint32_t *)0XE000E104)
#define NVIC_ISER2                        ((volatile uint32_t *)0XE000E108)
#define NVIC_ISER3                        ((volatile uint32_t *)0XE000E10C)

#define NVIC_ICER0                        ((volatile uint32_t *)0XE000E180)
#define NVIC_ICER1                        ((volatile uint32_t *)0XE000E184)
#define NVIC_ICER2                        ((volatile uint32_t *)0XE000E188)
#define NVIC_ICER3                        ((volatile uint32_t *)0XE000E18C)

#define NVIC_PR_BASE_ADDR                 ((volatile uint32_t *)0xE000E400)
/**************************************************************************************************************/

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include<stdint.h>
#include<stddef.h>
#include<string.h>
/*
 * Base addresses of SRAMS and FLASH
 */
#define FLASH_BASE_ADDR                  0x08000000U       // Base address for the Flash Memory
#define SRAM_BASE_ADDR                   0x20000000U       // Base address for the SRAM Memory
#define ROM_BASE_ADDR                    0x1FFF0000U       // Base address for the ROM Memory

/*
 * Base addresses for peripheral buses AHBx and APBx
 */
#define PERIPH_BASE_ADDR                  0x40000000U       // Base address for the peripherals
#define	APB1PERIPH_BASE_ADDR              PERIPH_BASE_ADDR  // Base address for the peripherals on APB1 Bus
#define	APB2PERIPH_BASE_ADDR              0x40010000U       // Base address for the peripherals on APB2 Bus
#define	AHB1PERIPH_BASE_ADDR              0x40020000U       // Base address for the peripherals on AHB1 Bus
#define	AHB2PERIPH_BASE_ADDR              0x50000000U       // Base address for the peripherals on AHB2 Bus

/*
 * Base addresses for the peripherals on AHB1 bus
 */
#define GPIOA_BASE_ADDR                   (AHB1PERIPH_BASE_ADDR + 0x0000)  // Base address for each GPIOA pin
#define GPIOB_BASE_ADDR                   (AHB1PERIPH_BASE_ADDR + 0x0400)  // Base address for each GPIOB pin
#define GPIOC_BASE_ADDR                   (AHB1PERIPH_BASE_ADDR + 0x0800)  // Base address for each GPIOC pin
#define GPIOD_BASE_ADDR                   (AHB1PERIPH_BASE_ADDR + 0x0C00)  // Base address for each GPIOD pin
#define GPIOE_BASE_ADDR                   (AHB1PERIPH_BASE_ADDR + 0x1000)  // Base address for each GPIOE pin
#define GPIOH_BASE_ADDR                   (AHB1PERIPH_BASE_ADDR + 0x1C00)  // Base address for each GPIOH pin
#define RCC_BASE_ADDR                     (AHB1PERIPH_BASE_ADDR + 0x3800)  // Base address for RCC
#define DMA1_BASE_ADDR                    (AHB1PERIPH_BASE_ADDR + 0x6000)  // Base address for DMA1
#define DMA2_BASE_ADDR					  (AHB1PERIPH_BASE_ADDR + 0x6400)  // Base address for DMA2

/*
 * Base addresses for the peripherals on APB2 bus
 */
#define USART1_BASE_ADDR				  (APB2PERIPH_BASE_ADDR + 1000)    // Base address of USART1
#define ADC1_BASE_ADDR					  (APB2PERIPH_BASE_ADDR + 2000)    // Base address of ADC1

/*
 * Base addresses for the peripherals on APB1 bus
 */
#define TIM2_BASE_ADDR					  (APB1PERIPH_BASE_ADDR	+ 0000)	   // Base address of Timer2

/****** Individual peripheral registers ******/

typedef struct
{
	volatile uint32_t MODER ;       // GPIOx Mode Register to configure the mode
	volatile uint32_t OTYPER ;     // GPIOx Output type Register to configure type of output
	volatile uint32_t OSPEEDR ;    // GPIOx Output speed Register to configure the speed of output
	volatile uint32_t PUPDR ;      // GPIOx PullUp PullDown Register to select the pullup or pulldown resistor
	volatile uint32_t IDR ;        // GPIOx InputData Register
	volatile uint32_t ODR ;        // GPIOx OutputData Register
	volatile uint32_t BSRR ;       // GPIOx Port Bit Set/Reset Register
	volatile uint32_t LCKR ;       // GPIOx Port Configuration Lock Register
	volatile uint32_t AFR[2] ;     // GPIOx Alternate Function Register where AFR[0] is Alternate Function Low and AFR[1] is Alternate Function High registers respectively
}GPIO_Reg_Def_t;

typedef struct
{
	volatile uint32_t CR ;            // RCC Clock Control Register for enabling and controlling the clocks
	volatile uint32_t PLLCFGR ;       // PLL Configuration Register to configure PLL
	volatile uint32_t CFGR ;          // RCC Clock Configure Register to select different clocks
	volatile uint32_t CIR ;           // Clock Interrupt Register
	volatile uint32_t AHB1RSTR ;      // AHB1 Reset Register to reset the peripherals on AHB1 bus
	volatile uint32_t AHB2RSTR ;      // AHB2 Reset Register to reset the peripherals on AHB2 bus
	volatile uint32_t RESERVED0[2] ;
	volatile uint32_t APB1RSTR ;      // APB1 Reset Register to reset the peripherals on APB1 bus
	volatile uint32_t APB2RSTR ;      // APB2 Reset Register to reset the peripherals on APB2 bus
	volatile uint32_t RESERVED[2] ;
	volatile uint32_t AHB1ENR ;       // AHB1 Peripheral Clock Enable Register to enable or disable the peripherals on AHB1 bus
	volatile uint32_t AHB2ENR ;       // AHB1 Peripheral Clock Enable Register to enable or disable the peripherals on AHB2 bus
	volatile uint32_t RESERVED2[2] ;
	volatile uint32_t APB1ENR ;       // APB1 Peripheral Clock Enable Register to enable or disable the peripherals on APB1 bus
	volatile uint32_t APB2ENR ;       // APB2 Peripheral Clock Enable Register to enable or disable the peripherals on APB2 bus
	volatile uint32_t RESERVED3[2] ;
	volatile uint32_t AHB1LPENR ;     // AHB1 Peripheral Clock Enable Register in Low Power Mode to enable or disable the peripherals on AHB1 bus in sleep mode
	volatile uint32_t AHB2LPENR ;     // AHB2 Peripheral Clock Enable Register in Low Power Mode to enable or disable the peripherals on AHB2 bus in sleep mode
	volatile uint32_t RESERVED4[2] ;
	volatile uint32_t APB1LPENR ;     // APB1 Peripheral Clock Enable Register in Low Power Mode to enable or disable the peripherals on APB1 bus in sleep mode
	volatile uint32_t APB2LPENR ;     // APB2 Peripheral Clock Enable Register in Low Power Mode to enable or disable the peripherals on APB2 bus in sleep mode
	volatile uint32_t RESERVED5[2] ;
	volatile uint32_t BDCR ;          // Backup Domain Control Register
	volatile uint32_t CSR ;           // Control & Status Register
	volatile uint32_t RESERVED6[2] ;
	volatile uint32_t SSCGR ;         // Spread Spectrum Clock Generation Register
	volatile uint32_t PLLI2SCFGR ;    // PLLI2S configuration register
	volatile uint32_t RESERVED7 ;
	volatile uint32_t DCKCFGR ;		  // Dedicated
}RCC_Reg_Def_t;

typedef struct
{
	volatile uint32_t LISR;
	volatile uint32_t HISR;
	volatile uint32_t LIFCR;
	volatile uint32_t HIFCR;
	volatile uint32_t S0CR;
	volatile uint32_t S0NDTR;
	volatile uint32_t S0PAR;
	volatile uint32_t S0M0AR;
	volatile uint32_t S0M1AR;
	volatile uint32_t S0FCR;
}DMA_Reg_Def_t;

typedef struct
{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
}USART_Reg_Def_t;

typedef struct
{
	volatile uint32_t SR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMPR[2];
	volatile uint32_t JOFR[4];
	volatile uint32_t HTR;
	volatile uint32_t LTR;
	volatile uint32_t SQR[3];
	volatile uint32_t JSQR;
	volatile uint32_t JDR[4];
	volatile uint32_t DR;
}ADC_Reg_Def_t;

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	volatile uint32_t RESERVED;
	volatile uint32_t CCR[4];
	volatile uint32_t RESERVED1;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;
}Timer_2to5_Reg_Def_t;

/*
 * Typecasting the Peripheral Base addresses with xxx_Reg_Def_t structure
 */
#define GPIOA                  ((GPIO_Reg_Def_t*)GPIOA_BASE_ADDR)
#define RCC                    ((RCC_Reg_Def_t*)RCC_BASE_ADDR)
#define DMA2  				   ((DMA_Reg_Def_t*)DMA2_BASE_ADDR)
#define USART1 				   ((USART_Reg_Def_t*)USART1_BASE_ADDR)
#define ADC1				   ((ADC_Reg_Def_t*)ADC1_BASE_ADDR)
#define TIM2				   ((Timer_2to5_Reg_Def_t*)TIM2_BASE_ADDR)

/*
 * Clock enable for AHB1 Peripherals
 */
#define GPIOA_PCLK_EN          (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN          (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN          (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN          (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN          (RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN          (RCC->AHB1ENR |= (1 << 7))
#define DMA2_PCLK_EN           (RCC->AHB1ENR |= (1 << 22))

/*
 * Clock enable for APB2 Peripherals
 */
#define USART1_PCLK_EN	       (RCC->APB2ENR |= (1 << 4))
#define ADC1_PCLK_EN		   (RCC->APB2ENR |= (1 << 8))

/*
 * Clock enable for APB1 Peripherals
 */
#define TIM2_PCLK_EN		   (RCC->APB1ENR |= (1 << 0))

/*
 * Clock disable for AHB1 Peripherals
 */
#define GPIOA_PCLK_DI          (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI          (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI          (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI          (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI          (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI          (RCC->AHB1ENR &= ~(1 << 7))
#define DMA2_PCLK_DI           (RCC->AHB1ENR &= ~(1 << 22))

/*
 * Clock disable for APB2 Peripherals
 */
#define USART1_PCLK_DI	       (RCC->APB2ENR &= ~(1 << 4))
#define ADC1_PCLK_DI		   (RCC->APB2ENR &= ~(1 << 8))

/*
 * Clock enable for APB1 Peripherals
 */
#define TIM2_PCLK_DI		   (RCC->APB1ENR &= ~(1 << 0))

/*
 * GENERAL MACROS
 */
#define ENABLE                  1
#define DISABLE                 0
#define SET                     ENABLE
#define RESET                   DISABLE
#define GPIO_PIN_RESET          DISABLE
#define GPIO_PIN_SET            ENABLE
#define NO_PR_BITS_IMPLEMENTED  4
#define FLAG_SET                SET
#define FLAG_RESET              RESET

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

#include"stm32f411xx_GPIO.h"
#include"stm32f411xx_Timer.h"
#include"stm32f411xx_ADC.h"
#include"stm32f411xx_DMA.h"
#include"stm32f411xx_USART.h"

#endif /* INC_STM32F411XX_H_ */
