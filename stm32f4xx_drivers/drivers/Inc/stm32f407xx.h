/*
 * stm32f407xx.h
 *
 *  Created on: Sep 18, 2025
 *      Author: LENOVO
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#define __vo                                volatile

/*Processor specific Details , get these details from cortex-m4 documentation */

/*NVIC Registers ISER and ICER register addresses*/

/* NVIC ISER*/
#define NVIC_ISER0   ((__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1   ((__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2   ((__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3   ((__vo uint32_t*)0xE000E10C )

/* NVIC ICER*/
#define NVIC_ICER0   ((__vo uint32_t*)0xE000E180 )
#define NVIC_ICER1   ((__vo uint32_t*)0xE000E184 )
#define NVIC_ICER2   ((__vo uint32_t*)0xE000E188 )
#define NVIC_ICER3   ((__vo uint32_t*)0xE000E18C )




/* NVIC Interrupt priority registers address calculation */
#define NVIC_PRI_BASE_ADDR  		((__vo uint32_t*)0xE000E400 )
#define NO_OF_BITS_IMPLEMENTED_IPR 		4


#define __vo                                volatile
#define FLASH_BASEADDR						0x08000000U
#define SRAM1_BASEADDR						0x20000000U
#define SRAM2_BASEADDR						0x2001C000U
#define ROM_BASEADDR						0x1FFF0000U
#define SRAM 								SRAM1_BASEADDR


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR 						0x40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR						0x40010000U
#define AHB1PERIPH_BASEADDR						0x40020000U
#define AHB2PERIPH_BASEADDR						0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus*/

#define GPIOA_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR                     (AHB1PERIPH_BASEADDR + 0x3800)



/*
 * Base addresses of peripherals which are hanging on APB1 bus*/
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus*/
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR        				(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)


#define RCC_BASEADDR 						(AHB1PERIPH_BASEADDR + 0x3800)

/******************Peripheral register structure Definitions ********************/

/*GPIO Peripheral Register Definition*/
typedef struct{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFRL;
	__vo uint32_t AFRH;
}GPIO_RegDef_t;

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR) /*typecasted values*/
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR) /*typecasted values*/
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR) /*typecasted values*/



// we need not create the below ones its redundant
/*
GPIO_RegDef_t *pGPIOA = GPIOA;
GPIO_RegDef_t *pGPIOB = GPIOB;
GPIO_RegDef_t *pGPIOC = GPIOC;
GPIO_RegDef_t *pGPIOD = GPIOD;
*/

/*RCC Peripheral Register Definition *
 *
 */
typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED1;
	__vo uint32_t RESERVED2;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED3;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t RESERVED5;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED6;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED7;
	__vo uint32_t RESERVED8;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED9;
	__vo uint32_t RESERVED10;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;

}RCC_RegDef_t;

#define RCC 			((RCC_RegDef_t*)RCC_BASEADDR)

/*
RCC_RegDef_t *pRCC = RCC;
*/

/*EXTI Peripheral Register Definition *
 *
 */

typedef struct{

	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;


#define EXTI            ((EXTI_RegDef_t*)EXTI_BASEADDR)

/*SYSCFG Peripheral Register Definition *
 * used to select the GPIO port
 */

typedef struct {
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	/*__vo uint32_t EXTICR1;
	__vo uint32_t EXTICR2;
	__vo uint32_t EXTICR3;
	__vo uint32_t EXTICR4;
	*/
	__vo uint32_t EXTICR[4];
	__vo uint32_t RESERVED1;
	__vo uint32_t RESERVED2;
	__vo uint32_t CMPCR;


}SYSCFG_RegDef_t;

#define SYSCFG         ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
/*define clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()			 (RCC->AHB1ENR |= ( 1 << 0 ))
#define GPIOB_PCLK_EN()			 (RCC->AHB1ENR |= ( 1 << 1 ))
#define GPIOC_PCLK_EN()			 (RCC->AHB1ENR |= ( 1 << 2 ))
#define GPIOD_PCLK_EN()			 (RCC->AHB1ENR |= ( 1 << 3 ))
#define GPIOE_PCLK_EN()			 (RCC->AHB1ENR |= ( 1 << 4 ))
#define GPIOF_PCLK_EN()			 (RCC->AHB1ENR |= ( 1 << 5 ))
#define GPIOG_PCLK_EN()			 (RCC->AHB1ENR |= ( 1 << 6 ))
#define GPIOH_PCLK_EN()			 (RCC->AHB1ENR |= ( 1 << 7 ))
#define GPIOI_PCLK_EN()			 (RCC->AHB1ENR |= ( 1 << 8 ))


/*define clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()			( RCC->APB1ENR |= ( 1<<21 ))
#define I2C2_PCLK_EN()			( RCC->APB1ENR |= ( 1<<22 ))
#define I2C3_PCLK_EN()			( RCC->APB1ENR |= ( 1<<23 ))

/*define clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()			( RCC->APB2ENR |= ( 1<<12 ))
#define SPI2_PCLK_EN()			( RCC->APB1ENR |= ( 1<<14 ))
#define SPI3_PCLK_EN()			( RCC->APB1ENR |= ( 1<<15 ))

/*define clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()			( RCC->APB2ENR |= ( 1<<4 ))
#define USART2_PCLK_EN()			( RCC->APB1ENR |= ( 1<<17 ))
#define USART3_PCLK_EN()			( RCC->APB1ENR |= ( 1<<18 ))
#define USART6_PCLK_EN()			( RCC->APB2ENR |= ( 1<<5 ))
/*define clock Enable Macros for UARTx peripherals
 */

#define UART4_PCLK_EN()			( RCC->AHB1ENR |= ( 1<<19 ))
#define UART5_PCLK_EN()			( RCC->AHB1ENR |= ( 1<<20 ))


//define clock enable macro for SYSCFG peripheral
#define SYSCFG_PCLK_EN()      ( RCC->APB1ENR |= ( 1<<14 ))



/*define clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()			( RCC->AHB1ENR &= ~( 1<<0 ))
#define GPIOB_PCLK_DI()			( RCC->AHB1ENR &= ~( 1<<1 ))
#define GPIOC_PCLK_DI()			( RCC->AHB1ENR &= ~( 1<<2 ))
#define GPIOD_PCLK_DI()			( RCC->AHB1ENR &= ~( 1<<3 ))
#define GPIOE_PCLK_DI()			( RCC->AHB1ENR &= ~( 1<<4 ))
#define GPIOF_PCLK_DI()			( RCC->AHB1ENR &= ~( 1<<5 ))
#define GPIOG_PCLK_DI()			( RCC->AHB1ENR &= ~( 1<<6 ))
#define GPIOH_PCLK_DI()			( RCC->AHB1ENR &= ~( 1<<7 ))
#define GPIOI_PCLK_DI()			( RCC->AHB1ENR &= ~( 1<<8 ))


/*define clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()			( RCC->APB1ENR &= ~( 1<<21 ))
#define I2C2_PCLK_DI()			( RCC->APB1ENR &= ~( 1<<22 ))
#define I2C3_PCLK_DI()			( RCC->APB1ENR &= ~( 1<<23 ))

/*define clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()			( RCC->APB2ENR &= ~( 1<<12 ))
#define SPI2_PCLK_DI()			( RCC->APB1ENR &= ~( 1<<14 ))
#define SPI3_PCLK_DI()			( RCC->APB1ENR &= ~( 1<<15 ))

/*define clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()			( RCC->APB2ENR &= ~( 1<<4 ))
#define USART2_PCLK_DI()			( RCC->APB1ENR &= ~( 1<<17 ))
#define USART3_PCLK_DI()			( RCC->APB1ENR &= ~( 1<<18 ))
#define USART6_PCLK_DI()			( RCC->APB2ENR &= ~( 1<<5 ))
/*define clock Disable Macros for UARTx peripherals
 */

#define UART4_PCLK_DI()			( RCC->AHB1ENR &= ~( 1<<19 ))
#define UART5_PCLK_DI()			( RCC->AHB1ENR &= ~( 1<<20 ))


#define ENABLE 				1
#define DISABLE     		0
#define SET 				ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET


#define GPIO_BASEADDR_TO_CODE(x)  		  ( (x==GPIOA)? 0 :\
											(x==GPIOB)? 1 :\
											(x==GPIOC)? 2 :\
											(x==GPIOD)? 3 :\
											(x==GPIOE)? 4 :\
											(x==GPIOF)? 5 :\
											(x==GPIOF)? 6 :\
											(x==GPIOG)? 7 :0 )


#define IRQ_NO_EXTI0  		6
#define IRQ_NO_EXTI1    	7
#define IRQ_NO_EXTI2  		8
#define IRQ_NO_EXTI3    	9
#define IRQ_NO_EXTI4  		10
#define IRQ_NO_EXTI9_5    	23
#define IRQ_NO_EXTI15_10	40




/*MAcros to reset the GPIOx peripherals */
//set it once and clear that again , if  left as 1 ... it will keep on resetting
#define GPIOA_REG_RESET()        do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR |= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()        do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR |= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()        do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR |= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()        do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR |= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()        do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR |= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()        do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR |= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()        do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR |= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()        do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR |= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()        do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR |= ~(1<<8));}while(0)

#include "stm32f4xx_gpio_driver.h"


#endif /* INC_STM32F407XX_H_ */



