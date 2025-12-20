/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: Dec 18, 2025
 *      Author: ggpai
 */

#ifndef INC_STM32F4XX_SPI_DRIVER_H_
#define INC_STM32F4XX_SPI_DRIVER_H_
#include "stm32f407xx.h"

typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

typedef struct{
	SPI_RegDef_t *pSPIx; //either SPI1 , SPI2 , SPI3
	SPI_Config_t SPIConfig;

}SPI_Handle_t;


/*
 * @SPI_DeviceMode  <-Refer the reference manual->
 */
#define SPI_DEVICE_MODE_SLAVE   0
#define SPI_DEVICE_MODE_MASTER 	1



/*
 * @SPI_BusConfig	<-Refer the reference manual->
 * */
#define SPI_BUS_CONFIG_FD   				1  //moslty working on this
#define SPI_BUS_CONFIG_HD					2
//#define SPI_BUS_CONFIG_SIMPLEX_TXONLY 		3 // we dont need this , its basically FD only with RX line disbaled
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 		3


/*
 * @SPI_SclkSpeed  <-Refer the reference manual->
 * */
#define SPI_SCLK_SPEED_DIV2 	0
#define SPI_SCLK_SPEED_DIV4 	1
#define SPI_SCLK_SPEED_DIV8 	2
#define SPI_SCLK_SPEED_DIV16 	3
#define SPI_SCLK_SPEED_DIV32 	4
#define SPI_SCLK_SPEED_DIV64 	5
#define SPI_SCLK_SPEED_DIV128 	6
#define SPI_SCLK_SPEED_DIV256 	7

/*
 * @SPI_DFF  <-Refer the reference manual->
 * */
#define SPI_DFF_8BITS  		0
#define SPI_DFF_16BITS  	1

/*
 * @SPI_CPOL  <-Refer the reference manual->
 * */
#define SPI_CPOL_HIGH 		0
#define SPI_CPOL_LOW		1

/*
 * @SPI_CPHA  <-Refer the reference manual->
 * */
#define SPI_CPHA_LOW		0
#define SPI_CPHA_HIGH		1

/*
 * @SPI_SSM  <-Refer the reference manual->
 * */
#define	SPI_SSM_EN		1
#define SPI_SSM_DI		0  // disabled by default


//Adding API Prototypes for SPI peripherals
/*Peripheral clock control */
void SPI_PeripheralClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

/*Init and De-Init*/
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*Send data and receive data */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t len);


/* Interrupt configuration and ISR handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * SPI_SR related Macros*/
#define TXE_NOT_EMPTY    	0
#define TXE_EMPTY		 	1

#define RXNE_EMPTY 				0
#define RXNE_NOT_EMPTY     		1

#endif /* INC_STM32F4XX_SPI_DRIVER_H_ */
