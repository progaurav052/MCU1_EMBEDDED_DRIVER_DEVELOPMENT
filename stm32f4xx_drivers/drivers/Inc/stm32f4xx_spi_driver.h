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
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t TxState;
	uint8_t RxState;

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
#define SPI_CPOL_HIGH 		1
#define SPI_CPOL_LOW		0

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


/*SPI Possible application state*/
#define SPI_READY 			0
#define SPI_BUSY_IN_TX		2
#define SPI_BUSY_IN_RX      1


//Adding API Prototypes for SPI peripherals
/*Peripheral clock control */
void SPI_PeripheralClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

/*Init and De-Init*/
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*Send data and receive data */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t len);

/*SPI Peripheral enable API */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIX,uint8_t EnorDi);

/*
 * SPI SSI configuration*/
void SPI_SSI_Config(SPI_RegDef_t *pSPIX,uint8_t EnorDi);

/* SPI SSOE configuration*/
void SPI_SSOE_Config(SPI_RegDef_t *pSPIX,uint8_t EnorDi);

/* SPI_SR_BSY status checking */
uint8_t SPI_SR_BSY_Status(SPI_RegDef_t *pSPIx);

/* Interrupt configuration and ISR handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

//Adding an function to check the flags ,repo code
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);

//interrupt modes for SPI_SendData and receive Data
uint8_t SPI_SendDataWithIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t len); // we wont use any loops here
uint8_t SPI_ReceiveDataWithIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t len); // we wont use any loops here

/*
 * SPI_CR1 register Bit Position Definition Macros*/
#define SPI_CR1_CPHA			  0
#define SPI_CR1_CPOL			  1
#define SPI_CR1_MSTR			  2
#define SPI_CR1_BR3_5			  3
#define SPI_CR1_SPE				  6
#define SPI_CR1_LSB_FIRST		  7
#define SPI_CR1_SSI				  8
#define SPI_CR1_SSM				  9
#define SPI_CR1_RX_ONLY			  10
#define SPI_CR1_DFF				  11
#define SPI_CR1_CRC_NEXT		  12
#define SPI_CR1_CRC_EN			  13
#define SPI_CR1_BIDI_OE		      14
#define SPI_CR1_BIDI_MODE	      15


/*
 * SPI_CR2 register Bit Position Definition Macros*/
#define SPI_CR2_RXDMAEN			  	0
#define SPI_CR2_TXDMAEN			  	1
#define SPI_CR2_SSOE			  	2
#define SPI_CR2_RES_1			  	3
#define SPI_CR2_FRF				  	4
#define SPI_CR2_ERRIE		  	5
#define SPI_CR2_RXNEIE			  	6
#define SPI_CR2_TXEIE			  	7
#define SPI_CR2_RES_2			  	8


/*
 * SPI_SR register Bit Position Definition Macros*/
#define SPI_SR_RXNE			  	0
#define SPI_SR_TXE			  	1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR			  	3
#define SPI_SR_CRC_ERR			4
#define SPI_SR_MODF		  		5
#define SPI_SR_OVR			  	6
#define SPI_SR_BSY			  	7
#define SPI_SR_FRE			  	8

/*
 * SPI_SR related Macros*/
#define TXE_NOT_EMPTY    	0
#define TXE_EMPTY		 	1

#define RXNE_EMPTY 				0
#define RXNE_NOT_EMPTY     		1

#define SPI_BSY_BUSY     			1
#define SPI_BSY_NOT_BUSY    		0


/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG    ( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   ( 1 << SPI_SR_BSY)

#endif /* INC_STM32F4XX_SPI_DRIVER_H_ */
