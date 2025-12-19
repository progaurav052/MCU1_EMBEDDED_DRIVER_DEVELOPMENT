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

#endif /* INC_STM32F4XX_SPI_DRIVER_H_ */
