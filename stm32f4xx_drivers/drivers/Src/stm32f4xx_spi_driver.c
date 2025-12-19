/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Dec 18, 2025
 *      Author: ggpai
 */

#include "stm32f4xx_spi_driver.h"


//Adding Definition for  API Prototypes for SPI peripherals

/*Peripheral clock control */
void SPI_PeripheralClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi){

	if(EnorDi==ENABLE)
	{
		if(pSPIx==SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx==SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx==SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx==SPI4)
		{
			SPI4_PCLK_EN();
		}

	}else{
		if(pSPIx==SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx==SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx==SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx==SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

/*Init and De-Init*/
void SPI_Init(SPI_Handle_t *pSPIHandle){

}
void SPI_DeInit(SPI_RegDef_t *pSPIx){

}

/*Send data and receive data */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t len){

}
void SPI_RecieveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t len){

}


/* Interrupt configuration and ISR handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi){

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){

}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){

}
