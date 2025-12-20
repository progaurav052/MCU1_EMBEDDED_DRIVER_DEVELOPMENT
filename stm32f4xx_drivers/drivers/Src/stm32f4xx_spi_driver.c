/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Dec 18, 2025
 *      Author: ggpai
 */

#include "stm32f4xx_spi_driver.h"


//Adding Definition for  API Prototypes for SPI peripherals

/*Peripheral clock control */

uint8_t getTXEBitStatus(SPI_RegDef_t *pSPIx)
{
	if(pSPIx->SPI_SR & (1 << 1))
	{
		return TXE_EMPTY;
	}
	else
	{
		return TXE_NOT_EMPTY;
	}
}
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

/* Initialization and De-Init*/
void SPI_Init(SPI_Handle_t *pSPIHandle){
	// configure the CR1 register
	// 1. Configure the device Mode :
	// Method is to build the Temp-register and than do an OR with the actual Peripheral register
	uint32_t tempReg = 0;
	tempReg|=(pSPIHandle->SPIConfig.SPI_DeviceMode <<2);

	//2.configure the BUS configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_FD)
	{
		tempReg &= ~(1<<SPI_CR1_BIDI_MODE);

	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_HD)
	{
		tempReg |=(1<<SPI_CR1_BIDI_MODE);

	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// for both TX and RX mode,the connection is like 2 line only , we only need not connect 1 line
		// clear the BIDI mode first
		tempReg &= ~(1<<SPI_CR1_BIDI_MODE);
		// set the RX only bit
		tempReg |=(1<<SPI_CR1_BIDI_OE);
	}

	//3.configure the SCLK speed in BR[2:0]
	tempReg |=(pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR3_5);
	//4.configure the DFF bit
	tempReg |= (1<<SPI_CR1_DFF);
	//5. configure CPOL
	tempReg |= (1<<SPI_CR1_CPOL);
	//6. configure CPHA
	tempReg |= (1<<SPI_CR1_CPHA);
	//7. configure SSM
	tempReg |= (1<<SPI_CR1_SSM);
	// save the tempReg to actual CR register
	pSPIHandle->pSPIx->SPI_CR1 = tempReg; // fresh value so can use = operator



}

void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx==SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx==SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx==SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx==SPI4)
	{
		SPI4_REG_RESET();
	}

}

/*Send data and receive data */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t len){
	//find the related algorithm for this from the net
	//Blocking API - Polling method
	while(len > 0) // in Bytes
	{
		// check if TX Buffer is empty , only than load the data into the DR (use the SPI_SR register for this)
		while(getTXEBitStatus(pSPIx) == TXE_NOT_EMPTY); // till the TXE is not empty keep hanging

		// we are here indicates : ready to load Data to DR,TX empty
		if(pSPIx->SPI_CR1 & (1 << 11))
		{
			 //DFF is 16 Bit
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);

			(uint16_t*)pTxBuffer++;
			len--;
			len--;

		}
		else
		{
			//DFF is 8 Bit
			pSPIx->SPI_DR = *(pTxBuffer);
			pTxBuffer++;
			len --;

		}

	}

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
