/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Dec 18, 2025
 *      Author: ggpai
 */

#include "stm32f4xx_spi_driver.h"
#include <stddef.h>

//Adding Definition for  API Prototypes for SPI peripherals

/*Peripheral clock control */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SPI_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

uint8_t getTXEBitStatus(SPI_RegDef_t *pSPIx)
{
	if(pSPIx->SPI_SR & (1 << SPI_SR_TXE))
	{
		return TXE_EMPTY;
	}
	else
	{
		return TXE_NOT_EMPTY;
	}
}

uint8_t getRXNEBitStatus(SPI_RegDef_t *pSPIx)
{
	if(pSPIx->SPI_SR & (1 << SPI_SR_RXNE))
	{
		return RXNE_NOT_EMPTY;
	}
	return RXNE_EMPTY;

}

uint8_t SPI_SR_BSY_Status(SPI_RegDef_t *pSPIx)
{
	if(pSPIx->SPI_SR & (SPI_BUSY_FLAG))
	{
		return SPI_BSY_BUSY;

	}
	return SPI_BSY_NOT_BUSY;

}



// interrupt handler implementations
static void spi_txeie_interrupt_handler(SPI_Handle_t *pSPIHandle){

		if((pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)))
		{
			 //DFF is 16 Bit
			pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer);
			(uint16_t*)pSPIHandle->pTxBuffer++;
			pSPIHandle->TxLen--;
		    pSPIHandle->TxLen--;

		}
		else
		{
			//DFF is 8 Bit
			pSPIHandle->pSPIx->SPI_DR = *(pSPIHandle->pTxBuffer);
			pSPIHandle->pTxBuffer++;
			pSPIHandle->TxLen--;

		}

        // close tranmission
		if(pSPIHandle->TxLen == 0)
		{

			//first disable the interrupt generating flag , TXEIE
		    SPI_CloseTransmission(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);

		}
}

static void spi_rxneie_interrupt_handler(SPI_Handle_t *pSPIHandle){

	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			*((uint16_t *)pSPIHandle->pRxBuffer)= pSPIHandle->pSPIx->SPI_DR;
			(uint16_t*)pSPIHandle->pRxBuffer++;
			pSPIHandle->RxLen--;
		    pSPIHandle->RxLen--;
		}
		else
		{
			// 8 bit DFF
			*(pSPIHandle->pRxBuffer)= pSPIHandle->pSPIx->SPI_DR;
			pSPIHandle->pRxBuffer++;
			pSPIHandle->RxLen--;
		   
		}
		 // close tranmission
		if(pSPIHandle->RxLen == 0)
		{

			//first disable the interrupt generating flag , TXEIE
            SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);

		}
}
static void spi_ovr_errie_interrupt_handler(SPI_Handle_t *pSPIHandle){


   //clear the ovr flag
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp =pSPIHandle->pSPIx->SPI_DR;
		temp =pSPIHandle->pSPIx->SPI_SR;
	}
   //inform the application , application will later take action (clear the OVR flag , close the transmission or reception)
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	// if over run error occurs in TX mode the application will be notified using the call back
	// and the application can explicitly clear the flags
	uint8_t temp;
	temp =pSPIx->SPI_DR;
	temp =pSPIx->SPI_SR;
	(void)temp;



}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen=0;
	pSPIHandle->TxState=SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen=0;
	pSPIHandle->RxState=SPI_READY;
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

	SPI_PeripheralClockControl(pSPIHandle->pSPIx, ENABLE);

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
		tempReg |=(1<<SPI_CR1_RX_ONLY);
	}

	//3.configure the SCLK speed in BR[2:0]
	tempReg |=(pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR3_5);
	//4.configure the DFF bit
	tempReg |= (pSPIHandle->SPIConfig.SPI_DFF<<SPI_CR1_DFF);
	//5. configure CPOL
	tempReg |= (pSPIHandle->SPIConfig.SPI_CPOL<<SPI_CR1_CPOL);
	//6. configure CPHA
	tempReg |= (pSPIHandle->SPIConfig.SPI_CPHA<<SPI_CR1_CPHA);
	//7. configure SSM
	tempReg |= (pSPIHandle->SPIConfig.SPI_SSM<<SPI_CR1_SSM);
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
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)  == FLAG_RESET ); // till the TXE is not empty keep hanging , only once empty push the new data
        // the above code is to ensure no overwriting of already existing data happens 
		if((pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)))
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
uint8_t SPI_SendDataWithIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t len){
   // we wont be doing data transmission here 
   // only storing buffer address and len info in global varaibles , that is handle structure
   // we will be enabling the TXEIE control bit to get interrupt when TXE flag is set in SR register
   // do all of this when SPI is not busy in TX transmission
	uint8_t state = pSPIHandle->TxState;

	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
    {
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//enable the TXEIE control bit ,this generates the interrupt whenever the TXE flag is set 
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);
   }
  
   return state;
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t len)
{
	while(len > 0)
	{

		// *pRxBuffer is the pointer to array where we store the incoming Data
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == (uint8_t)FLAG_RESET ); // till RX buffer is not full do not read
	    //above code is done to ensure new data everytime , not read empty rx buffer 
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			*((uint16_t *)pRxBuffer)= pSPIx->SPI_DR;
			(uint16_t*)pRxBuffer++;
			len--;
			len--;
		}
		else
		{
			// 8 bit DFF
			*(pRxBuffer)= pSPIx->SPI_DR;
			pRxBuffer++;
			len--;
			len--;
		}


	}
}
uint8_t SPI_ReceiveDataWithIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t len)	{
	   // we wont be doing data transmission here
	   // only storing buffer address and len info in global varaibles , that is handle structure
	   // we will be enabling the TXEIE control bit to get interrupt when TXE flag is set in SR register
	   // do all of this when SPI is not busy in TX transmission

		uint8_t state = pSPIHandle->RxState;

		if(pSPIHandle->RxState != SPI_BUSY_IN_RX)
	    {
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = len;
			pSPIHandle->RxState = SPI_BUSY_IN_RX;
			//enable the RXNEIE control bit to get the interrupt when RX buffer is full
			pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);
	   }

	   return state;

}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIX,uint8_t EnorDi){

	if(EnorDi ==ENABLE)
	{
		pSPIX->SPI_CR1 |=(1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIX->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
	}

}

void SPI_SSI_Config(SPI_RegDef_t *pSPIX,uint8_t EnorDi)
{
	//used to configure the SSI bit , in case of SSM = 1 and SSI 0 , in single master case this will cause NSS of master to ground
	//which will cause MODF fault and Reset the MSTR bit which will make master as slave
	if(EnorDi == ENABLE)
	{
		pSPIX->SPI_CR1|=(1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIX->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR2|=(1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->SPI_CR2|=~(1 << SPI_CR2_SSOE);
	}
}





/* Interrupt configuration and ISR handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi){
		// Note only around 96 IRQ are implemented
		// we can use just first three ISER and ICER registers
	    // writing zero in this register bit  has no effect
		if(EnorDi == ENABLE)
		{
			if(IRQNumber <=31)
			{
	           *NVIC_ISER0 |=(1 << IRQNumber);
			}
			else if(IRQNumber >=32 && IRQNumber <64)
			{
				*NVIC_ISER1 |=(1 << (IRQNumber % 32));

			}
			else if(IRQNumber >=64 && IRQNumber <96)
			{
				*NVIC_ISER2 |=(1 << (IRQNumber % 32));
			}
		}
		else
		{
			if(IRQNumber <=31)
			{
				*NVIC_ICER0 |=(1 << IRQNumber);
			}
			else if(IRQNumber >=32 && IRQNumber <64)
			{
				*NVIC_ICER1 |=(1 << (IRQNumber % 32));

			}
			else if(IRQNumber >=64 && IRQNumber <96)
			{
				*NVIC_ICER2 |=(1 << (IRQNumber % 32));

			}
		}

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){

	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	// in each of the section first 4 bits lower half is not implemented , only upper half is implemented
	uint8_t shift_amount = (8 *iprx_section) + (8 - NO_OF_BITS_IMPLEMENTED_IPR);

	*(NVIC_PRI_BASE_ADDR + iprx) |= (IRQPriority  << shift_amount);

}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	//this will be called by an ISR
	//similar mechanism to that of GPIO ISR , check the code

	//check first why the interrupt was called
	// 1. TXEIE or 2. RXNEIE or 3. some error (CRC or OVR)

	uint8_t temp1,temp2;
	temp1 = pSPIHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & ( 1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		// interrupt because of TXEIE bit
		spi_txeie_interrupt_handler(pSPIHandle);

	}


	temp1 = pSPIHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & ( 1 << SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{

		spi_rxneie_interrupt_handler(pSPIHandle);

	}
	temp1 = pSPIHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & ( 1 << SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{

		spi_ovr_errie_interrupt_handler(pSPIHandle);

	}

}


__attribute__ ((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEvent){

}
