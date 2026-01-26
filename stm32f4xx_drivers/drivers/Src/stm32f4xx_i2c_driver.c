/*
 * stm32f4xx_i2c_driver.c
 *
 *  Created on: Jan 26, 2026
 *      Author: ggpai
 */


#include "stm32f4xx_i2c_driver.h"

void I2C_PeripheralClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi){
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

		}
}

/*Init and De-Init*/
void I2C_Init(I2C_Handle_t *pI2CHandle){

}
void I2C_DeInit(I2C_RegDef_t *pI2Cx){

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
}

/*I2C Peripheral enable API */
void I2C_PeripheralControl(I2C_RegDef_t *pI2CX,uint8_t EnorDi){

		if(EnorDi ==ENABLE)
		{
			pSPIX->SPI_CR1 |=(1 << SPI_CR1_SPE);
		}
		else
		{
			pSPIX->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
		}

}

/* Interrupt configuration and ISR handling */
void I2C_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi){
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){

		uint8_t iprx = IRQNumber/4;
		uint8_t iprx_section = IRQNumber%4;
		// in each of the section first 4 bits lower half is not implemented , only upper half is implemented
		uint8_t shift_amount = (8 *iprx_section) + (8 - NO_OF_BITS_IMPLEMENTED_IPR);

		*(NVIC_PRI_BASE_ADDR + iprx) |= (IRQPriority  << shift_amount);

}


//Adding an function to check the flags ,repo code
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName){
	if(pSPIx->SPI_SR & FlagName)
		{
			return FLAG_SET;
		}
		return FLAG_RESET;
}
