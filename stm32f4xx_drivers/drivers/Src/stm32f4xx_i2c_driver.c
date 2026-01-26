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
			if(pI2Cx==I2C1)
			{
				I2C1_PCLK_EN();
			}
			else if(pI2Cx==I2C2)
			{
				I2C2_PCLK_EN();
			}
			else if(pI2Cx==I2C3)
			{
				I2C3_PCLK_EN();
			}

		}else{
			if(pI2Cx==I2C1)
			{
				I2C1_PCLK_DI();
			}
			else if(pI2Cx==I2C2)
			{
				I2C2_PCLK_DI();
			}
			else if(pI2Cx==I2C3)
			{
				I2C3_PCLK_DI();
			}

		}
}

/*Init and De-Init*/
void I2C_Init(I2C_Handle_t *pI2CHandle){


	uint32_t tempReg=0;

	// configure the ACK bit in CR1 register
	tempReg|=(pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK);
    pI2CHandle->pI2Cx->I2C_CR1=tempReg;

	//configure the SCL speed , for this calculation we need to know the PCLK speed (it can be anything based on preScaler)
	//first configure the FREQ field in CR2 register
    tempReg=0;
    tempReg|=Cal_PCLK_Speed()/1000000;
    pI2CHandle->pI2Cx->I2C_CR2=tempReg;


    //program the Device own address
    tempReg=0;
    tempReg|=pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tempReg|= (1<<14); //SPECIFIED IN MANUAL

    pI2CHandle->pI2Cx->I2C_OAR1=tempReg;








}

uint32_t findClockSource()
{
	uint32_t temp;

	temp = ((RCC->CFGR >> 2) & 0x3);

	return temp;

}
uint32_t  Cal_PCLK_Speed()
{
	// clock source --> AHb1 prescaler --> APB1 prescaler ---> APB1 peripheral clocks

	// find the clock source from RCC_CFGR Regsiter
	uint32_t clckSource=0,SysClk=0,AHB_Prescaler=0, APB_Prescaler=0,temp=0,PCLK_SPEED;

	clckSource=findClockSource();

	if(clckSource==0)
	{
       sysClk=16000000;

	}
	else if(clckSource ==1)
	{
		sysClk=8000000;

	}
	else if(clckSource ==2)
	{
		sysClk=RCC_GetPLLOutputClock();

	}

	//need to find the preSclaer of AHB

	temp = ((RCC->CFGR >> 4) &  0xF);
	if(temp<8)
	{
		AHB_Prescaler=1;
	}
	else
	{
		AHB_Prescaler=AHB_PS[temp-8];
	}


	//need to find the Prescaler of APB
	temp = ((RCC->CFGR >> 13) &  0x7);
	if(temp < 4)
	{
		APB_Prescaler=1;
	}
	else
	{
		APB_Prescaler=APB_Prescaler[temp-4];
	}


	PCLK_SPEED=((SysClk/AHB_Prescaler)/APB_Prescaler);

	return PCLK_SPEED;




}
void I2C_DeInit(I2C_RegDef_t *pI2Cx){

		if(pI2Cx==I2C1)
		{
			I2C1_REG_RESET();
		}
		else if(pI2Cx==I2C2)
		{
			I2C2_REG_RESET();
		}
		else if(pI2Cx==I2C3)
		{
			I2C3_REG_RESET();
		}
}

/*I2C Peripheral enable API */
void I2C_PeripheralControl(I2C_RegDef_t *pI2CX,uint8_t EnorDi){

		if(EnorDi ==ENABLE)
		{
			pI2CX->I2C_CR1 |=(1 << I2C_CR1_PE);
		}
		else
		{
			pI2CX->I2C_CR1 &= ~(1 << I2C_CR1_PE);
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
	if(pI2Cx->I2C_SR1 & FlagName)
		{
			return FLAG_SET;
		}
		return FLAG_RESET;
}
