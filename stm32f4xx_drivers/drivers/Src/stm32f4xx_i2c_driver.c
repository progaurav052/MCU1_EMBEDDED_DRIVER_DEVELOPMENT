/*
 * stm32f4xx_i2c_driver.c
 *
 *  Created on: Jan 26, 2026
 *      Author: ggpai
 */


#include "stm32f4xx_i2c_driver.h"

static uint32_t findClockSource()
{
	uint32_t temp;

	temp = ((RCC->CFGR >> 2) & 0x3);

	return temp;

}
static uint32_t  get_PCLK_Speed()
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

static void I2C_generateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1  |= (1 << I2C_CR1_START);

}

static void I2C_clearADDRFlag(I2C_RegDef_t *pI2Cx){

	pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_ADDR);

}

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
    tempReg|=get_PCLK_Speed()/1000000;
    pI2CHandle->pI2Cx->I2C_CR2=(tempReg & 0x3F);


    //program the Device own address
    tempReg=0;
    tempReg|=pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tempReg|= (1<<14); //SPECIFIED IN MANUAL
    pI2CHandle->pI2Cx->I2C_OAR1=tempReg;


    //configure the CCR register,configure the CCR field
    tempReg=0;
    uint32_t ccr_value=0;
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <=I2C_SCL_SPEED_SM)
    {
    	//configure the mode bit 0 by default for STD mode
    	//configure the CCR value in CCR field
    	//we assume in std mode tHigh == tLow
    	ccr_value=(get_PCLK_Speed() / (2* pI2CHandle->I2C_Config.I2C_SCLSpeed));
    	tempReg |=(ccr_value &0xFFF);


    }
    else
    {
    	//configure the mode as fast mode 15th bit is 1
    	tempReg |=(1 <<I2C_CCR_FS);

    	//configure the duty cycle as well , give by the application
    	tempReg |=(pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);

    	//based on the duty cycle the tHigh and tLow changes
    	if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
    	{
    		ccr_value=(get_PCLK_Speed() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
    	}
    	else
    	{
    		ccr_value=(get_PCLK_Speed() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
    	}
    	//configure the CCR value in CCR field
    	tempReg |=(ccr_value &0xFFF);

    }
    pI2CHandle->pI2Cx->I2C_CCR=tempReg;

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

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint8_t Len,uint8_t SlaveAddr)
{
	//1.Generate start condition
	//create an small helper function for this  private to driver file
	I2C_generateStartCondition(pI2CHandle->pI2Cx);

	//2.confirm that start condition is generated by checking the SB flag in SR1
	// Note : Until SB is cleared SCL will be stretched (pulled to Low)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));
	// will be cleared by reading SR1 and writing to DR

	//3.send the address of the slave with r/2 bit set to w(0) (8 bits in total)
	SlaveAddr << 1;
	SlaveAddr &=~(1);
	pI2CHandle->pI2Cx->I2C_DR |=SlaveAddr;

	//4.confirm that address phase is completed by reading the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)); // if set as 1 ADDR is ack proceed to transmit  by clearing the ADDR bit


	//5. clear the ADDR bit in SR1
	// Note : until this ADDR is not cleared SCL will be stretched (pulled to Low)
	I2C_clearADDRFlag(pI2CHandle->pI2Cx);



}
