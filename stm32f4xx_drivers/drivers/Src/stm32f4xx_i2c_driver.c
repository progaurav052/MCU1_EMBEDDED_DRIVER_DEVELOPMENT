/*
 * stm32f4xx_i2c_driver.c
 *
 *  Created on: Jan 26, 2026
 *      Author: ggpai
 */


#include "stm32f4xx_i2c_driver.h"




static void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//implement the code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 <<I2C_CR2_ITBUFEN);

	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 <<I2C_CR2_ITEVFEN);
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->RxSize =0;

	if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
	// we have to maintain the state to what it was before transmission;


}
static void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//implement the code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 <<I2C_CR2_ITBUFEN);

	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 <<I2C_CR2_ITEVFEN);

	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
	pI2CHandle->TxRxState = I2C_READY;




}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1  |= (1 << I2C_CR1_START);

}
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr){
		SlaveAddr=SlaveAddr << 1;
		SlaveAddr &=~(1);
		pI2Cx->I2C_DR =SlaveAddr;
}
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr){
		SlaveAddr=SlaveAddr << 1;
		SlaveAddr |=1;
		pI2Cx->I2C_DR =SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){

	// the same code can we used for Non Interrupt mode as well
	// in the case when SendData / receiveData API (non interrupt mode is used) ... than STATE will be in I2C_Ready by default so else part is executed
	uint32_t dummyRead;
	if(pI2CHandle->pI2Cx->I2C_SR2 & (1<<I2C_SR2_MSL)) //@why this condition is important
	{
		if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize ==1)
			{
				//Diable the acking
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);
				//clear the ADDR flag
				dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
				dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
				(void) dummyRead;

			}
		}
		else
		{
			dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
			dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
			(void) dummyRead;
		}

	}

}
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_STOP);
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

	I2C_PeripheralClockControl(pI2CHandle->pI2Cx, ENABLE);


	// configure the ACK bit in CR1 register
	tempReg|=(pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK);
    pI2CHandle->pI2Cx->I2C_CR1 |=tempReg;

	//configure the SCL speed , for this calculation we need to know the PCLK speed (it can be anything based on preScaler)
	//first configure the FREQ field in CR2 register
    tempReg=0;
    tempReg|=RCC_GetPCLK1Value()/1000000;
    pI2CHandle->pI2Cx->I2C_CR2|=(tempReg & 0x3F);


    //program the Device own address
    tempReg=0;
    tempReg|=pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tempReg|= (1<<14); //SPECIFIED IN MANUAL
    pI2CHandle->pI2Cx->I2C_OAR1|=tempReg;


    //configure the CCR register,configure the CCR field
    tempReg=0;
    uint32_t ccr_value=0;
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <=I2C_SCL_SPEED_SM)
    {
    	//configure the mode bit 0 by default for STD mode
    	//configure the CCR value in CCR field
    	//we assume in std mode tHigh == tLow
    	ccr_value=(RCC_GetPCLK1Value() / (2* pI2CHandle->I2C_Config.I2C_SCLSpeed));
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
    		ccr_value=(RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
    	}
    	else
    	{
    		ccr_value=(RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
    	}
    	//configure the CCR value in CCR field
    	tempReg |=(ccr_value &0xFFF);

    }
    pI2CHandle->pI2Cx->I2C_CCR|=tempReg;



    //TRise configuration
    tempReg=0;
    //need to find maximum allowed rise time for SCL in SM mode and fast mode
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed==I2C_SCL_SPEED_SM)
    {
    	//standard mode , Trise is 1000ns
    	tempReg = ((RCC_GetPCLK1Value()/1000000U)+1);

    }
    else{
    	//fast mode  , Trise is 300ns
    	tempReg=((RCC_GetPCLK1Value()*300)/1000000000U)+1;

    }
    pI2CHandle->pI2Cx->I2C_TRISE|=tempReg;





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


//Adding an function to check the flags ,repository code
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName){
	if(pI2Cx->I2C_SR1 & FlagName)
		{
			return FLAG_SET;
		}
		return FLAG_RESET;


}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		pI2Cx->I2C_CR1 |= (I2C_ACK_ENABLE << I2C_CR1_ACK);

	}
	else
	{
		pI2Cx->I2C_CR1 &= ~(I2C_ACK_ENABLE << I2C_CR1_ACK);
	}
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint8_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	//1.Generate start condition
	//create an small helper function for this  private to driver file
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2.confirm that start condition is generated by checking the SB flag in SR1
	// Note : Until SB is cleared SCL will be stretched (pulled to Low),cleared by reading SR1 and writing Data in DR which is done in ExecuteAdddresPhase
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));
	// will be cleared by reading SR1 and writing to DR

	//3.send the address of the slave with r/2 bit set to w(0) (8 bits in total)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);


	//4.confirm that address phase is completed by reading the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)); // if set as 1 ADDR is ack proceed to transmit  by clearing the ADDR bit


	//5. clear the ADDR bit in SR1 , cleared by reading SR1 and SR2
	// Note : until this ADDR is not cleared SCL will be stretched (pulled to Low)
	I2C_ClearADDRFlag(pI2CHandle);

	//6.send the data until Length becomes 0
	while(Len > 0)
	{
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) ); //Wait till TXE is set
			pI2CHandle->pI2Cx->I2C_DR = *pTxBuffer;
			pTxBuffer++;
			Len--;
	}

	//7. when Length becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
		//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
		//   when BTF=1 SCL will be stretched (pulled to LOW)

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) );
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF) );


	//8. Generate stop condition , when executed the BTF is automatically cleared

	if(Sr == I2C_DISABLE_SR )
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


}
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	//1.generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2.confirm that start condition is generated by checking the SB flag in SR1
	// Note : Until SB is cleared SCL will be stretched (pulled to Low),cleared by reading SR1 and writing Data in DR which is done in ExecuteAdddresPhase
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//3.Send the address of the slave with r/w bit ,we are doing read so this bit is 1
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4. wait until the address phase is completed by checking the ADDR flag is set
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));
	// procedure if to read only 1 byte from slave , while clearing the ADDR bit .. we do Disable ACK , STOP bit set .. and than finally clear the ADDR


	if(Len ==1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);


		//clear the ADDR flag .. so the clock stretching is released
		I2C_ClearADDRFlag(pI2CHandle);

		// wait until RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );
		// read Data in to buffer
		*pRxBuffer =pI2CHandle->pI2Cx->I2C_DR;
		Len--;


		//Generate the stop condition
		if(Sr == I2C_DISABLE_SR )
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx); // for one byte of Data this code position is important , else before data is come to SR ... bUs will be released;



	}
	else if (Len > 1)
	{

		// use an for loop implementation here , cause we have to handle the case when reading last 2 byte
		I2C_ClearADDRFlag(pI2CHandle);

		for(uint32_t i=Len;i>0;i--)
		{
			// wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) ); // wait until RXNE becomes 1 for the 2nd last byte , when this happens parallely last 1 byte  data is coming to SR
			//sequence mentioned in reference manual 
			if(i==2)
			{
				//special Handling
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE); // reading 2nd last byte , but in SR Last byte is getting transferred

				if(Sr == I2C_DISABLE_SR )
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

				// the above steps to be followed are mentioned in the reference manual

			}
			*pRxBuffer =pI2CHandle->pI2Cx->I2C_DR;
			pRxBuffer++;

		}

	}

	//re-enable acking , if Acking was enabled before coming to this API , we need to maintain that

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}

}
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint8_t Len,uint8_t SlaveAddr,uint8_t Sr){
		uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) // since I2C is an half duplex , at a time it can be only read or write
		{
			pI2CHandle->pTxBuffer = pTxBuffer;
			pI2CHandle->TxLen =Len;
			pI2CHandle->TxRxState =I2C_BUSY_IN_TX;
			pI2CHandle->DevAddr =SlaveAddr;
			pI2CHandle->Sr =Sr;

			//Implement code to Generate START Condition

			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVFEN);


			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;

}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len,uint8_t SlaveAddr,uint8_t Sr){
		uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pRxBuffer = pI2CHandle->pTxBuffer;
			pI2CHandle->RxLen =Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
			pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition

			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVFEN);

			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
		}

		return busystate;
}

//Slave Mode API :
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx,uint8_t data){
	pI2Cx->I2C_DR=data;

}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->I2C_DR;
}

void I2C_SlaveModeEnableDisableInterruptControlBits(I2C_RegDef_t *pI2Cx,uint8_t EnorDi){
	if(EnorDi == ENABLE)
	{
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVFEN);

		//Implement the code to enable ITERREN Control Bit
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
	}
	else
	{

		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);


		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVFEN);


		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}


//EVENT ISR Handler
// when the interrupt is triggered , ISR is called , that ISR calls the handler ... where the servicing is done
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	//Interrupt handling for both master and slave mode of a device

	uint32_t temp1,temp2,temp3;

	temp1 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITEVFEN);
	temp2 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITBUFEN); // gate for txe and RXNE

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_SB);
	if (temp1 && temp3) {
		//SB flag is set , ITR is generated , SB phase is successful, cleared by reading the SR1 and DR , SR1 reading is done above
		// Execute the address phase
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);

		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);

		}
	}

	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3)
	{
		//ADDR flag is set , ADDR phase is successful
		//cleared by reading SR1 and SR2;

		I2C_ClearADDRFlag(pI2CHandle);


	}

	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_BTF);
	if (temp1 && temp3) {
		// BTF flag is set
		// BTF flag prevents underrun and overrun, helps in closing the transmission
		// when TXE =1 and BTF =1 , we can close the transmission
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			if(pI2CHandle->pI2Cx->I2C_SR1 & (1 <<I2C_SR1_TXE))
			{
				if(pI2CHandle->TxLen == 0)
				{
					// TXE set , BTF set
					//1. Generate stop condition
					if (pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2. reset all the member elements of the handle structure
					// handle structure should be cleared
					I2C_CloseSendData(pI2CHandle);

					//3. notify the application about tranmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}


			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
				// we have got nothing to do,
				//we do not use BTF to close Rx reception

		}
		

	}

	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set , will only get executed in slave mode
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_STOPF);
	if (temp1 && temp3) {
		//STOPF flag is set
		//clear the stopf (i.e read Sr1 , write to Cr1)

		pI2CHandle->pI2Cx->I2C_CR1 |= 0x0;

		//notify the apllication on stop detection
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOPF);


	}
	//5. Handle For interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TXE);
	if (temp1 && temp2 && temp3) {

		//all these are done when the device is  master , need to check this
		if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{
			// TXE flag is set , we have to do transmission.. by loading the data
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){  //@why this condition is necessary here
				if (pI2CHandle->TxLen > 0) {
					//1. load the data in DR
					pI2CHandle->pI2Cx->I2C_DR = *pI2CHandle->pTxBuffer;
					//2. Decrement the length
					pI2CHandle->TxLen--;
					//3. increment the buffer address
					pI2CHandle->pTxBuffer++;

				}
			}

		}
		else
		{
			//device is in slave mode :
			// same logic as above check if the device is in receive mode
			if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA)) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}



	}

	//6. Handle For interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_RXNE);
	if (temp1 && temp2 && temp3) {

		// RXNE flag is set, we have to do Rx in this case
		// Need to handle the case of Len 1 byte and length 2 bytes
		if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL)) {
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) { // @why this condition is important
				if (pI2CHandle->RxSize == 1) {
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
					pI2CHandle->RxLen--;

				} else if (pI2CHandle->RxSize > 1) {
					if (pI2CHandle->RxLen == 2) {
						I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
					}
					//read DR
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
					pI2CHandle->pRxBuffer++;
					pI2CHandle->RxLen--;
				}
				if (pI2CHandle->RxLen == 0) { // after last reading code will come here
					//close the Rx and notify the application
					//1. generate stop condition
					if (pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					//2. close I2C Rx , reassign the handle structure
					//2. reset all the member elements of the handle structure
					// handle structure should be cleared
					I2C_CloseReceiveData(pI2CHandle);

					//3. notify the application
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);

				}
			}

		}
		else
		{
			//device is in slave mode :
			// same logic as above check if the device is in receive mode
			if(!(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}



	}

}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}

__attribute__ ((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEvent){
 // weak implementation
 // code is called by the application , usually in  caseof event completion
}
