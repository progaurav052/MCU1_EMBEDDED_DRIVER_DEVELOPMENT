/*
 * I2C_MasterReceiveDataIT.c
 *
 *  Created on: Feb 15, 2026
 *      Author: ggpai
 */


/*
 * I2C_MasterReceive_data.c
 *
 *  Created on: Feb 9, 2026
 *      Author: ggpai
 */

/*
 * i2c_Master_semd_data.c
 *
 *  Created on: Jan 31, 2026
 *      Author: ggpai
 */
#include "stm32f407xx.h"
#include <string.h>
#include <stddef.h>




#define HIGH  1
#define LOW   0
#define BTN_PRESSED 	LOW

#define SLAVE_ADDR  	0x68
#define MY_ADDR   SLAVE_ADDR

I2C_Handle_t I2C1_Handle;

uint8_t TxBuffer[32]="STM32 Slave Mode Testing";

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

// refer the alternate functionality from datasheet to get SCL and SDA Gpio pins for I2C1
// AF4 will be used
void I2C1_GpioInits(){

	GPIO_Handle_t I2C1_pins;

	// we want PB6 as SCL , PB9 as SDA
	I2C1_pins.pGPIOx=GPIOB;
	I2C1_pins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	I2C1_pins.GPIO_PinConfig.GPIO_PinAltFunMode=GPIO_ALTFN_4;
	I2C1_pins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	I2C1_pins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD; // we need OD configuration for SDA and SCL line , we use PU resistors
	I2C1_pins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU; // , refer schema

	//SCL line
	I2C1_pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_6;
	GPIO_Init(&I2C1_pins);

	//SDA line
	I2C1_pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_9;
	GPIO_Init(&I2C1_pins);



}

void I2C1_Inits(){


	I2C1_Handle.pI2Cx=I2C1;
	I2C1_Handle.I2C_Config.I2C_AckControl=I2C_ACK_ENABLE;
	I2C1_Handle.I2C_Config.I2C_FMDutyCycle=I2C_FM_DUTY_2; // does not matter we are using Standard mode for this example
	I2C1_Handle.I2C_Config.I2C_DeviceAddress=MY_ADDR; // if slave this is where we store slave address in OAR
	I2C1_Handle.I2C_Config.I2C_SCLSpeed=I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1_Handle);




}

// we will use internal push button
// when button pressed ,send Data
void GPIO_ButtonInit()
{
	GPIO_Handle_t GPIOButton; // user wakeup button connected to PA0 , internal button

	//config for Button - PA0; = input mode
	GPIOButton.pGPIOx=GPIOA;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD; // internal button , refer the Schema doc for this config

	// Enbale the RCC clock

	GPIO_PeripheralClockControl(GPIOButton.pGPIOx, ENABLE);

	//initialize the config
	GPIO_Init(&GPIOButton);

}

int main()
{


	GPIO_ButtonInit();

	//I2C pin init

	I2C1_GpioInits();

	//I2C peripheral init

	I2C1_Inits();
	//I2C IRQ configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE); // interrupts are enabled , ISER is enabled.
	// since in this application our Device is acting as slave, we need to enable the Interrupt control bits , only when these bits are enabled interrupts will be generated from the peripheral

	I2C_SlaveModeEnableDisableInterruptControlBits(I2C1,ENABLE);
	//after all the configuration is done enable the I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ManageAcking(I2C1_Handle.pI2Cx,ENABLE);


	while(1); // hangs in infinite while loop
	//whenever an event happens example TXE set or RXNE set .. interrupt will be triggered and ISR will be called which will in turn call the applicationEventCallBack






}

void I2C1_EV_IRQHandler (void)
{
	I2C_EV_IRQHandling(&I2C1_Handle);
}


void I2C1_ER_IRQHandler (void)
{
	I2C_ER_IRQHandling(&I2C1_Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{

	static uint8_t commandCode = 0;
	static  uint8_t Cnt = 0;

	if(AppEv == I2C_EV_DATA_REQ)
	{
		//Master wants some data. slave has to send it
		if(commandCode == 0x51)
		{
			//send the length information to the master
			I2C_SlaveSendData(pI2CHandle->pI2Cx,strlen((char*)TxBuffer));
		}else if (commandCode == 0x52)
		{
			//Send the contents of Tx_buf
			I2C_SlaveSendData(pI2CHandle->pI2Cx,TxBuffer[Cnt++]);

		}
	}else if (AppEv == I2C_EV_DATA_RCV)
	{
		//Data is waiting for the slave to read . slave has to read it
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
		// when master initially sends command code 0x51 ... during this after the address phase is completed the TRA of slave is 0 , when RXNE becomes 1 ISR is serviced and commandCode value is stored -->
		//....ISR exits , TXE ISR starts executing as TXE is set ... ||ly on master  MasterRecieve is executed .. when the address phase is completed the TRA os master is set to 0 and TRA of slave is set to 1
		//.... when this happens master executed SendData code ... hence its ensured that communication is in sync.



	}else if (AppEv == I2C_ERROR_AF)
	{
		//This happens only during slave txing .
		//Master has sent the NACK. so slave should understand that master doesnt need
		//more data.
		commandCode = 0xff;
		Cnt = 0;
	}
	else if (AppEv == I2C_EV_STOPF)
	{
		//This happens only during slave reception .
		//Master has ended the I2C communication with the slave.
	}

}
/*
 * I2C_slaveProgrammingIT.c
 *
 *  Created on: Feb 24, 2026
 *      Author: ggpai
 */


