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

#define MY_ADDRESS  	0x61
#define SLAVE_ADDR  	0x68

I2C_Handle_t I2C1_Handle;
uint8_t NBytes;
uint8_t rcvBuffer[32];

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
	I2C1_pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_7;
	GPIO_Init(&I2C1_pins);



}

void I2C1_Inits(){


	I2C1_Handle.pI2Cx=I2C1;
	I2C1_Handle.I2C_Config.I2C_AckControl=I2C_ACK_ENABLE;
	I2C1_Handle.I2C_Config.I2C_FMDutyCycle=I2C_FM_DUTY_2; // does not matter we are using Standard mode for this example
	I2C1_Handle.I2C_Config.I2C_DeviceAddress=MY_ADDRESS; // if slave this is where we store slave address in OAR
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
	uint8_t commandCode;

	GPIO_ButtonInit();

	//I2C pin init
	I2C1_GpioInits();

	//I2C peripheral init

	I2C1_Inits();

	//after all the configuration is done enable the I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ManageAcking(I2C1_Handle.pI2Cx,ENABLE);

	//when the  button is pressed , arduino sends the data to STM board
	//first sends the number of bytes , followed by the those bytes
	//make use of button code
	while(1)
	{

		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		commandCode=0x51;

		//send 0x51 to read number of bytes in read phase
		I2C_MasterSendData(&I2C1_Handle,&commandCode,1,SLAVE_ADDR);

		I2C_MasterReceiveData(&I2C1_Handle,&NBytes,1, SLAVE_ADDR);
		//after this you will have number of bytes in the Nbytes variable

		commandCode=0x52;
		//send 0x51 to read number of bytes in read phase
		I2C_MasterSendData(&I2C1_Handle,&commandCode,1,SLAVE_ADDR);

		// do another read to read those Nbytes of Data
		I2C_MasterReceiveData(&I2C1_Handle,rcvBuffer,NBytes,SLAVE_ADDR);



	}


	return 0;

}

