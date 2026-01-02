/*
 * spi_cmd_handling.c
 *
 *  Created on: Jan 2, 2026
 *      Author: ggpai
 */

#include <string.h>
#include "stm32f407xx.h"
#define HIGH  1
#define LOW   0

#define BTN_PRESSED   LOW
#define GPIO_BTN_PIN   0

#define ACK_BYTE_VALUE    0xF5
#define NACK_BYTE_VALUE   0xA5


//command codes
#define COMMAND_LED_CTRL       	0x50
#define COMMAND_SENSOR_READ    	0x51
#define COMMAND_LED_READ       	0x52
#define COMMAND_PRINT		   	0x53
#define COMMAND_ID_READ			0x54

//Arduino analog pins
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2     2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4
#define ANALOG_PIN5     5


#define LED_ON    1
#define LED_OFF   0


#define LED_PIN   9

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

void SPI_GPIOInits()
{
	// initialize the peripheral clock before init
	GPIO_Handle_t SPI_GpioPins;

	SPI_GpioPins.pGPIOx=GPIOB;
	SPI_GpioPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	SPI_GpioPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP; //mentioned in ref manual
	SPI_GpioPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	SPI_GpioPins.GPIO_PinConfig.GPIO_PinAltFunMode=GPIO_ALTFN_5;
	SPI_GpioPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;


	//MOSI init
	SPI_GpioPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	GPIO_Init(&SPI_GpioPins);

	//MISO init
	SPI_GpioPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
	GPIO_Init(&SPI_GpioPins);

	//SCLK init
	SPI_GpioPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIO_Init(&SPI_GpioPins);

	//NSS init
	SPI_GpioPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIO_Init(&SPI_GpioPins);



}

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

void SPI_Inits()
{
	SPI_Handle_t SPI_Pins;

	SPI_Pins.pSPIx=SPI2;

	SPI_Pins.SPIConfig.SPI_BusConfig=SPI_BUS_CONFIG_FD;
	SPI_Pins.SPIConfig.SPI_DFF=SPI_DFF_8BITS;
	SPI_Pins.SPIConfig.SPI_CPOL=SPI_CPOL_LOW;
	SPI_Pins.SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	SPI_Pins.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	SPI_Pins.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV2;
	SPI_Pins.SPIConfig.SPI_SSM=SPI_SSM_DI;

	// Init
	SPI_Init(&SPI_Pins);

}

uint8_t SPI_verifySlaveResponse(uint8_t ack_byte)
{
	if(ack_byte == ACK_BYTE_VALUE)
	{
		return 1;
	}
	return 0;

}
int main()
{
	//master slave communication
	// master will send the command first to which slave will respond with ACK or NACK byte
	// if ACK byte, send 1 or more command arguments, based on this it will perform an action
	// if NACK byte Display error message
	// for each action either slave might do some action like turning on led  or respond back with some data like analog read etc

	//Do the GPIO_init for all SPI_GPIO Pins
	//Define the Message to be sent

	uint8_t dummy_write_byte = 0xff;
	uint8_t dummy_read_byte;
	uint8_t ack_byte;


	uint8_t args[2];



	SPI_GPIOInits();

	GPIO_ButtonInit(); // for button press

	SPI_Inits(); // init SPI Peripheral with config

	// we are using the Hardware slave management , need to config the SSOE bit in CR2 register
	SPI_SSOE_Config(SPI2, ENABLE);
	// we need to enable the peripheral after all the init

	while(1)
	{

	    	while(!(GPIO_ReadFromInputPin(GPIOA,GPIO_BTN_PIN) == BTN_PRESSED)); //keep waiting for Btn press
	    	// add Delay if required , to solve debouncing issue of button
	    	delay();


	    	SPI_PeripheralControl(SPI2, ENABLE);

	    	//1.  CMD_LED_CTRL     <pin no (1)>   <value (1)>
	    	uint8_t commandCode=COMMAND_LED_CTRL;
	    	SPI_SendData(SPI2,&commandCode, sizeof(commandCode)); // this will push the commandCode into the TX buffer -> shift register than transmitted on MOSI Line to slave
	    	SPI_ReceiveData(SPI2,&dummy_read_byte,sizeof(dummy_read_byte)); // to clear the Rx buffer which got filled when we did SendData, can be cleared by Reading the Rx buffer
	    	// if slave recognizes the command code it will send ACK byte, which we we should receive over MISO Line
	    	// slave does not initiate the data transfer so we should send some dummy data to push the ACK byte from its Shift register to our Master
	    	// Send dummy_write_ byte
	    	SPI_SendData(SPI2,&dummy_write_byte,sizeof(dummy_write_byte));
	    	// after the above send Data we will have the ACK/NACK Byte in our shift register->RxBuffer , which will read into our <received_data>
	    	SPI_ReceiveData(SPI2,&ack_byte,sizeof(ack_byte));

	    	if(SPI_verifySlaveResponse(ack_byte)==ACK_BYTE_VALUE)
	    	{
	    		// proceed with sending arguments
	    		args[0]=LED_PIN;
	    		args[1]=LED_ON;
	    		SPI_SendData(SPI2,args,2);

	    	}
	    	//End of CMD_CTRL_LED command



	    	//once transfer is done disable the SPI Peripheral
	    	//Good Methodology  to use an SPI_SR BSY bit to make sure SPI is not busy and only than we disable the SPI Peripheral , rather than doing abruptly
	    	// if busy keep hanging
	    	while(SPI_SR_BSY_Status(SPI2)==SPI_BSY_BUSY);
	    	// after all data send , Disable the Peripheral
	    	SPI_PeripheralControl(SPI2,DISABLE);

	 }


	return 0;

}
