/*
 * spi_tx_arduino.c
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
int main ()
{
	//using STM (Master) and Arduino(slave) board
	//Need to use GPIO pins for MISO , MOSI , NSS and SCLK

	//Define the Message to be sent
	char master_data[]="Hi My name is Gaurav Pai";

	uint8_t master_data_len= strlen(master_data);
	SPI_GPIOInits();

	GPIO_ButtonInit();

    SPI_Inits();

    // we are using the Hardware slave management , need to config the SSOE bit in CR2 register
    SPI_SSOE_Config(SPI2, ENABLE);
    // we need to enable the peripheral after all the init


    while(1)
    {

    	while(!GPIO_ReadFromInputPin(GPIOA,GPIO_BTN_PIN));


    	SPI_PeripheralControl(SPI2, ENABLE);
    	// first send the number of bytes to be transfered
    	SPI_SendData(SPI2,&master_data_len,1);

    	// transfer those bytes
    	SPI_SendData(SPI2,(uint8_t *)master_data, master_data_len);
    	// above procedure is followed in sketch file --> First send number bytes , followed by those bytes


    	//once transfer is done disable the SPI Peripheral
    	//Good Methodology  to use an SPI_SR BSY bit to make sure SPI is not busy and only than we disable the SPI Peripheral , rather than doing abruptly
    	// if busy keep hanging
    	while(SPI_SR_BSY_Status(SPI2)==SPI_BSY_BUSY);
    	// after all data send , Disable the Peripheral
    	SPI_PeripheralControl(SPI2,DISABLE);


    }

	return 0;

}
