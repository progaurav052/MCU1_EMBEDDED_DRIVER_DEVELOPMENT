/*
 * main.c
 *
 *  Created on: Dec 21, 2025
 *      Author: ggpai
 */

#include "stm32f407xx.h"

void SPI_GPIOInits(void)
{
	// initialize the GPIO Port of your choice in AFMODE
	GPIO_Handle_t SPI_GpioPins;

	SPI_GpioPins.pGPIOx=GPIOB;
	SPI_GpioPins.GPIO_PinConfig.GPIO_PinAltFunMode=GPIO_ALTFN_5;
	SPI_GpioPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	SPI_GpioPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP; // for I2c its OD mentioned in manual
	SPI_GpioPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	SPI_GpioPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	//enable the Peripheral clock
	GPIO_PeripheralClockControl(SPI_GpioPins.pGPIOx, ENABLE);


	// SCLK pin
    SPI_GpioPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
    GPIO_Init(&SPI_GpioPins);

    // NSS
    //SPI_GpioPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
    //GPIO_Init(&SPI_GpioPins);

    // MOSI
    SPI_GpioPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
    GPIO_Init(&SPI_GpioPins);

    // MISO
    //SPI_GpioPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
    //GPIO_Init(&SPI_GpioPins);


}

void SPI_Inits()
{
	SPI_Handle_t SPI_Pins;

	SPI_Pins.pSPIx=SPI2;
	SPI_Pins.SPIConfig.SPI_BusConfig=SPI_BUS_CONFIG_FD;
	SPI_Pins.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	SPI_Pins.SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
	SPI_Pins.SPIConfig.SPI_CPOL=SPI_CPOL_HIGH;
	SPI_Pins.SPIConfig.SPI_DFF=SPI_DFF_8BITS;
	SPI_Pins.SPIConfig.SPI_SSM=SPI_SSM_EN; // we are using master only
	SPI_Pins.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV2;

	// enable the Peripheral clock


	//Initialize
	SPI_Init(&SPI_Pins);

}
int main()
{
	//create an Data buffer	char user_data[]= "Hello world"; // length in bytes

    char user_data[]="J";
	// Identified the GPIO pins we need to use in AF mode from Datasheet
	//GPIO initialize
	SPI_GPIOInits();

	//SPI initialize
	SPI_Inits();

	SPI_SSI_Config(SPI2,ENABLE);

	//for MASTER Slave communication where you need to do Hardware slave management by using NSS ping for master to pull the SS of slave to ground
	//use SPI_CR2_SSOE bit to enable the NSS output for master , which can be controlled with SSM and SPE bit in CR register
	//Refer Notes for this
    //SPI_SSOE_Config(SPI2,ENABLE);

	//Enable the SPI peripheral -- > all the Initialization  to CR register has to be done before this step (recommended)
	SPI_PeripheralControl(SPI2,ENABLE);

	//Send Data
	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));


	//Good Methodology  to use an SPI_SR BSY bit to make sure SPI is not busy and only than we disable the SPI Peripheral , rather than doing abruptly
	// if busy keep hanging
	while(SPI_SR_BSY_Status(SPI2)==SPI_BSY_BUSY);

	// after all data send , Disable the Peripheral
	SPI_PeripheralControl(SPI2,DISABLE);

	while(1);

	return 0;

}
