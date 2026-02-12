/*
 * 007_spi_interrupt_communication.c
 *
 *  Created on: Jan 25, 2026
 *      Author: ggpai
 */

#include "stm32f407xx.h"
#include<stdio.h>
#include<stddef.h>

int data_available=0;
int stopreceiving;
#define MAX_LEN 500
char readbuffer[MAX_LEN];

uint8_t ReadByte;


SPI_Handle_t SPI_Pins;

void SPI_GPIOInits()
{
	// initialize the peripheral clock before init
	GPIO_Handle_t SPI_GpioPins;

	memset(&SPI_GpioPins,0,sizeof(SPI_GpioPins)); // good Practice
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

void SPI_Inits()
{


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

void Gpio_Inits(){

	// Need this to configure PD6 as interrupt mode to get FT _ interrupt
	GPIO_Handle_t GpioITR;

	memset(&GpioITR,0,sizeof(GpioITR)); // Good Practice
	GpioITR.pGPIOx=GPIOD;
	GpioITR.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
	GpioITR.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_6;
	GpioITR.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioITR.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;

	GPIO_Init(&GpioITR);


	// this GPIOD port has to configured for interrupt as well
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5 , ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,15);





}


int main()
{
	uint8_t dummyWriteByte=0xff;

	//SPI Peripheral inits
	SPI_GPIOInits();
	SPI_Inits();

	//GPIO_init for interrupt
	Gpio_Inits();

	// we are using NSS pin so need to enbale NSS output
	SPI_SSOE_Config(SPI2, ENABLE);
	SPI_IRQInterruptConfig(IRQ_NO_SPI2,ENABLE);

	while(1){

			stopreceiving=0;
			while(!data_available);

			//at this point interrupt GPIO_interrupt got triggered due to FT interrupt, Data must be availabe
			//make sure to DISABLE interrupt on GPIO_line when data is being transmitted
			GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5 , DISABLE);

			//enbale the SPI Peripheral
			SPI_PeripheralControl(SPI2, ENABLE);


			while(!stopreceiving)
			{
				while(SPI_SendDataWithIT(&SPI_Pins,&dummyWriteByte,1) ==SPI_BUSY_IN_TX);
				while(SPI_ReceiveDataWithIT(&SPI_Pins,&ReadByte,1) == SPI_BUSY_IN_RX);

				//these while loops are just an design pattern that, in our case this wont be very use full
				// think of case where 2 back to back sendDataWithIT is used ... and dry run the code
				//Need to understand ISR execution rule in detail for this

			}

		// confirm SPI is not busy
			while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

			//Disable the SPI2 peripheral
			SPI_PeripheralControl(SPI2,DISABLE);

			printf("Rcvd data = %s\n",readbuffer);

			data_available = 0;

			GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);

	}

	return 0;

}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i = 0; //Each time SPI_ApplicationEventCallback() is called again, i keeps its previous value
	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
				readbuffer[i++] = ReadByte;
				if (ReadByte == '\0') {
				    stopreceiving=1;
				    i = 0;
				}
				else if (i == MAX_LEN) {
				    readbuffer[i-1] = '\0';  // force termination
				    stopreceiving=1;
				    i = 0;
				}
	}

}

void EXTI9_5_IRQHandler(void)
{
	// ISR which will call  the GPIO_handler
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	data_available=1;


}

void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI_Pins);
}
