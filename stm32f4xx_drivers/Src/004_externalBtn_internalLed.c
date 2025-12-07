/*
 * 004_externalBtn_internalLed.c
 *
 *  Created on: Dec 5, 2025
 *      Author: ggpai
 */


#include "stm32f407xx.h"

int main()
{
	GPIO_Handle_t GPIOLed_12,GPIOLed_13,GPIOLed_14,GPIOLed_15,GPIOExtBtn,GPIOLedE_8;

	GPIOLed_12.pGPIOx=GPIOD;
	GPIOLed_12.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIOLed_12.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIOLed_12.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIOLed_12.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOLed_12.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;

	GPIOLed_13.pGPIOx=GPIOD;
	GPIOLed_13.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIOLed_13.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIOLed_13.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIOLed_13.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOLed_13.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;

	GPIOLed_14.pGPIOx=GPIOD;
	GPIOLed_14.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIOLed_14.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
	GPIOLed_14.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIOLed_14.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOLed_14.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;

	GPIOLed_15.pGPIOx=GPIOD;
	GPIOLed_15.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIOLed_15.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	GPIOLed_15.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIOLed_15.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOLed_15.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;

	GPIOLedE_8.pGPIOx=GPIOA;
	GPIOLedE_8.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIOLedE_8.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_8;
	GPIOLedE_8.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIOLedE_8.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOLedE_8.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;


	GPIOExtBtn.pGPIOx=GPIOB;
	GPIOExtBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIOExtBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIOExtBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
	GPIOExtBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	GPIO_PeripheralClockControl(GPIOD, ENABLE);
	GPIO_PeripheralClockControl(GPIOExtBtn.pGPIOx, ENABLE);

	GPIO_Init(&GPIOLed_12);
	GPIO_Init(&GPIOLed_13);
	GPIO_Init(&GPIOLed_14);
	GPIO_Init(&GPIOLed_15);
	GPIO_Init(&GPIOExtBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOExtBtn.pGPIOx, GPIOExtBtn.GPIO_PinConfig.GPIO_PinNumber))
		{
			GPIO_WriteToOutputPin(GPIOLed_12.pGPIOx, GPIOLed_12.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);
			GPIO_WriteToOutputPin(GPIOLed_13.pGPIOx, GPIOLed_13.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);
			GPIO_WriteToOutputPin(GPIOLed_14.pGPIOx, GPIOLed_14.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);
			GPIO_WriteToOutputPin(GPIOLed_15.pGPIOx, GPIOLed_15.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);
			GPIO_WriteToOutputPin(GPIOLedE_8.pGPIOx, GPIOLedE_8.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);
		}
		else
		{
			GPIO_WriteToOutputPin(GPIOLed_12.pGPIOx, GPIOLed_12.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);
			GPIO_WriteToOutputPin(GPIOLed_13.pGPIOx, GPIOLed_13.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);
			GPIO_WriteToOutputPin(GPIOLed_14.pGPIOx, GPIOLed_14.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);
			GPIO_WriteToOutputPin(GPIOLed_15.pGPIOx, GPIOLed_15.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);
			GPIO_WriteToOutputPin(GPIOLedE_8.pGPIOx, GPIOLedE_8.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);

		}

	}



	return 0;
}
