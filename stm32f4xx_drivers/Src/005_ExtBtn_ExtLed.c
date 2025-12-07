/*
 * 005_ExtBtn_ExtLed.c
 *
 *  Created on: Dec 6, 2025
 *      Author: ggpai
 */

#include "stm32f407xx.h"
#define LOW 0
#define BTN_PRESSED LOW
void delay(void){
	for(uint32_t i=0;i<500000;i++);

}
int main()
{
	GPIO_Handle_t GPIOLed,GPIOButton;


	GPIOButton.pGPIOx=GPIOB;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;

	GPIOLed.pGPIOx=GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_7;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;


	GPIO_PeripheralClockControl(GPIOB, ENABLE);
	GPIO_PeripheralClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIOLed);
	GPIO_Init(&GPIOButton);

	while(1){
		if(GPIO_ReadFromInputPin(GPIOButton.pGPIOx,GPIOButton.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED){

			GPIO_WriteToOutputPin(GPIOLed.pGPIOx,GPIOLed.GPIO_PinConfig.GPIO_PinNumber,GPIO_PIN_SET);

		}
		else
		{
			GPIO_WriteToOutputPin(GPIOLed.pGPIOx,GPIOLed.GPIO_PinConfig.GPIO_PinNumber,GPIO_PIN_RESET);
		}


	}




	return 0;

}
