/*
 * 003_led_btnPress.c
 *
 *  Created on: Dec 1, 2025
 *      Author: ggpai
 */

#include "stm32f407xx.h"
void delay(void){
	for(uint32_t i=0;i<500000;i++);

}
int main(){
	GPIO_Handle_t GPIOLed,GPIOButton; // user wakeup button connected to PA0

	// config for Led
	GPIOLed.pGPIOx=GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;


	//config for Button - PA0; = input mode

	GPIOButton.pGPIOx=GPIOA;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	// Enbale the RCC clock
	GPIO_PeripheralClockControl(GPIOLed.pGPIOx, ENABLE);
	GPIO_PeripheralClockControl(GPIOButton.pGPIOx, ENABLE);

	//initialize the config
	GPIO_Init(&GPIOButton);
	GPIO_Init(&GPIOLed);


	while(1){
		if(GPIO_ReadFromInputPin(GPIOButton.pGPIOx,GPIOButton.GPIO_PinConfig.GPIO_PinNumber)==1){
			 // this delay is left to the user to configure ... current code is running good without dealy
			// used to settle the vibrations in the plates
			GPIO_WriteToOutputPin(GPIOLed.pGPIOx, GPIOLed.GPIO_PinConfig.GPIO_PinNumber,GPIO_PIN_SET);
		}
		else{
			GPIO_WriteToOutputPin(GPIOLed.pGPIOx, GPIOLed.GPIO_PinConfig.GPIO_PinNumber,GPIO_PIN_RESET);
		}

	}


	return 0;

}
