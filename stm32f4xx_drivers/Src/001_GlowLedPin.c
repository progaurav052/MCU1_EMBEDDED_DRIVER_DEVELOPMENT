/*
 * 001_GlowLedPin.c
 *
 *  Created on: Nov 30, 2025
 *      Author: ggpai
 */


#include "stm32f407xx.h"

int main(){

	//Need to glow an Led in Push Pull Mode (Default)
	GPIO_Handle_t GPIOLed;
	GPIOLed.pGPIOx=GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	// Need to enable the RCC AHB1ENR clock

	GPIO_PeripheralClockControl(GPIOLed.pGPIOx, ENABLE);

	// now pass the address of this handle to the driver.c init function
	GPIO_Init(&GPIOLed);
	GPIO_WriteToOutputPin(GPIOLed.pGPIOx,GPIOLed.GPIO_PinConfig.GPIO_PinNumber,GPIO_PIN_SET);

    return 0;

}
