/*
 * main.c
 *
 *  Created on: Dec 9, 2025
 *      Author: ggpai
 */

#include "stm32f407xx.h"

int main()
{
	// connect the Button to PD5  external Led to P12
	GPIO_Handle_t GPIOLed,GPIOButton;

	//initialize the GPIOLed and GPIO Button

	GPIOLed.pGPIOx=GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIOButton.pGPIOx=GPIOD;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;


	// enable the peripheral clock control
	GPIO_PeripheralClockControl(GPIOLed.pGPIOx, ENABLE);
	GPIO_PeripheralClockControl(GPIOButton.pGPIOx, ENABLE);


	//initialize the configuration using init-function
	GPIO_Init(&GPIOButton);
	GPIO_Init(&GPIOLed);

	//call the functions to configure the interrupt on the Processor side (NVIC)
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENBALE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, 15);





	while(1);


}

void EXTI9_5_IRQHandler(void)
{
	// ISR which will call  the GPIO_handler

	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);



}
