/*
 * main.c
 *
 *  Created on: Dec 9, 2025
 *      Author: ggpai
 */

#include "stm32f407xx.h"

int main(){
	return 0;

}

void EXTI0_IRQHandler(void){
	// handle the Interrupt
	GPIO_IRQHandling(0);

}
