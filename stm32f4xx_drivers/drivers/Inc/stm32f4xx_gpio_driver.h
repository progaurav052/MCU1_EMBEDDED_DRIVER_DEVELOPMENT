/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: Sep 20, 2025
 *      Author: LENOVO
 */

#ifndef INC_STM32F4XX_GPIO_DRIVER_H_
#define INC_STM32F4XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/* structure to config various modes , initial config*/
typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


// Handle structure for GPIO pin
typedef struct{

	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;


}GPIO_Handle_t;

// this above structure will be provided by the driver API to the applications to change the config


/******DRIVER API prototype****************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle); //initialize all modes
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi); //for a given GPIO base address enable/diable clock

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber); // Read a bit from IDR of GPIO port
uint16_t  GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx); // Read full IDR 16 bits
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber); // from which pin Interrupt has been triggerd


#endif /* INC_STM32F4XX_GPIO_DRIVER_H_ */
