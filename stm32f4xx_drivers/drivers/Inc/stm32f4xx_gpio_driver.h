/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: Sep 20, 2025
 *      Author: LENOVO
 */

#ifndef INC_STM32F4XX_GPIO_DRIVER_H_
#define INC_STM32F4XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/* structure to configuration various modes , initial configuration*/
// user application should create an Handle structure and send it to driver , driver from this configuration  will init the Peripheral

typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;   /*!< Possible values from @GPIO_PIN_MODES>*/
	uint8_t GPIO_PinSpeed;   /*!< Possible values from @GPIO_SPEED_*>*/
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

/*GPIO pin Number possible values*/

#define GPIO_PIN_NO_0           0
#define GPIO_PIN_NO_1           1
#define GPIO_PIN_NO_2           2
#define GPIO_PIN_NO_3           3
#define GPIO_PIN_NO_4           4
#define GPIO_PIN_NO_5           5
#define GPIO_PIN_NO_6           6
#define GPIO_PIN_NO_7           7
#define GPIO_PIN_NO_8           8
#define GPIO_PIN_NO_9           9
#define GPIO_PIN_NO_10          10
#define GPIO_PIN_NO_11          11
#define GPIO_PIN_NO_12          12
#define GPIO_PIN_NO_13          13
#define GPIO_PIN_NO_14          14
#define GPIO_PIN_NO_15          15


/* GPIO Pin Possible Modes */
#define GPIO_MODE_IN  			0
#define GPIO_MODE_OUT   		1
#define GPIO_MODE_ALTFN 		2
#define GPIO_MODE_ANALOG 		3
// we also have to configure for the Interrupt Modes such as falling edge , rising edge , etc ...
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT 		5
#define GPIO_MODE_IT_RFT		6



/*GPIO Pin Possible output Types*/
#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1


/*GPIO pin possible output speed modes*/
#define GPIO_SPEED_LOW  			0
#define GPIO_SPEED_MEDIUM   		1
#define GPIO_SPEED_FAST 			2
#define GPIO_SPEED_HIGH				3


/*GPIO pin pull up pull down configuration  */
#define GPIO_NO_PUPD				0
#define GPIO_PIN_PU					1
#define GPIO_PIN_PD                 2

/*GPIO ALternate functionality possible values */
// there are 15 different alternate functionalities

#define GPIO_ALTFN_0   				0
#define GPIO_ALTFN_1   				1
#define GPIO_ALTFN_2   				2
#define GPIO_ALTFN_3   				3
#define GPIO_ALTFN_4   				4
#define GPIO_ALTFN_5   				5
#define GPIO_ALTFN_6   				6
#define GPIO_ALTFN_7   				7
#define GPIO_ALTFN_8   				8
#define GPIO_ALTFN_9   				9
#define GPIO_ALTFN_10  				10
#define GPIO_ALTFN_11   			11
#define GPIO_ALTFN_12   			12
#define GPIO_ALTFN_13  				13
#define GPIO_ALTFN_14   			14
#define GPIO_ALTFN_15  				15

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
