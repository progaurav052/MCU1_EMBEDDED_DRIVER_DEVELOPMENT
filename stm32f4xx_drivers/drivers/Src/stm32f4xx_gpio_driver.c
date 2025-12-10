/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: Sep 20, 2025
 *      Author: LENOVO
 */

#include "stm32f4xx_gpio_driver.h"
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp=0;

	//1. configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// non - interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		// now do set this mode in the GPIO register structure which is pointing to actual GPIOx peripheral
		pGPIOHandle->pGPIOx->MODER &=(~(0x3 <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		pGPIOHandle->pGPIOx->MODER |= temp;

	}
	else
	{
		// interrupt mode - code it later
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT)
		{
			// configure Falling trigger using FTSR
			EXTI->FTSR|= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//clear the corresponding RTSR bit for safety
			EXTI->RTSR|= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT)
		{
			//configure Rising trigger RTSR
			EXTI->RTSR|= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//clear the corresponding FTSR bit for safety
			EXTI->FTSR|= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT)
		{
			// configure both falling trigger and rising trigger
			// FTSR and RTSR
			EXTI->FTSR|= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR|= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		//2. configure the GPIO port selection in the SYSCFG_EXTICR

		uint8_t temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode =GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1]|= (portcode << (temp2*4));




		//3.  enable the EXTI interrupt Delivery using Interrupt mask register
        // enable that EXTI line  to deliver the interrupt from MCU side
		EXTI->IMR=(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	//2. configure the speed
	temp=0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &=(~(0x3 <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3. configure the pupd settings
	temp=0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &=(~(0x3 <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	//4. configure the optype
	// this code should be configured only if the mode is in OUTPUT Mode
	// Need to add checks for this code
	temp=0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &=(~(0x1 <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. configure the altfn
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
			// configure the altfn registers
		uint8_t temp1,temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode /8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode %8;

		if(temp1 ==0){
			pGPIOHandle->pGPIOx->AFRL &=~(0xF << (4 * temp2));
			pGPIOHandle->pGPIOx->AFRL |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));

		}
		else
		{
			pGPIOHandle->pGPIOx->AFRH &=~(0xF<< (4 * temp2));
			pGPIOHandle->pGPIOx->AFRH |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
		}


	}

}

void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi){

	if(EnorDi == ENABLE){
		// check *pGIOx is pointing to which GPIO peripheral
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();

		}

	}
	else{
				if(pGPIOx == GPIOA){
					GPIOA_PCLK_DI();
				}
				else if(pGPIOx == GPIOB){
					GPIOB_PCLK_DI();
				}
				else if(pGPIOx == GPIOC){
					GPIOC_PCLK_DI();
				}
				else if(pGPIOx == GPIOD){
					GPIOD_PCLK_DI();
				}
				else if(pGPIOx == GPIOE){
					GPIOE_PCLK_DI();
				}
				else if(pGPIOx == GPIOF){
					GPIOF_PCLK_DI();
				}
				else if(pGPIOx == GPIOG){
					GPIOG_PCLK_DI();
				}
				else if(pGPIOx == GPIOH){
					GPIOH_PCLK_DI();
				}
				else if(pGPIOx == GPIOI){
					GPIOI_PCLK_DI();

				}

	}


}
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber)&0x1); // this logic is more suitable rather than shifting 1 to the right , because return value is 0 or 1
	return value; //either 0 or 1
} // Read a bit from IDR of GPIO port
uint16_t  GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;

} // Read full IDR 16 bits
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber, uint8_t Value){
	// set an bit in the ODR
	if(Value==GPIO_PIN_SET)
	{
		pGPIOx->ODR|=(1<<PinNumber);

	}
	else if(Value == GPIO_PIN_RESET)
	{
		pGPIOx->ODR&=~(1<<PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value){

	pGPIOx->ODR=Value;

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR^=(1<<PinNumber);

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
				GPIOA_REG_RESET();
			}
			else if(pGPIOx == GPIOB){
				GPIOB_REG_RESET();
			}
			else if(pGPIOx == GPIOC){
				GPIOC_REG_RESET();
			}
			else if(pGPIOx == GPIOD){
				GPIOD_REG_RESET();
			}
			else if(pGPIOx == GPIOE){
				GPIOE_REG_RESET();
			}
			else if(pGPIOx == GPIOF){
				GPIOF_REG_RESET();
			}
			else if(pGPIOx == GPIOG){
				GPIOG_REG_RESET();
			}
			else if(pGPIOx == GPIOH){
				GPIOH_REG_RESET();
			}
			else if(pGPIOx == GPIOI){
				GPIOI_REG_RESET();
			}
}



/* Interrupt configuration , configuring NVIC registers ISER and ICER */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnorDi){
	// Note only around 96 IRQ are implemented
	// we can use just first three ISER and ICER registers
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <=31)
		{
           *NVIC_ISER0 |=(1 << IRQNumber);
		}
		else if(IRQNumber >=32 && IRQNumber <64)
		{
			*NVIC_ISER1 |=(1 << (IRQNumber % 32));

		}
		else if(IRQNumber >=64 && IRQNumber <96)
		{
			*NVIC_ISER2 |=(1 << (IRQNumber % 32));
		}
	}
	else
	{
		if(IRQNumber <=31)
		{
			*NVIC_ICER0 |=(1 << IRQNumber);
		}
		else if(IRQNumber >=32 && IRQNumber <64)
		{
			*NVIC_ICER1 |=(1 << (IRQNumber % 32));

		}
		else if(IRQNumber >=64 && IRQNumber <96)
		{
			*NVIC_ICER2 |=(1 << (IRQNumber % 32));

		}
	}

}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	// in each of the section first 4 bits lower half is not implemented , only upper half is implemented
	uint8_t shift_amount = (8 *iprx_section) + (8 - NO_OF_BITS_IMPLEMENTED_IPR);

	*(NVIC_PRI_BASE_ADDR + (iprx*4)) |= (IRQPriority  << shift_amount);
}



void GPIO_IRQHandling(uint8_t PinNumber){
 //clear EXTI PR register corresponding to the pin number
 if(EXTI->PR &(1 << PinNumber))
 {
	 //clear by writing 1 , this is the procedure mentioned in Documentation
	 EXTI->PR |=(1 << PinNumber);

 }

}
