/*
 * stm32f4xx_rcc_driver.c
 *
 *  Created on: Feb 27, 2026
 *      Author: ggpai
 */

#include "stm32f4xx_rcc_driver.h"

uint32_t  AHB_PS[8]={2,4,8,16,64,128,256,512};
uint32_t  APB_PS[5]={2,4,8,16};

void RCC_GetPLLOutputClock(){

}

static uint32_t findClockSource()
{
	uint32_t temp;

	temp = ((RCC->CFGR >> 2) & 0x3);

	return temp;

}
uint32_t  RCC_GetPCLK2Value(){
	// clock source --> AHb1 prescaler --> APB1 prescaler ---> APB1 peripheral clocks

		// find the clock source from RCC_CFGR Regsiter
		uint32_t clckSource=0,SysClk=0,AHB_Prescaler=0, APB_Prescaler=0,temp=0,PCLK_SPEED;

		clckSource=findClockSource();

		if(clckSource==0)
		{
	       SysClk=16000000;

		}
		else if(clckSource ==1)
		{
			SysClk=8000000;

		}
		else if(clckSource ==2)
		{
			RCC_GetPLLOutputClock();

		}

		//need to find the preSclaer of AHB

		temp = ((RCC->CFGR >> 4) &  0xF);
		if(temp<8)
		{
			AHB_Prescaler=1;
		}
		else
		{
			AHB_Prescaler=AHB_PS[temp-8];
		}


		//need to find the Prescaler of APB
		temp = ((RCC->CFGR >> 13) &  0x7);
		if(temp < 4)
		{
			APB_Prescaler=1;
		}
		else
		{
			APB_Prescaler=APB_PS[temp-4];
		}


		PCLK_SPEED=((SysClk/AHB_Prescaler)/APB_Prescaler);

		return PCLK_SPEED;


}
uint32_t RCC_GetPCLK1Value(){
	// clock source --> AHb1 prescaler --> APB1 prescaler ---> APB1 peripheral clocks

		// find the clock source from RCC_CFGR Regsiter
		uint32_t clckSource=0,SysClk=0,AHB_Prescaler=0, APB_Prescaler=0,temp=0,PCLK_SPEED;

		clckSource=findClockSource();

		if(clckSource==0)
		{
	       SysClk=16000000;

		}
		else if(clckSource ==1)
		{
			SysClk=8000000;

		}
		else if(clckSource ==2)
		{
			RCC_GetPLLOutputClock();

		}

		//need to find the preSclaer of AHB

		temp = ((RCC->CFGR >> 4) &  0xF);
		if(temp<8)
		{
			AHB_Prescaler=1;
		}
		else
		{
			AHB_Prescaler=AHB_PS[temp-8];
		}


		//need to find the Prescaler of APB
		temp = ((RCC->CFGR >> 10) &  0x7);
		if(temp < 4)
		{
			APB_Prescaler=1;
		}
		else
		{
			APB_Prescaler=APB_PS[temp-4];
		}


		PCLK_SPEED=((SysClk/AHB_Prescaler)/APB_Prescaler);

		return PCLK_SPEED;

}
