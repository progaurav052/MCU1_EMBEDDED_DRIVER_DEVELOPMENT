/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: Dec 18, 2025
 *      Author: ggpai
 */

#ifndef INC_STM32F4XX_SPI_DRIVER_H_
#define INC_STM32F4XX_SPI_DRIVER_H_
#include "stm32f407xx.h"

typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

typedef struct{
	SPI_RegDef_t *pSPIx; //either SPI1 , SPI2 , SPI3
	SPI_Config_t SPIConfig;

}SPI_Handle_t;






#endif /* INC_STM32F4XX_SPI_DRIVER_H_ */
