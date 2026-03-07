/*
 * lcd.h
 *
 *  Created on: Mar 2, 2026
 *      Author: ggpai
 */

#ifndef LCD_H_
#define LCD_H_

#include "stm32f407xx.h"

//also define some user configurable macros
#define LCD_GPIO_PORT 	GPIOD
#define LCD_GPIO_RS		GPIO_PIN_NO_0
#define LCD_GPIO_RW		GPIO_PIN_NO_1
#define LCD_GPIO_EN		GPIO_PIN_NO_2
#define LCD_GPIO_D4		GPIO_PIN_NO_3
#define LCD_GPIO_D5		GPIO_PIN_NO_4
#define LCD_GPIO_D6		GPIO_PIN_NO_5
#define LCD_GPIO_D7		GPIO_PIN_NO_6
#define LCD_GPIO_PIN_MODE	GPIO_MODE_OUT


void lcd_init(void);

#endif /* LCD_H_ */
