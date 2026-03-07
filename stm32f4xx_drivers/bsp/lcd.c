/*
 * lcd.c
 *
 *  Created on: Mar 2, 2026
 *      Author: ggpai
 */


#include "lcd.h"
static void lcd_gpio_pin_config();

void lcd_init(){
	//1.configure the GPIO Pins used by LCD
	lcd_gpio_pin_config();

	//2. do LCD Init - refer the datasheet of LCD

}

static void lcd_gpio_pin_config()
{
	GPIO_Handle_t lcd_gpio_pins;

	lcd_gpio_pins.pGPIOx=LCD_GPIO_PORT;
	lcd_gpio_pins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	lcd_gpio_pins.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_RS;
	lcd_gpio_pins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	lcd_gpio_pins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	lcd_gpio_pins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

	GPIO_Init(&lcd_gpio_pins);


	lcd_gpio_pins.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
	GPIO_Init(&lcd_gpio_pins);

	lcd_gpio_pins.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	GPIO_Init(&lcd_gpio_pins);

	lcd_gpio_pins.GPIO_PinConfig.GPIO_PinNumber=LCD_GPIO_D4;
	GPIO_Init(&lcd_gpio_pins);

	lcd_gpio_pins.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
	GPIO_Init(&lcd_gpio_pins);

	lcd_gpio_pins.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	GPIO_Init(&lcd_gpio_pins);

	lcd_gpio_pins.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	GPIO_Init(&lcd_gpio_pins);




	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	//all pins are configured in Output mode in our case


}
