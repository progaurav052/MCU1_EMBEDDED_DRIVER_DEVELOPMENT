/*
 * lcd.c
 *
 *  Created on: Mar 2, 2026
 *      Author: ggpai
 */


#include "lcd.h"

static void write_4_bits(uint8_t value);
static void lcd_enable(void);
static void mdelay(uint32_t cnt);
static void udelay(uint32_t cnt);



void lcd_send_command(uint8_t cmd) // this API will be exposed to application
{
	/*RS =0 , for LCD command */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	/*RW =0 , for write to LCD */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	//first send higher nibble
	write_4_bits(cmd >> 4);

	//lower bits nibble
	write_4_bits(cmd & 0x0F);


}

void lcd_send_char(uint8_t data)// this API will be exposed to application
{
	/*RS =0 , for LCD command */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

	/*RW =0 , for write to LCD */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	//first send higher nibble
	write_4_bits(data >> 4);

	//lower bits nibble
	write_4_bits(data & 0x0F);


}
void lcd_print_string(char *message)
{
	//char * is pointer to string in c , remember this
	do
	{
		lcd_send_char((uint8_t)*message++); // we are sending data to send_char function


	}while(*message!='\0');

}

void lcd_init(){

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

		//flow chart --> power on-->
		mdelay(40);// give an 40ms delay as mentioned in HITACHI doc for LCD

		/*RS =0 , for LCD command */
		GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

		/*RW =0 , for write to LCD */
		GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

		write_4_bits(0x03); //0000_0011


		mdelay(5);

		write_4_bits(0x03); //0000_0011

		udelay(150);

		write_4_bits(0x03); //0000_0011
		write_4_bits(0x02); //0000_0011

		//follow the flow chart in data sheet to complete the initialization
		lcd_send_command(LCD_CMD_4DL_2N_5X8F); // function set command Data_length, number_of_display_lines , font (pixels)

		//display control - > on /off
		lcd_send_command(LCD_CMD_DON_CURON);

		lcd_display_clear();

		//entry mode set
		lcd_send_command(LCD_CMD_INCADD);



}





static void write_4_bits(uint8_t value)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((value >> 0)& 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((value >> 1)& 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((value >> 2)& 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((value >> 3)& 0x1));

	//after writing every nibble instruct the LCD to latch that data inside the LCD  ... done using enable();
	lcd_enable();

}

static void lcd_enable(void){
	//here we have to do high to low transition on enable line...
	//when the LCD detects this transition from high to low ... it read the data from the data lines
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	udelay(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	udelay(100); // need to wait before sending next instruction / or check busy flag //instruction execution time

}

void lcd_display_clear(void){
	//display clear command :
	lcd_send_command(LCD_CMD_DIS_CLEAR);
	// we need ~2 ms delay after this
	mdelay(2);
}
/*Cursor returns to home position */
void lcd_display_return_home(void)
{

	lcd_send_command(LCD_CMD_DIS_RETURN_HOME);
	/*
	 * check page number 24 of datasheet.
	 * return home command execution wait time is around 2ms
	 */
	mdelay(2);
}
void lcd_set_cursor(uint8_t row,uint8_t column)
{
	// row value --> 1 or 2
	// column value --> 1 <> 16 ... possible hexa value is only 15 , F
	// we should decreement the column value first
	column--;
	if(row==1)
	{
		// in both the cases we we need to send command
		lcd_send_command(column|=(0x80));

	}
	else if(row==2)
	{
		lcd_send_command(column|=(0xC0));
	}
}

static void mdelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000); i++);
}

static void udelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1); i++);
}
