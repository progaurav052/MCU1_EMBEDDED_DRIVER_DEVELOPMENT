/*
 * ds1307.c
 *
 *  Created on: Mar 2, 2026
 *      Author: ggpai
 */
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "ds1307.h"

static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_config(void);
static uint8_t ds1307_read(uint8_t reg_addr);
static void ds1307_write(uint8_t value,uint8_t reg_addr);
static uint8_t binary_to_bcd(uint8_t value);


I2C_Handle_t g_ds1307I2cHandle;

uint8_t ds1307_init(void){

	// 1. init the i2c pins
	ds1307_i2c_pin_config(); // helper function to initialize the i2c pins

	// 2. i2c peripheral init
	ds1307_i2c_config();
	// 3. enable thei2c peripheral
	I2C_PeripheralControl(g_ds1307I2cHandle.pI2Cx, ENABLE);

	// 4. on initial power application to RTC module the CH will be 1 , we need to change this 0 .. for the oscillator to run
    // for this we need to write 1 to the 7 bit of reg_address 00h
	//use the I2C_master sendData function , NOTE USE the DS1307 doc to see how to write to an address
	ds1307_write(0x00,DS1307_ADDR_SEC);

	// 5. verify if CH is set to 0 after writing 0 in previous step

	uint8_t DS1307_ch_state = ds1307_read(DS1307_ADDR_SEC);


	return  ((DS1307_ch_state >> 7 ) & 0x1); // ch is 7th bit




}

void ds1307_set_current_time(RTC_Time_t *rtc_time){

	//here we have to go to the individual sec , min , hrs register and set the value provided by user config in BCD format

	//1. program the seconds field
	uint8_t seconds = rtc_time->seconds;
	//convert to bcd
	seconds = binary_to_bcd(seconds);
	//program this bcd format seconds in 00h register
	//make sure the 8th bit is still  0 for the oscillator to work
	seconds&=~(1 << 7);
	ds1307_write(seconds, DS1307_ADDR_SEC);

	//program the minutes
	ds1307_write(binary_to_bcd(rtc_time->minutes),DS1307_ADDR_MIN);

	//program the hour , 12 hr and 24 hr format
	hrs = binary_to_bcd(rtc_time->hours);

	if (rtc_time->time_format == TIME_FORMAT_24HRS) {
		hrs &= ~(1 << 6);
	} else {
		hrs |= (1 << 6);
		hrs = (rtc_time->time_format == TIME_FORMAT_12HRS_PM) ? hrs | (1 << 5) : hrs & ~(1 << 5);
	}

	ds1307_write(hrs, DS1307_ADDR_HRS);


}
void ds1307_get_current_time(RTC_Time_t *rtc_time){

	// we need to fetch seconds , min , hours , time format and store it back in the data structure
	uint8_t seconds,hrs;

		seconds = ds1307_read(DS1307_ADDR_SEC);

		seconds &= ~( 1 << 7);

		rtc_time->seconds = bcd_to_binary(seconds);
		rtc_time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));

		hrs = ds1307_read(DS1307_ADDR_HRS);
		if(hrs & ( 1 << 6)){
			//12 hr format
			rtc_time->time_format =  (hrs & ( 1 << 5)) ? TIME_FORMAT_12HRS_PM  	: TIME_FORMAT_12HRS_AM ;
			hrs &= ~(0x3 << 5);//Clear 6 and 5
		}else{
			//24 hr format
			rtc_time->time_format = TIME_FORMAT_24HRS;
		}

		rtc_time->hours = bcd_to_binary(hrs);
	}
}

void ds1307_set_current_date(RTC_Date_t *rtc_date){

	ds1307_write(binary_to_bcd(rtc_date->date), DS1307_ADDR_DATE);

	ds1307_write(binary_to_bcd(rtc_date->month), DS1307_ADDR_MONTH);

	ds1307_write(binary_to_bcd(rtc_date->year), DS1307_ADDR_YEAR);

	ds1307_write(binary_to_bcd(rtc_date->day), DS1307_ADDR_DAY);


}
void ds1307_get_current_date(RTC_Date_t *rtc_date){

	rtc_date->day = bcd_to_binary(ds1307_read(DS1307_ADDR_DAY));
	rtc_date->date = bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));
	rtc_date->month = bcd_to_binary(ds1307_read(DS1307_ADDR_MONTH));
	rtc_date->year = bcd_to_binary(ds1307_read(DS1307_ADDR_YEAR));
}

static void ds1307_i2c_pin_config(void)
{
	GPIO_Handle_t i2c_sda,i2c_scl;

	memset(&i2c_sda,0,sizeof(i2c_sda)); //best practice to do
	memset(&i2c_scl,0,sizeof(i2c_scl));

	/*
	 * I2C1_SCL ==> PB6
	 * I2C1_SDA ==> PB7
	 */

	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
	i2c_sda.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD_CONFIG;
	i2c_sda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&i2c_sda);


	i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
	i2c_scl.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD_CONFIG;
	i2c_scl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&i2c_scl);

}


static void ds1307_i2c_config(void)
{
	g_ds1307I2cHandle.pI2Cx = DS1307_I2C;
	g_ds1307I2cHandle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	g_ds1307I2cHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;
	I2C_Init(&g_ds1307I2cHandle);
}

static uint8_t ds1307_read(uint8_t reg_addr){
	// this function is done to read the value at particular reg_address of DS1307 chip
	// firstly i am using it to verify if the value at 00h reg_address after making ch=0 has actually worked
	// for this first make the internal address pointer point to the given address
	uint8_t data;
	I2C_MasterSendData(&g_ds1307I2cHandle, &reg_addr, 1, DS1307_I2C_ADDRESS,0);

	//receive
	I2C_MasterReceiveData(&g_ds1307I2cHandle, &data, 1, DS1307_I2C_ADDRESS,0);

	return data;


}
static void ds1307_write(uint8_t value,uint8_t reg_addr){

	// we need to send address first followed by value
	// by sending address first the internal address pointer of DS1307 chip will point to the reg_adress
	uint8_t tx[2];
	tx[0] = reg_addr;
	tx[1] = value;
	I2C_MasterSendData(&g_ds1307I2cHandle, tx, 2, DS1307_I2C_ADDRESS, 0);
}

static uint8_t binary_to_bcd(uint8_t value){

}
