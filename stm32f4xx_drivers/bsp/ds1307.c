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

}
void ds1307_get_current_time(RTC_Time_t *rtc_time){

}

void ds1307_set_current_date(RTC_Date_t *rtc_date){

}
void ds1307_get_current_date(RTC_Date_t *rtc_date){

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
	uint8_t ch_state;
	I2C_MasterSendData(&g_ds1307I2cHandle, &reg_addr, 1, DS1307_I2C_ADDRESS,0);

	//receive
	I2C_MasterReceiveData(&g_ds1307I2cHandle, &ch_state, 1, DS1307_I2C_ADDRESS,0);

	return ch_state;


}
static void ds1307_write(uint8_t value,uint8_t reg_addr){

	// we need to send address first followed by value
	// by sending address first the internal address pointer of DS1307 chip will point to the reg_adress
	uint8_t tx[2];
	tx[0] = reg_addr;
	tx[1] = value;
	I2C_MasterSendData(&g_ds1307I2cHandle, tx, 2, DS1307_I2C_ADDRESS, 0);
}

