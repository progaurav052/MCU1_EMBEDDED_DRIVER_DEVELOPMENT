/*
 * rtc_lcd.c
 *
 *  Created on: Mar 2, 2026
 *      Author: ggpai
 */
#include <stdio.h>
#include "ds1307.h"
#include "lcd.h"

#define SYSTICK_TIM_CLK 160000000UL
//function to initialize the Systick timer
void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1; // do the calculation on book , can be understood

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}
void mdelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000); i++);
}
char* get_day_of_week(uint8_t i)
{
	char* days[] = { "SUN","MON","TUES","WED","THUR","FRI","SAT"};

	return days[i-1];
}

void number_to_string(uint8_t num , char* buf)
{

	if(num < 10){
		buf[0] = '0';
		buf[1] = num+48;
	}else if(num >= 10 && num < 99)
	{
		buf[0] = (num/10) + 48; // this could be buf[3]
		buf[1]= (num % 10) + 48; // this could be buf[6] // what we send as parameter depends on it.
	}
}


//hh:mm:ss
char* time_to_string(RTC_Time_t *rtc_time)
{
	static char buf[9];

	buf[2]= ':';
	buf[5]= ':';

	number_to_string(rtc_time->hours,buf);
	number_to_string(rtc_time->minutes,&buf[3]);
	number_to_string(rtc_time->seconds,&buf[6]);

	buf[8] = '\0';

	return buf;

}

//dd/mm/yy
char* date_to_string(RTC_Date_t *rtc_date)
{
	static char buf[9];

	buf[2]= '/';
	buf[5]= '/';

	number_to_string(rtc_date->date,buf);
	number_to_string(rtc_date->month,&buf[3]);
	number_to_string(rtc_date->year,&buf[6]);

	buf[8]= '\0';

	return buf;

}

int main()
{
	RTC_Time_t current_time;
	RTC_Date_t current_date;


	printf("RTC test\n");
	if(ds1307_init())
	{
		//return 0 , meaning init has not happened succesfully  .. in our case ch bit is set
		printf("RTC init failed\n");
		while(1);


	}
	lcd_init();
	lcd_print_string("RTC test on LCD...");

    //1 interrupt for every 1 sec

	mdelay(3000);
	lcd_display_clear();
	lcd_display_return_home();


	//1. program current time and date info

	current_date.day=FRIDAY;
	current_date.date=5;
	current_date.month=3;
	current_date.year=26;

	current_time.hours=12;
	current_time.minutes=36;
	current_time.seconds=45;
	current_time.time_format=TIME_FORMAT_12HRS_AM;

	init_systick_timer(1);

	ds1307_set_current_date(&current_date);
	ds1307_set_current_time(&current_time);



	ds1307_get_current_date(&current_date);
	ds1307_get_current_time(&current_time);

	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS)
	{
		//print the time using AM and PM
		am_pm = (current_time.time_format) ? "PM" : "AM";
		printf("current time = %s %s\n", time_to_string(&current_time),am_pm); // return format : hh:mm:ss  AM/PM
		lcd_print_string(time_to_string(&current_time));
		lcd_print_string(am_pm);


	}
	else
	{
		printf("current time = %s\n", time_to_string(&current_time));
		lcd_print_string(time_to_string(&current_time));


	}
	lcd_set_cursor(2, 1);

	printf("Current date = %s <%s>\n",date_to_string(&current_date),get_day_of_week(current_date.day));

	lcd_print_string(date_to_string(&current_date));
	lcd_print_string(get_day_of_week(current_date.day));

	while(1);
	return 0;
}

void SysTick_Handler(void)
{
	RTC_Time_t current_time;
	RTC_Date_t current_date;


	ds1307_get_current_time(&current_time);


	char *am_pm;
	if (current_time.time_format != TIME_FORMAT_24HRS) {
		//print the time using AM and PM
		am_pm = (current_time.time_format) ? "PM" : "AM";
		printf("current time = %s %s\n", time_to_string(&current_time), am_pm); // return format : hh:mm:ss  AM/PM
		lcd_print_string(time_to_string(&current_time));
		lcd_print_string(am_pm);

	} else {
		printf("current time = %s\n", time_to_string(&current_time));
	}
	lcd_set_cursor(2, 1);
	printf("Current date = %s <%s>\n", date_to_string(&current_date),get_day_of_week(current_date.day));

	lcd_print_string(date_to_string(&current_date));
	lcd_send_char('<');
	lcd_print_string(get_day_of_week(current_date.day));
	lcd_send_char('>');

}
/*
 * rtc_ds1307_stm_lcd.c
 *
 *  Created on: Mar 8, 2026
 *      Author: ggpai
 */


