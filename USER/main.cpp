#include "math.h"
#include "string.h"
#include "ctype.h"
#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "stdio.h"
#include "servo.h"
#include "mLog.h"
#include "adc.h"


#define PRESS_MIN	20
#define PRESS_MAX	6000

#define VOLTAGE_MIN 150
#define VOLTAGE_MAX 3300

u8 state = 0;
u16 val = 0;
u16 value_AD = 0;

long PRESS_AO = 0;
int VOLTAGE_AO = 0;

long map(long x, long in_min, long in_max, long out_min, long out_max);

int main(void)
{		
	delay_init();	
	NVIC_Configuration(); 	 
	uart_init(9600);	 
	Adc_Init();

	delay_ms(1000);

	USARTx_printf(USART1, "Test start\r\n");
	while(1)
	{
		value_AD = Get_Adc_Average(1,10);	
		VOLTAGE_AO = map(value_AD, 0, 4095, 0, 3300);
		if(VOLTAGE_AO < VOLTAGE_MIN)
		{
			PRESS_AO = 0;
		}
		else if(VOLTAGE_AO > VOLTAGE_MAX)
		{
			PRESS_AO = PRESS_MAX;
		}
		else
		{
			PRESS_AO = map(VOLTAGE_AO, VOLTAGE_MIN, VOLTAGE_MAX, PRESS_MIN, PRESS_MAX);
		}
		USARTx_printf(USART1, "%d %d %d \r\n",value_AD,VOLTAGE_AO,PRESS_AO);	
						
		delay_ms(500);
	}

}


long map(long x, long in_min, long in_max, long out_min, long out_max) {
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
