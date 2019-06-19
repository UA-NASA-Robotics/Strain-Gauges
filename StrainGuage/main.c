/*
 * StrainGuage.c
 *
 * Created: 4/9/2019 5:18:41 PM
 * Author : John
 */
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "Timer.h"

#include "HX711.h"
#include "HX711_2.h"
#include "can.h"
#include "CANFastTransfer.h"
#include "UART_handler.h"

int main(void)
{
	//CKSEL = 0b0100
	PORTD = 0b00000000;//D7-D5 are debug LEDs, D4 is the button input.
	DDRD  = 0b11100000;
	DDRB  = 0b00000100; //save PB2 for switch input
	DDRC = DDRC |(1 << DDC4) | (1 << DDC1);
	PORTC &= ~((1 << PORTC1));
	PORTC |= (1 << PORTC4);
	USART_Init(9600);
	/* Replace with your application code */
	HX711_begin(128);
	initTimer0();
	can_init();
	initCANFastTransfer();
	_delay_ms(500);
	read();
	//printVal("Read: ");
	//printNum((long)read_average(20));
	read_average(20);
	//printVal("Get Value: ");
	//printNum((long)get_value(5));
	get_value(5);
	//printVal("Get Units: ");
	//printNum((long)get_units(5));
	get_units(5);
	//printVal("\r");
	set_scale(2280.f);

	tare(10);
	timer_t mytimer;
	setTimerInterval(&mytimer,1000);
	while (1)
	{

		if(timerDone(&mytimer))
		{
			long val = (get_units(5) +  get_units(5)) / 2;
			if(val > 0)
				PORTD ^= (1 << PORTD6);
			else if(val < 0)
				PORTD ^= (1 << PORTD7);
			printVal("Strain: ");
			printNum(val);
			printVal("\r");
			ToSendCAN(0, STRAIN_CAN_ADDRESS);
			ToSendCAN(3, val);
			sendDataCAN(5);   //Send to master

		}
	}
}

