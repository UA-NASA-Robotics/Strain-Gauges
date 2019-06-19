/*
 * LEDs.c
 *
 * Created: 5/3/2017 11:13:47 PM
 *  Author: Zac
 */ 

#include "LEDs.h"
#include <avr/io.h>

void setLED(LEDs_ENUMED_t ledNumber, bool state)
{
	int ledToSwitch = 0;
	//HIGH SETS
	if(!state)
	{
		switch(ledNumber)
		{
			case LED0:
				PINB = PINB | LED0_P_HIGH;
				break;
			//LEDs on PORTF
			case LED1:
				ledToSwitch=LED1_P_HIGH;
				break;
			case LED2:
				ledToSwitch=LED2_P_HIGH;
				break;
			case LED3:
				ledToSwitch=LED3_P_HIGH;
				break;

		}
	}
	else
	{
			switch(ledNumber)
			{
				case LED0:
				PINB = PINB | LED0_P_LOW;
				break;
				//LEDs on PORTF
				case LED1:
				ledToSwitch=LED1_P_LOW;
				break;
				case LED2:
				ledToSwitch=LED2_P_LOW;
				break;
				case LED3:
				ledToSwitch=LED3_P_LOW;
				break;
				
			}
		
		
	}
	if(ledNumber != LID0)
	{
		unsigned int portRead;
		portRead = PIND;
		if(!state)
		{
			//Turn it high (OFF)
			PORTD = portRead | (ledToSwitch);
		}
		else
		{
			//Turn it low (ON)
			PORTD = portRead & (ledToSwitch);
		}
	}
	
}

void toggleLED(LEDs_ENUMED_t ledNumber)
{
	int ledToSwitch=0;
	switch(ledNumber)
	{
		case LED0:
			PINB=LED0_P_HIGH;
			break;
		//LEDs on PORTD
		case LED1:
			ledToSwitch=LED1_P_HIGH;
			break;
		case LED2:
			ledToSwitch=LED2_P_HIGH;
			break;
		case LED3:
			ledToSwitch=LED3_P_HIGH;
			break;
	}
	PIND = (ledToSwitch);
	//switch(ledNumber)
	//{
		//case LED1:
		//case LED2:
		//case LED10:
		//case LED11:
		//case LED12:
			////Toggle pin State
			//PINF = (ledToSwitch);
			//break;
		//case LED3:
		//case LED4:
		//case LED5:
		//case LED6:
		//case LED7:
		//case LED8:
		//case LED9:
			////Toggle pin State
			//PINA= (ledToSwitch);
			//break;
	//}
}