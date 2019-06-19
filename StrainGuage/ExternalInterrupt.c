/*
 * ExternalInterrupt.c
 *
 * Created: 5/13/2017 7:54:52 PM
 *  Author: reed
 */ 
#include "ExternalInterrupt.h"
#include <avr/io.h>
#include <avr/interrupt.h>
long counts = 0;

void initExternalInt()
{
	//enable interrupt 2 and have it trigger on the rising edge.
	//change 1<<ISC20 to 0<<ISC20 to have it on falling edge. 
	EICRA = (1<<ISC21) | (1<<ISC20); 
	//enable interrupt for external interrupt 2. 
	EIMSK = (1<<INT2); 
}
void returnCounts()
{
	
}


ISR(INT2_vect)
{
	counts++; 
	//toggleLED(5);
}