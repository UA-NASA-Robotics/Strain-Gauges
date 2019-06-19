/*
 * Timer.c
 *
 * Created: 5/5/2017 12:39:32 AM
 *  Author: Zac
 */

#include "Timer.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "LEDs.h"

#define PRESCALER1    (1<<CS00)
#define PRESCALER8    (1<<CS01)       //TOO FAST
#define PRESCALER64   (1<<CS01)|(1<<CS00)   //TIMER INTERVAL ISR 1ms
#define PRESCALER256  (1<<CS02)       //TIMER INTERVAL ISR 4ms
#define PRESCALER1024 (1<<CS02)|(1<<CS00)   //TIMER INTERVAL ISR 16ms

#define PRESCALER_ZERO  PRESCALER256
#define PRESCALER_ZEROUSING 256
#define PRESCALER_ONE PRESCALER1
#define PRESCALER_ONEUSING 1

unsigned long long globalTime=0;

void setTimerInterval(timer_t * t, unsigned long l)
{
	t->timerLength=l;
}

unsigned long long millis(void)
{
	return globalTime;

}

void resetTimer(timer_t * t)
{
	t->prevTime=globalTime;
}

bool timerDone(timer_t * t)
{
	if(globalTime >= t->prevTime+t->timerLength)
	{
		t->prevTime=globalTime;
		return true;
	}
	else
	{
		return false;
	}
}
timer_t delayTimer;
void delay(int val)
{
	setTimerInterval(&delayTimer, val);
	while(!timerDone(&delayTimer));

}
void initTimer0(void)
{
	TCCR0B = PRESCALER_ZERO;  //Set the prescaler
	TIMSK0 = (1<<TOIE0);    //ENABLE Timer Overflow interrupt
}

void initTimer1(void)
{
	TCCR1B = PRESCALER_ONE;   //Set the prescaler
	TIMSK1 = (1<<TOIE1);    //ENABLE Timer Overflow interrupt
	initTimer1OCB();
}

void initTimer1OCB(void)
{
	//configuring for fast PWM mode
	TCCR1A = (1<<COM1B1) | (0<<COM1B0); //Clear OC1B on compare match, set at TOP
	//WGM1 is configured to have a 10-bit resolution for FAST PWM.
	//set WGM to 0b0110 for 9bit, or 0b0101 for 8bit resolution
	TCCR1A |= (0<<WGM13) | (1<<WGM12) | (1<<WGM11) |(1<<WGM10);
	//write OCR1B to 0 to ensure PWM analog voltage starts at 0 volts
	OCR1B = 0;
	//probably don't want to disable
	//TIMSK1 |= (1<<OCIE1B);
}

void setOC1BPulseWidth(int input)
{
	//input is a value between 0 and 1000
	//think of it as an interpreted percentage
	//from 0% to 100.0%
	//NOTE: max input is really 1023=100%

	if(input>400)  //732
	{
		input = 400;
	}
	OCR1B = input;
}


ISR(TIMER0_OVF_vect)
{
#if (PRESCALER_ZEROUSING == 1024)
	globalTime+=16;
#elif (PRESCALER_ZEROUSING == 256)
	globalTime+=4;
#elif (PRESCALER_ZEROUSING == 8)
	globalTime++;
#endif
	TIFR0 = (0<<TOV0);  //Reset timer0 overflow interrupt flag
}

ISR(TIMER1_OVF_vect)
{

	TIFR1 = (0<<TOV1);  //Reset timer0 overflow interrupt flag
}