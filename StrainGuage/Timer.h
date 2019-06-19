/*
 * Timer.h
 *
 * Created: 5/5/2017 12:39:43 AM
 *  Author: Zac
 */


#ifndef TIMER_H_
#define TIMER_H_

#include <stdbool.h>

typedef struct {
	unsigned long long timerLength;
	unsigned long long prevTime;
} timer_t;


void delay(int val);
void initTimer0(void);
void initTimer1(void);
void initTimer1OCB(void);
void setOC1BPulseWidth(int input);
unsigned long long millis(void);


bool timerDone(timer_t * t);
void setTimerInterval(timer_t * t, unsigned long l);
void resetTimer(timer_t * t);



#endif /* TIMER_H_ */