/*
 * LEDs.h
 *
 * Created: 5/3/2017 11:13:56 PM
 *  Author: Zac
 */ 


#ifndef LEDS_H_
#define LEDS_H_

#include <stdbool.h>

#define NUMBER_OF_LEDS 4

#define ON  true
#define OFF false

#define Test1 3
#define Test2 4
#define Test3 5


typedef enum{
	LED0=0,
	LED1,
	LED2,
	LED3
}LEDs_ENUMED_t;

//'Top LEDs'
#define LED0_P_HIGH 0x04	//2 //B2	-	0
#define LED1_P_HIGH	0x80	//7	//D7	-	1
#define LED2_P_HIGH	0x40	//6	//D6	-	2
#define LED3_P_HIGH	0x20	//5	//D5	-	3

#define LED0_P_LOW	0xFB;	//2 //B2	-	0
#define LED1_P_LOW	0x7F	//7	//D7	-	1
#define LED2_P_LOW	0xBF	//6	//D6	-	2
#define LED3_P_LOW	0xDF	//5	//D5	-	3


void setLED(LEDs_ENUMED_t ledNumber, bool state);
void toggleLED(LEDs_ENUMED_t ledNumber);

#endif /* LEDS_H_ */