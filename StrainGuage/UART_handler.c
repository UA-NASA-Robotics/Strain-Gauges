/*
 * UART_handler.c
 *
 * Created: 5/18/2017 2:59:16 PM
 *  Author: Zac
 */
#define F_CPU 16000000UL
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
void USART_Init( unsigned long baud) {

	LINBTR = LINBTR | 52;
	switch (baud)
	{
	case 115200:
		LINBRR = 50;
//      LINBRRH = 0x06;
//      LINBRRL =0x82;
		break;
	case 57600:
		LINBRR = 16;
		break;
	case 38400:
		LINBRR = 25;
		break;
	case 19200:
		LINBRR = 51;
		break;
	default:
		LINBRR = 53; //9600
	}
	LINCR = (1 << 3) | (0b101);
	//UCSR1C  = (0<<UMSEL1) | (0<<UPM1) | (0<<USBS1) | (1<<UCSZ10) | (1<<UCSZ11);
	//UCSR1B = (1<<RXEN1) | (1<<TXEN1) | (1<<RXCIE1);

	//cb_init(&UART1_TX_Buffer, UART1_TX_Buffer_Size, 1);
	//cb_init(&UART1_RX_Buffer, UART1_RX_Buffer_Size, 1);
}


void SendChar(char data)
{
	while(LINSIR & (1<<LBUSY));
	LINDAT = data;

	//_delay_us(100);
}
void printVal(char *str)
{
	while(*str)
		SendChar(*str++);

}
char str[16];
void printNum(long val)
{
	itoa(val, str, 10);
	printVal(str);
}
