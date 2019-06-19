/**
 *
 * HX711 library for Arduino
 * https://github.com/bogde/HX711
 *
 * MIT License
 * (c) 2018 Bogdan Necula
 *
**/
#define F_CPU 16000000UL
#include "HX711_2.h"
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Timer.h"
#include "UART_handler.h"
#define HIGH 1
#define LOW  0

#define LSBFIRST 0
#define MSBFIRST 1

#define PD_SCK    PORTC
#define PD_SCKLOW (PD_SCK & 0b10111111)
#define PD_SCKHIGH  (PD_SCK | (0b01000000))

#define ReadDOUT  ((PINC >> PINC5) & 0x01)
#define DOUT    PORTC
#define DOUT_MASK 0b00100000
#define DOUTLOW   (PD_SCK & 0b11011111)
#define DOUTHIGH  (PD_SCK | (HIGH << 5))
// TEENSYDUINO has a port of Dean Camera's ATOMIC_BLOCK macros for AVR to ARM Cortex M3.
//#define HAS_ATOMIC_BLOCK (defined(ARDUINO_ARCH_AVR) || defined(TEENSYDUINO))

// Whether we are running on either the ESP8266 or the ESP32.
//#define ARCH_ESPRESSIF (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))

// Whether we are actually running on FreeRTOS.
//#define IS_FREE_RTOS defined(ARDUINO_ARCH_ESP32)

// Define macro designating whether we're running on a reasonable
// fast CPU and so should slow down sampling from GPIO.
#define FAST_CPU 1

#include <util/atomic.h>


#if FAST_CPU
// Make shiftIn() be aware of clockspeed for
// faster CPUs like ESP32, Teensy 3.x and friends.
// See also:
// - https://github.com/bogde/HX711/issues/75
// - https://github.com/arduino/Arduino/issues/6561
// - https://community.hiveeyes.org/t/using-bogdans-canonical-hx711-library-on-the-esp32/539





//uint8_t PD_SCK; // Power Down and Serial Clock Input Pin
//uint8_t DOUT;   // Serial Data Output Pin
uint8_t GAIN_2;   // amplification factor
long OFFSET_2 = 0;  // used for tare weight
float SCALE_2 = 1;  // used to return weight in grams, kg, ounces, whatever



uint8_t shiftInSlow_2(uint8_t data,uint8_t clock,uint8_t bitOrder) {
	uint8_t value = 0;
	uint8_t i;


	PD_SCK = PD_SCKHIGH;
	_delay_us(1);
	value |= ReadDOUT << (7);
	PD_SCK = PD_SCKLOW;
	_delay_us(1);
	PD_SCK = PD_SCKHIGH;
	_delay_us(1);
	value |=ReadDOUT << (6);
	PD_SCK = PD_SCKLOW;
	_delay_us(1);
	PD_SCK = PD_SCKHIGH;
	_delay_us(1);
	value |= ReadDOUT << (5);
	PD_SCK = PD_SCKLOW;
	_delay_us(1);
	PD_SCK = PD_SCKHIGH;
	_delay_us(1);
	value |= ReadDOUT << (4);
	PD_SCK = PD_SCKLOW;
	_delay_us(1);
	PD_SCK = PD_SCKHIGH;
	_delay_us(1);
	value |= ReadDOUT << (3);
	PD_SCK = PD_SCKLOW;
	_delay_us(1);
	PD_SCK = PD_SCKHIGH;
	_delay_us(1);
	value |= ReadDOUT << (2);
	PD_SCK = PD_SCKLOW;
	_delay_us(1);
	PD_SCK = PD_SCKHIGH;
	_delay_us(1);
	value |= ReadDOUT << (1);
	PD_SCK = PD_SCKLOW;
	_delay_us(1);
	PD_SCK = PD_SCKHIGH;
	_delay_us(1);
	value |= ReadDOUT;
	PD_SCK = PD_SCKLOW;
	_delay_us(1);

	return value;
}
#define SHIFTIN_WITH_SPEED_SUPPORT(data,clock,order) shiftInSlow_2(data,clock,order)
#else
#define SHIFTIN_WITH_SPEED_SUPPORT(data,clock,order) shiftIn_2(data,clock,order)
#endif

#define DOUT_INPUT() do {DDRB &= ~(1 << DDB5); } while (0);
#define DOUT_OUTPUT()  do {DDRB = DDRB | (1 << DDB5)} while (0);

#define PD_SCK_INPUT() do {DDRC &= ~(1 << DDC6);} while (0);
#define PD_SCK_OUTPUT()  do {DDRC = DDRC | (1 << DDC6);} while (0);

void HX711_begin_2( uint8_t gain) {

	DOUT_INPUT();
	PD_SCK_OUTPUT();

	set_gain_2(gain);
}

bool is_ready_2() {
	return (DOUT & DOUT_MASK) >> 5 == LOW;
}

void set_gain_2(uint8_t gain) {
	switch (gain) {
	case 128:   // channel A, gain factor 128
		GAIN_2 = 1;
		break;
	case 64:    // channel A, gain factor 64
		GAIN_2 = 3;
		break;
	case 32:    // channel B, gain factor 32
		GAIN_2 = 2;
		break;
	}

	PD_SCK = PD_SCKLOW;
	read_2();
}

long read_2() {

	// Wait for the chip to become ready.
	wait_ready_2(0);

	// Define structures for reading data into.
	unsigned long value = 0;
	uint8_t data[3] = { 0 };
	uint8_t filler = 0x00;

	// Protect the read sequence from system interrupts.  If an interrupt occurs during
	// the time the PD_SCK signal is high it will stretch the length of the clock pulse.
	// If the total pulse time exceeds 60 uSec this will cause the HX711 to enter
	// power down mode during the middle of the read sequence.  While the device will
	// wake up when PD_SCK goes low again, the reset starts a new conversion cycle which
	// forces DOUT high until that cycle is completed.
	//
	// The result is that all subsequent bits read by shiftIn() will read back as 1,
	// corrupting the value returned by read().  The ATOMIC_BLOCK macro disables
	// interrupts during the sequence and then restores the interrupt mask to its previous
	// state after the sequence completes, insuring that the entire read-and-gain-set
	// sequence is not interrupted.  The macro has a few minor advantages over bracketing
	// the sequence between `noInterrupts()` and `interrupts()` calls.


	// Disable interrupts.
	cli();


	// Pulse the clock pin 24 times to read the data.
	data[2] = SHIFTIN_WITH_SPEED_SUPPORT(DOUT, PD_SCK, MSBFIRST);
	data[1] = SHIFTIN_WITH_SPEED_SUPPORT(DOUT, PD_SCK, MSBFIRST);
	data[0] = SHIFTIN_WITH_SPEED_SUPPORT(DOUT, PD_SCK, MSBFIRST);

	// Set the channel and the gain factor for the next reading using the clock pin.
	for (unsigned int i = 0; i < GAIN_2; i++) {
		PD_SCK = PD_SCKHIGH;

		PD_SCK = PD_SCKLOW;

	}


	// Enable interrupts again.
	sei();


	// Replicate the most significant bit to pad out a 32-bit signed integer
	if (data[2] & 0x80) {
		filler = 0xFF;
	} else {
		filler = 0x00;
	}

	// Construct a 32-bit signed integer
	value = ( (unsigned long)(filler) << 24
						| (unsigned long)(data[2]) << 16
						| (unsigned long)(data[1]) << 8
						| (unsigned long)(data[0]) );
//  if(value > 0)
//    PORTD ^= (1 << PORTD6);
//  else if(value < 0)
//    PORTD ^= (1 << PORTD7);
//  printVal("Hello: ");
//  printNum(value);
//  printVal("\r");
	return (long)(value);
}

void wait_ready_2(unsigned long delay_ms) {
	// Wait for the chip to become ready.
	// This is a blocking implementation and will
	// halt the sketch until a load cell is connected.
	while (!is_ready_2()) {
		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
		// https://github.com/bogde/HX711/issues/73
		delay(delay_ms);
	}
}

bool wait_ready_retry_2(int retries, unsigned long delay_ms) {
	// Wait for the chip to become ready by
	// retrying for a specified amount of attempts.
	// https://github.com/bogde/HX711/issues/76
	int count = 0;
	while (count < retries) {
		if (is_ready_2()) {
			return true;
		}
		delay(delay_ms);
		count++;
	}
	return false;
}

bool wait_ready_timeout_2(unsigned long timeout, unsigned long delay_ms) {
	// Wait for the chip to become ready until timeout.
	// https://github.com/bogde/HX711/pull/96
	unsigned long millisStarted = millis();
	while (millis() - millisStarted < timeout) {
		if (is_ready_2()) {
			return true;
		}
		delay(delay_ms);
	}
	return false;
}

long read_average_2(uint8_t times) {
	long sum = 0;
	for (uint8_t i = 0; i < times; i++) {
		sum += read_2();
		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
		// https://github.com/bogde/HX711/issues/73
		//_delay_ms(0);
	}
	return sum / times;
}

double get_value_2(uint8_t times) {
	return read_average_2(times) - OFFSET_2;
}

float get_units_2(uint8_t times) {
	return get_value_2(times) / SCALE_2;
}

void tare_2(uint8_t times) {
	double sum = read_average_2(times);
	set_offset_2(sum);
}

void set_scale_2(float scale) {
	SCALE_2 = scale;
}

float get_scale_2() {
	return SCALE_2;
}

void set_offset_2(long offset) {
	OFFSET_2 = offset;
}

long get_offset_2() {
	return OFFSET_2;
}

void power_down_2() {
	PD_SCK = PD_SCKLOW;
	PD_SCK = PD_SCKHIGH;
}

void power_up_2() {
	PD_SCK = PD_SCKLOW;
}
