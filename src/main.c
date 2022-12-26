/*
* DHTExpander.c
*
* Created: 23-Dec-22
* Author : Falk
*/

/*
Translates from I2C to 16 binary inputs (buttons).
14 of them are pulled up, so the other pin of the button connects to ground.
2 of the inputs (PD7, PB0) are inverted (pulled down, button connects to vcc).

General operation:
0. Blink LED 1 (PC3) every 500 ms (shows operation)
1. Wait for input signal from I2C.
Setting bit 0 via I2C in "register" 0x00 triggers a readout.
Once readout is complete, bit reads as 0 again.
This bit is also automatically set after register 0x03 is read.

Bit 1 in register 0x00 signals if no reset by the watchdog has occurred.
The initial state is high, meaning no watchdog reset.
If it gets cleared by the system it should be reset by the user
to detect the next watchdog reset which is unintended behavior.

2. "Register" 0x00 is evaluated frequently.

Data format in registers 0x01 to 0x03:
Two bytes of binary values, followed by a checksum (R0x01 + R0x02).

I2C behavior:
Register address auto-increment is implemented.
If an address beyond the register file is requested, the address is set to 0x00 instead.

I2C implementation is mostly taken from https://rn-wissen.de/wiki/index.php/TWI_Slave_mit_avr-gcc.

This is intended to run on some 8MHz ATmega8.
Running on 3.3V is recommended due to i2c safety when operating with raspberry pi.

Pin assignments
PC4: SDA
PC5: SCL

PD0: LED

PD7: inverted Button
PB0: inverted Button
PC0-3: Buttons
PD1-6: Buttons
PB1-2: Buttons
PB6-7: Buttons
*/

#ifndef F_CPU
# define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "twislave.h"

// Initializes outputs.
static void io_init() {
	// Make all pins inputs except for LED pin PC0.
	// No pull ups necessary because of external pull ups.
	// We use pull ups anyway to make debugging without external pull ups easier.
	DDRB = 0x00;
	PORTB = 0b11000110;
	DDRC = 0x00;
	PORTC = 0b00001111;
	DDRD = 0x01;
	PORTD = 0b01111110;
	
	// Enable LED.
	PORTD |= (1 << PD0);
}

/*Function to left rotate n by d bits*/
uint8_t leftRotate(uint8_t n, uint8_t d)
{
	/* In n<<d, last d bits are 0. To put first 3 bits of n
	at last, do bitwise or of n<<d with n >>(INT_BITS -
	d) */
	return (n << d) | (n >> (8 - d));
}

static void readpins(uint8_t* data){
	// Buttons are on PB0..2, PB6..7.
	data[0] = PINB & 0b11000111;
	// Insert PC1..PC3 into btn_values[0] bit positions 3..5.
	data[0] |= ((PINC & 0b00001110) << 2);
	
	// PD0 is LED, insert PC0 instead.
	data[1] = (PIND & 0xFE) | (PINC & 0x01);
}

// Reads all binary values 3 times
// & computes most frequent values
// & copies values to i2c registers.
// Interrupts must be disabled.
static void readout() {
	uint8_t btn_values[6];
	uint8_t firstByte;
	uint8_t secondByte;
	uint8_t checksum;
	
	// Read pins 3 times during 1ms.
	readpins(btn_values);
	_delay_us(500);
	readpins(btn_values+2);
	_delay_us(500);
	readpins(btn_values+4);
	
	
	// Compute majority.
	firstByte = (btn_values[0] & btn_values[2]) |
	(btn_values[0] & btn_values[4]) |
	(btn_values[2] & btn_values[4]);
	secondByte = (btn_values[1] & btn_values[3]) |
	(btn_values[1] & btn_values[5]) |
	(btn_values[3] & btn_values[5]);

	// Calculate checksum.
	checksum = leftRotate(firstByte + leftRotate(secondByte, 3), 3);
	
	// Write to i2c buffer.
	i2cdata[1] = firstByte;
	i2cdata[2] = secondByte;
	i2cdata[3] = checksum;
	// Redundancy.
	i2cdata[4] = leftRotate(firstByte, 3);
	i2cdata[5] = leftRotate(secondByte, 3);
	i2cdata[6] = leftRotate(checksum, 3);
	// Redundancy.
	i2cdata[7] = leftRotate(firstByte, 6);
	i2cdata[8] = leftRotate(secondByte, 6);
	i2cdata[9] = leftRotate(checksum, 6);
}

int main(void) {
	// Set inputs/outputs.
	io_init();

	// 1 means no WDT reset occurred.
	i2cdata[0] = I2C_BIT_WDT_RESET;

	// Enable watchdog to restart if we didn't reset it for 2 seconds.
	wdt_enable(WDTO_2S);
	// Enable I2C.
	init_twi_slave(I2C_SLAVE_ADDRESS);
	// Read first time to have values at first i2c read.
	readout();
	// Enable Interrupts.
	sei();

	while (1) {
		
		// Reset watchdog timer.
		wdt_reset();
		if (MCUCSR & (1 << WDRF)) {
			// A reset by the watchdog has occurred.
			// Signal this by clearing bit 2 in status byte.
			i2cdata[0] &= ~(I2C_BIT_WDT_RESET);
			// Clear flag for next time.
			MCUCSR &= ~(1 << WDRF);
		}
			

		// If there is a request to read the buttons...
		if ((i2cdata[0] & I2C_BIT_READOUT)) {
			// Disabling interrupts, so i2cdata is kept consistent.
			cli();
			{
				// Disable LED.
				PORTD &= ~(1 << PD0);
				
				
				// Read and write values to i2c registers.
				readout();

				// Clear bit one in status byte.
				i2cdata[0] &= ~I2C_BIT_READOUT;
				
				// Enable LED.
				PORTD |= (1 << PD0);
			}
			sei();
		}
	}
}