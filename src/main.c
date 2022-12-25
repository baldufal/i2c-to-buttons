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
#include <string.h> // For memset.
#include "twislave.h"

// Buffer for 16 values.
static uint8_t btn_values[2];

// Initializes outputs.
static void io_init() {
    // Make all pins inputs except for LED pin PCD.
    // No pull ups necessary because of external pull ups
    DDRB = 0x00;
    DDRC = 0x00;
    DDRD = 0x01;
}

// Reads all binary values.
static void readout() {
    btn_values[0] = PINB & 0b11000111; // Buttons are on PB0..2, PB6..7
    btn_values[1] = PIND & 0xFE; // PD0 is LED

    // insert PC1..PC3 into btn_values[0] bit positions 3..5
    uint8_t pin_c_1_to_3 = PINC & 0b00001110;
    btn_values[0] |= (pin_c_1_to_3 << 2);

    // insert PC0 into btn_values[1] at bit position 0
    btn_values[1] |= (PINC & 0x01);
}

int main(void) {
    // Set inputs/outputs.
    io_init();

    // Clear button values.
    memset(btn_values, 0, sizeof(btn_values));

    // Clear I2C register buffer.
    // We can't use memset: https://stackoverflow.com/questions/17144570/how-to-set-volatile-array-to-zero-using-memset
    for (int i = 0; i < i2c_buffer_size; i++) {
        i2cdata[i] = 0;
    }

    // 1 means no WDT reset occurred.
    i2cdata[0] |= I2C_BIT_WDT_RESET;

    // Enable watchdog to restart if we didn't reset it for 2 seconds.
    wdt_enable(WDTO_2S);
    // Enable I2C.
    init_twi_slave(I2C_SLAVE_ADDRESS);
    // Enable Interrupts.
    sei();

    uint16_t led_counter = 0;

    while (1) {
        // Calibrated to 1Hz.
        _delay_us(940);
        led_counter++;

        // Every 500ms execute this.
        if (led_counter > 500) {
            led_counter = 0;

            // Blink LED.
            PORTD ^= (1 << PD0);

            // Reset watchdog timer.
            wdt_reset();
            if (MCUCSR & (1 << WDRF)) {
                // A reset by the watchdog has occurred.
                // Signal this by clearing bit 2 in status byte.
                i2cdata[0] &= ~(I2C_BIT_WDT_RESET);
                // Clear flag for next time.
                MCUCSR &= ~(1 << WDRF);
            }
        }

        // If there is a request to read the buttons...
        if ((i2cdata[0] & I2C_BIT_READOUT)) {
            // Disabling interrupts, so i2cdata is kept consistent.
            cli();
            {
                readout();

                // Copy results to I2C buffer for readout.
                i2cdata[1] = btn_values[0];
                i2cdata[2] = btn_values[1];

                // Calculate checksum.
                i2cdata[3] = btn_values[0] + btn_values[1];

                // Clear bit one in status byte.
                i2cdata[0] &= ~I2C_BIT_READOUT;
            }
            sei();
        }
    }
}