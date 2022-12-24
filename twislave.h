/*
 * twislave.h
 *
* Created: 23-Dec-22
* Author : Falk
 * Source: https://rn-wissen.de/wiki/index.php/TWI_Slave_mit_avr-gcc
 */

#ifndef TWISLAVE_H_
#define TWISLAVE_H_

#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdint.h>

// I2C data buffer size.
// [0]: status/config
// [1]: first data byte
// [2]: second data byte
// [3]: checksum byte
//
#define i2c_buffer_size 4

// I2C slave address.
#define I2C_SLAVE_ADDRESS 0x3e

// Status register flags.
#define I2C_BIT_READOUT 0x01
#define I2C_BIT_WDT_RESET 0x02

volatile uint8_t i2cdata[i2c_buffer_size];

// Initializes TWI with the given address.
// I2C addresses are 7 bytes, init_twi_slave will shift the given address by one bit.
void init_twi_slave(uint8_t addr);

// Don't change below here

// Old versions of AVR-GCC do interrupt stuff differently.
#if (__GNUC__ * 100 + __GNUC_MINOR__) < 304
    #error "This library requires AVR-GCC 3.4.5 or later, update to newer AVR-GCC compiler"
#endif

//Schutz vor unsinnigen Buffergroessen
#if (i2c_buffer_size > 254)
    #error buffer size needs to be less than 254.
#endif

#if (i2c_buffer_size < 2)
    #error buffer size needs to be at least two bytes.
#endif

#endif /* TWISLAVE_H_ */