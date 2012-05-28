/*
 * Stepper Cape - an I2C based stepper driver cape for the Beaglebone
 * Copyright (C) 2012 Nick Johnson
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define F_CPU 8000000UL  // 8 MHz

#include <util/delay.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "register.h"
#include "usiTwiSlave.h"

FUSES = {
	.low = FUSE_SUT0 & FUSE_CKSEL3 & FUSE_CKSEL1 & FUSE_CKSEL0,
	.high = FUSE_SPIEN,
};

#define TRUE 1
#define FALSE 0

#define DDR_STEPPER0 DDRB
#define PORT_STEPPER0 PORTB
#define STEPPER0_DIR PB0
#define STEPPER0_EN PB6
#define STEPPER0_STEP PB1
#define STEPPER0_MS0 PB2
#define STEPPER0_MICROSTEP_MASK ((uint8_t)~(_BV(PB4) | _BV(PB3) | _BV(PB2)))

#define DDR_STEPPER1 DDRD
#define PORT_STEPPER1 PORTD
#define STEPPER1_DIR PD0
#define STEPPER1_EN PD5
#define STEPPER1_STEP PD1
#define STEPPER1_MS0 PD2
#define STEPPER1_MICROSTEP_MASK ((uint8_t)~(_BV(PD4) | _BV(PD3) | _BV(PD2)))

#define PORT_INPUTS PORTA
#define INPUT0 PA0
#define INPUT1 PA1

#define ENABLE_PWM_SETUP() (TCCR1A = 0, TCCR1B = _BV(WGM12), TIMSK |= _BV(TOIE1))

typedef uint8_t (*register_write_handler)(uint8_t reg, uint8_t oldval);

// Predefinitions of read/write handlers so we can reference them
uint8_t i2c_read(uint8_t reg, volatile uint8_t *value);
uint8_t i2c_write(uint8_t reg, uint8_t value);
void main(void) __attribute__((noreturn));

#define copy_bit(from_reg, from_bit, to_reg, to_bit) if((from_reg) & _BV(from_bit)) { \
    to_reg |= _BV(to_bit); \
} else { \
    to_reg &= ~_BV(to_bit); \
}

register_t registers;

#define BUFFER_CAPACITY 32
volatile uint8_t buffer[BUFFER_CAPACITY];
volatile uint8_t buffer_head = 0; // Oldest valid element
volatile uint8_t buffer_tail = 0; // First empty slot
volatile uint8_t buffer_nybble = 0;

// EEPROM copy of registers; updated on write to register 127.
register_t EEMEM eeprom_registers = {
    .reg = {
        .slave_addr = 0x26,
        .clk_flags = {
			.stall_stop = 0,
			.cs = 0,
		},
        .step_interval = 0,
        .limit_flags = {
			.s0_stop = 0,
			.s1_stop = 0,
			.i0_invert = 0,
			.i1_invert = 0,
		},
        .microstep = {
			.stepper0 = 0,
			.stepper1 = 0,
		},
        .buffer_free = BUFFER_CAPACITY,
        .buffer_append = 0
    }
};

volatile uint8_t eeprom_dirty = FALSE;

uint8_t write_noop(uint8_t reg, uint8_t oldval) {
	return TRUE;
}

uint8_t write_ignore(uint8_t reg, uint8_t oldval) {
	registers.bytes[reg] = oldval;
	return TRUE;
}

uint8_t write_slave_addr(uint8_t reg, uint8_t oldval) {
    usiTwiSlaveInit(registers.reg.slave_addr, i2c_read, i2c_write);
	return TRUE;
}

uint8_t write_clk_flags(uint8_t reg, uint8_t oldval) {
	// Copy lower 3 bits to clock select flags
	TCCR1B = (TCCR1B & 0xF8) | registers.reg.clk_flags.cs;
	return TRUE;
}

uint8_t write_step_interval_msb(uint8_t reg, uint8_t oldval) {
	// Update counter max value
	OCR1A = registers.reg.step_interval;
	return TRUE;
}

uint8_t write_limit_flags(uint8_t reg, uint8_t oldval) {
	if(registers.reg.limit_flags.i0_pullup) {
		PORT_INPUTS |= _BV(INPUT0);
	} else {
		PORT_INPUTS &= ~_BV(INPUT0);
	}
	if(registers.reg.limit_flags.i1_pullup) {
		PORT_INPUTS |= _BV(INPUT1);
	} else {
		PORT_INPUTS &= ~_BV(INPUT1);
	}
	return TRUE;
}

uint8_t write_microstep(uint8_t reg, uint8_t oldval) {
	// Update output pins with respective microstep settings
	PORT_STEPPER0 = (PORT_STEPPER0 & STEPPER0_MICROSTEP_MASK) | registers.reg.microstep.stepper0;
	PORT_STEPPER1 = (PORT_STEPPER1 & STEPPER1_MICROSTEP_MASK) | registers.reg.microstep.stepper1;
	return TRUE;
}

uint8_t buffer_append(uint8_t reg, uint8_t oldval) {
	buffer[buffer_tail] = registers.reg.buffer_append;
	registers.reg.buffer_append = 0;
	buffer_tail = (buffer_tail + 1) % BUFFER_CAPACITY;
	registers.reg.buffer_free -= 1;
	// Only ack if the buffer is not full
	return registers.reg.buffer_free;
}

// 00 = disable
// 01 = stop
// 10 = step back
// 11 = step forward

ISR(TIMER1_OVF_vect) {
	// Step pins low
	PORT_STEPPER0 &= ~_BV(STEPPER0_STEP);
	PORT_STEPPER1 &= ~_BV(STEPPER1_STEP);

	if(registers.reg.buffer_free < BUFFER_CAPACITY) {
		// Fetch current instruction
		uint8_t data = (buffer[buffer_head] >> (buffer_nybble << 2)) & 0xF;
		buffer_head = (buffer_head + buffer_nybble) % BUFFER_CAPACITY;

		// If we're about to make the buffer non-full, ack the pending write
		if(registers.reg.buffer_free == 0 && buffer_nybble)
			usiTwiSlaveAck();

		registers.reg.buffer_free += buffer_nybble;
		buffer_nybble ^= 1;
		
		uint8_t in0 = ((PORT_INPUTS & _BV(INPUT0)) != 0) ^ registers.reg.limit_flags.i0_invert;
		uint8_t in1 = ((PORT_INPUTS & _BV(INPUT1)) != 0) ^ registers.reg.limit_flags.i1_invert;
		if(!in0 || !registers.reg.limit_flags.s0_stop) {
			if(data & 0x2) {
				// Step instruction
				copy_bit(data, 0, PORT_STEPPER0, STEPPER0_DIR);
				PORT_STEPPER0 &= ~_BV(STEPPER0_EN);
				PORT_STEPPER0 |= _BV(STEPPER0_STEP);
			} else {
				// Stop or disable instruction
				copy_bit(~data, 0, PORT_STEPPER0, STEPPER0_EN);
			}
		}

		if(!in1 || !registers.reg.limit_flags.s1_stop) {
			if(data & 0x8) {
				// Step instruction
				copy_bit(data, 2, PORT_STEPPER1, STEPPER1_DIR);
				PORT_STEPPER1 &= ~_BV(STEPPER1_EN);
				PORT_STEPPER1 |= _BV(STEPPER1_STEP) | _BV(STEPPER1_EN);
			} else {
				// Stop or disable instruction
				copy_bit(~data, 2, PORT_STEPPER1, STEPPER1_EN);
			}
		}
	} else {
		// Buffer underrun! Stall!
		if(registers.reg.clk_flags.stall_stop) {
			// Stop the timer when the buffer is empty
			registers.reg.clk_flags.cs = 0;
			TCCR1B &= 0xF8;
		}
	}
}

const register_write_handler write_handlers[] = {
    write_slave_addr,
    write_clk_flags,
    write_noop,
    write_step_interval_msb,
    write_limit_flags,
    write_microstep,
    write_ignore,
    buffer_append,
};

uint8_t i2c_read(uint8_t reg, volatile uint8_t *value) {
    if(reg < NUM_REGISTERS) {
        *value = registers.bytes[reg];
    } else {
		*value = 0;
    }
    // Send ACK
    return TRUE;
}

uint8_t i2c_write(uint8_t reg, uint8_t value) {
	uint8_t ret = 1;
    if(reg < NUM_REGISTERS) {
//		ret = ((register_write_handler)pgm_read_word_near(write_handlers + reg))(reg, &value);
		uint8_t oldval = registers.bytes[reg];
		registers.bytes[reg] = value;
        ret = write_handlers[reg](reg, oldval);
    } else if(reg == 127) {
        // Update eeprom (asynchronously, so we don't block the interrupt).
        eeprom_dirty = TRUE;
    } else {
        // Invalid register - do nothing.
    }
    return ret;
}

void ioinit(void) {
	// Set all the output pins
    DDR_STEPPER0 |= _BV(STEPPER0_DIR) | _BV(STEPPER0_EN) |
                    _BV(STEPPER0_STEP) | _BV(STEPPER0_MS0) |
                    _BV(STEPPER0_MS0 + 1) | _BV(STEPPER0_MS0 + 2);
    DDR_STEPPER1 |= _BV(STEPPER1_DIR) | _BV(STEPPER1_EN) |
                    _BV(STEPPER1_STEP) | _BV(STEPPER1_MS0) |
                    _BV(STEPPER1_MS0 + 1) | _BV(STEPPER1_MS0 + 2);
                    
    // Set enable pins to 1, since they're active-low
    PORT_STEPPER0 |= _BV(STEPPER0_EN);
    PORT_STEPPER1 |= _BV(STEPPER1_EN);
    
    // Setup PWM: Clear timer on compare. Don't enable it yet.
    ENABLE_PWM_SETUP();
}

void read_registers(void) {
    // Initialize from eeprom
    eeprom_read_block(&registers, &eeprom_registers, sizeof(registers) - 1);

    // Run all the write funcs in order to initialize the device
    // But don't write to the buffer append register!
    for(int i = 0; i < NUM_REGISTERS - 1; i++)
        write_handlers[i](i, registers.bytes[i]);

	registers.reg.buffer_free = BUFFER_CAPACITY;
}

void main(void) {
    ioinit();
    read_registers();
    // Enable interrupts
    sei();
    
    for(;;) {
        if(eeprom_dirty) {
            eeprom_write_block(&registers, &eeprom_registers, sizeof(registers));
            eeprom_dirty = FALSE;
        }
    }
}
