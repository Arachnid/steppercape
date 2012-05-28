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

#define NUM_REGISTERS 7

#define STALL_STOP 4

typedef union {
    uint8_t bytes[NUM_REGISTERS];
    struct {
        uint8_t slave_addr;		  // I2C slave address
        // [ -, -, STALL_STOP, -, CS12, CS11, CS10]
        uint8_t clk_flags;        // Timer enable and prescaler selection
        uint16_t step_interval;   // Timer steps per stepper step
        // [ -, -, I1_INVERT, I0_INVERT, S1_I1_STOP, S1_I0_STOP, S0_I1_STOP, S0_I0_STOP]
        struct {
			unsigned int s0_stop : 1;
			unsigned int s1_stop : 1;
			unsigned int reserved1 : 2;
			unsigned int i0_invert : 1;
			unsigned int i1_invert : 1;
		} limit_flags;
        //uint8_t limit_flags;	  // Limit switch flags
        uint8_t microstep;		  // Microstep settings
        uint8_t buffer_free;	  // Free space in buffer
        uint8_t buffer_append;	  // Appends to the input buffer
    } __attribute__((__packed__)) reg;
} register_t;
