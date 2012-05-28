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

#define NUM_REGISTERS 8

typedef union {
    uint8_t bytes[NUM_REGISTERS];
    struct {
        uint8_t slave_addr;		  // I2C slave address
        struct {
			unsigned int cs : 3;
			unsigned int : 1;
			unsigned int stall_stop : 1;
		} clk_flags;			  // Timer enable and prescaler selection
        uint16_t step_interval;   // Timer steps per stepper step
        struct {
			unsigned int s0_stop : 1;
			unsigned int s1_stop : 1;
			unsigned int : 2;
			unsigned int i0_invert : 1;
			unsigned int i1_invert : 1;
			unsigned int i0_pullup : 1;
			unsigned int i1_pullup : 1;
		} limit_flags;
		struct {
			unsigned int stepper0 : 3;
			unsigned int : 1;
			unsigned int stepper1 : 3;
		} microstep;			  // Microstep settings
        uint8_t buffer_free;	  // Free space in buffer
        uint8_t buffer_append;	  // Appends to the input buffer
    } __attribute__((__packed__)) reg;
} register_t;
