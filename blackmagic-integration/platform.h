/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef BLACKMAGIC_PLATFORM_H_
#define BLACKMAGIC_PLATFORM_H_

// Avoid includes that bring in ch.h from ChibiOS here since there
// are collisions with variables and defines in blackmagic.
#include <stdint.h>
#include <stdbool.h>
#include <timing.h>

typedef void* GpioPort_t;
typedef uint32_t GpioPin_t;

// Global variables
extern GpioPort_t platform_swdio_port;
extern GpioPin_t platform_swdio_pin;
extern GpioPort_t platform_swclk_port;
extern GpioPin_t platform_swclk_pin;

// Re-declared to avoid including commands.h
extern void commands_printf(const char* format, ...);

#define PLATFORM_HAS_DEBUG
#define PLATFORM_PRINTF commands_printf

#define SWDIO_PORT 	platform_swdio_port
#define SWDIO_PIN	platform_swdio_pin
#define SWCLK_PORT 	platform_swclk_port
#define SWCLK_PIN	platform_swclk_pin

void platform_set_default_pins(void);
void platform_set_nrf5x_pins(void);

void gpio_set(GpioPort_t port, GpioPin_t pin);
void gpio_clear(GpioPort_t port, GpioPin_t pin);
void gpio_set_val(GpioPort_t port, GpioPin_t pin, bool val);
bool gpio_get(GpioPort_t port, GpioPin_t pin);

void SWDIO_MODE_FLOAT(void);
void SWDIO_MODE_DRIVE(void);

/* Global variable used to blackmagic to delay execution a bit.
   Set to zero since depending on the optimization level the
   compiler might remove the delay anyway. */
static const uint32_t swd_delay_cnt = 0;

#endif /* BLACKMAGIC_PLATFORM_H_ */
