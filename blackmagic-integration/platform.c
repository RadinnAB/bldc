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

#include "platform.h"
#include "ch.h"
#include "hal.h"
#include "commands.h"
#include "hw.h"

#define SWDIO_PORT_DEFAULT		GPIOA
#define SWDIO_PIN_DEFAULT		13
#define SWCLK_PORT_DEFAULT	 	GPIOA
#define SWCLK_PIN_DEFAULT		14

// Use variables for ports and pins, so that they can be changed
GpioPort_t platform_swdio_port = SWDIO_PORT_DEFAULT;
GpioPin_t platform_swdio_pin = SWDIO_PIN_DEFAULT;
GpioPort_t platform_swclk_port = SWCLK_PORT_DEFAULT;
GpioPin_t platform_swclk_pin = SWCLK_PIN_DEFAULT;

void platform_delay(uint32_t ms) {
	chThdSleepMilliseconds(ms);
}

uint32_t platform_time_ms(void) {
	return ST2MS(chVTGetSystemTimeX());
}

void platform_srst_set_val(bool assert) {
	(void)assert;
}

bool platform_srst_get_val(void) {
	return false;
}

void platform_set_default_pins(void) {
	platform_swdio_port = SWDIO_PORT_DEFAULT;
	platform_swdio_pin = SWDIO_PIN_DEFAULT;
	platform_swclk_port = SWCLK_PORT_DEFAULT;
	platform_swclk_pin = SWCLK_PIN_DEFAULT;
}

void platform_set_nrf5x_pins(void) {
#ifdef NRF5x_SWDIO_GPIO
	platform_swdio_port = NRF5x_SWDIO_GPIO;
	platform_swdio_pin = NRF5x_SWDIO_PIN;
	platform_swclk_port = NRF5x_SWCLK_GPIO;
	platform_swclk_pin = NRF5x_SWCLK_PIN;
#endif
}

void gpio_set(GpioPort_t port, GpioPin_t pin) {
    palSetPad((stm32_gpio_t*)port, pin);
    palSetPad((stm32_gpio_t*)port, pin);
    palSetPad((stm32_gpio_t*)port, pin);
}

void gpio_clear(GpioPort_t port, GpioPin_t pin) {
    palClearPad((stm32_gpio_t*)port, pin);
    palClearPad((stm32_gpio_t*)port, pin);
    palClearPad((stm32_gpio_t*)port, pin);
}

void gpio_set_val(GpioPort_t port, GpioPin_t pin, bool val) {
    palWritePad((stm32_gpio_t*)port, pin, val ? PAL_HIGH : PAL_LOW);
    palWritePad((stm32_gpio_t*)port, pin, val ? PAL_HIGH : PAL_LOW);
    palWritePad((stm32_gpio_t*)port, pin, val ? PAL_HIGH : PAL_LOW);
}

bool gpio_get(GpioPort_t port, GpioPin_t pin) {
    return palReadPad((stm32_gpio_t*)port, pin);
}

void SWDIO_MODE_FLOAT(void) {
    palSetPadMode((stm32_gpio_t*)SWDIO_PORT, SWDIO_PIN, PAL_MODE_INPUT);
}

void SWDIO_MODE_DRIVE(void) {
    palSetPadMode((stm32_gpio_t*)SWDIO_PORT, SWDIO_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
}
