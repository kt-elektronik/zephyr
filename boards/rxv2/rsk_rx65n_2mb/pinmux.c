/*
 * Copyright (c) 2018 Philémon Jaermann
 *
 * Copyright (c) 2020 KT-Elektronik, Klaucke und Partner GmbH
 * SPDX-License-Identifier: Apache-2.0
 */


#include <init.h>
#include <drivers/pinmux.h>
#include <drivers/gpio.h>


#include <kernel.h>
#include <device.h>
#include <sys/sys_io.h>
//#include <soc.h>
//#include <pinmux/stm32/pinmux_.h>



static int pinmux_rsk_rx65n_2mb_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	// const struct device *p = device_get_binding(CONFIG_PINMUX_RX_NAME);
	// pinmux_pin_set(p, 2, 0); 

	const struct device *port0, *port1;

	port0 = device_get_binding(DT_LABEL(DT_NODELABEL(port0)));
	if (!port0) {
		return -ENODEV;
	}

	port1 = device_get_binding(DT_LABEL(DT_NODELABEL(port1)));
	if (!port1) {
		return -ENODEV;
	}

	// pinmux_pin_set or gpio_pin_set ?

	pinmux_pin_set(port0, 4, 1);

	gpio_pin_configure(port0, 6, GPIO_OUTPUT);
	gpio_pin_set(port0, 6, 0);



	return 0;
}

SYS_INIT(pinmux_rsk_rx65n_2mb_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);


