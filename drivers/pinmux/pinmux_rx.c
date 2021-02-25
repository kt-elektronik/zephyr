/*
 * Copyright (c) 2020 KT-Elektronik
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief PINMUX driver for the Renesas RX65 Processor
 */

#include <errno.h>
#include <device.h>
#include <drivers/pinmux.h>
#include <soc.h>



/* TODO: this should be included by
 * #include <../../modules/hal/renesas/FITModules/r_bsp/board/generic_rx65n/r_bsp.h>
 * The include path have to be added by CMake/Kconfig!
 */
#include <../../modules/hal/renesas/FITModules/r_bsp/mcu/rx65n/register_access/gnuc/iodefine.h>


struct pinmux_rx65_config {
	uintptr_t base;
};


/* set pin value via pinmux_pin_set(*dev, pin, *func) -- only for setting level or more? */
static int pinmux_rx65_set(const struct device *dev, uint32_t pin, uint32_t func)
{
	volatile struct st_port0 *port;
	port = (volatile struct st_port0 *) dev;

	if (func == 0)
		port->PODR.BYTE &= (~(1 << (pin & 0x00FFu)));
	else
		port->PODR.BYTE |= (1 << (pin & 0x00FFu));

	return 0;
}



/* read pin value via pinmux_pin_get(*dev, pin, *func) */
static int pinmux_rx65_get(const struct device *dev, uint32_t pin, uint32_t *func)
{


	return 0;
}

static int pinmux_rx65_pullup(const struct device *dev, uint32_t pin, uint8_t func)
{
	return -ENOTSUP;
}

static int pinmux_rx65_input(const struct device *dev, uint32_t pin, uint8_t func)
{
	return -ENOTSUP;
}

static int pinmux_rx65_init(const struct device *dev)
{

	return 0;
}

static struct pinmux_driver_api pinmux_rx65_driver_api = {
	.set = pinmux_rx65_set,
	.get = pinmux_rx65_get,
	.pullup = pinmux_rx65_pullup,
	.input = pinmux_rx65_input
};

static const struct pinmux_rx65_config pinmux_rx65_0_config = {
	.base = 0, //RX65_PINMUX_0_BASE_ADDR,
};

DEVICE_DEFINE(pinmux_rx65_0, CONFIG_PINMUX_RX_NAME,
		    &pinmux_rx65_init, device_pm_control_nop,
		    NULL, &pinmux_rx65_0_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &pinmux_rx65_driver_api);			


// DEVICE_DT_DEFINE(DT_NODELABEL(pinmux),
// 		    &pinmux_rx65_init,
// 		    device_pm_control_nop,
// 		    NULL, &pinmux_rx65_0_config,
// 		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
// 		    &pinmux_rx65_driver_api);	

	

// DEVICE_DT_INST_DEFINE(0, &pinmux_rx65_init,
// 		    device_pm_control_nop, NULL, NULL,
// 		    PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
// 		    &pinmux_rx65_driver_api);


