/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief System/hardware module for RX SOC family
 */

#include <device.h>
#include <init.h>
#include <kernel.h>
#include <arch/cpu.h>
#include <soc.h>

#include <drivers/pinmux.h>
#include <drivers/gpio.h>

/* Renesas FIT module for iodefine.h data structures */
#include <platform.h>

/**
 * @brief Reboot the system
 *
 * @param type Types (SYS_REBOOT_COLD,
 * SYS_REBOOT_WARM) not supported by processor.
 *
 * When successful, this routine does not return.
 *
 * @return N/A
 */
void sys_arch_reboot(int type)
{
	ARG_UNUSED(type);

	SYSTEM.PRCR.WORD = 0xA502; /* Enable writing to the Software Reset */
	SYSTEM.SWRR = 0xA501;	   /* Software Reset */
	SYSTEM.PRCR.WORD = 0xA500; /* Disable writing to the Software Reset */
}

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */
static int rx66n_init(const struct device *arg)
{
	uint32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();
	irq_unlock(key);

	return 0;
}

/**
 * @brief initialize one pin of a RX family MCU
 *
 * @param pinmux		the pinmux device for the pin
 * @param gpioe			gpio device for the pin
 * @param pin			pin number
 * @param pinmux_function	function of the pinmux to select
 * @param gpio_flags		gpio flags
 *
 * @returns	pinmux or gpio device error code or 0 on success
 */
int rx_pin_init(const struct device *pinmux,
	const struct device *gpio, uint8_t pin, uint32_t pinmux_func,
	uint32_t gpio_flags)
{
	int ret;

	if (pinmux != NULL && device_is_ready(pinmux)) {
		ret = pinmux_pin_set(pinmux, pin, pinmux_func);
	} else {
		ret = -EINVAL;
	}

	if (ret == 0 && gpio != NULL && device_is_ready(gpio)) {
		ret = gpio_pin_configure(gpio, pin, gpio_flags);
	}
	return ret;
}

SYS_INIT(rx66n_init, PRE_KERNEL_1, 0);
