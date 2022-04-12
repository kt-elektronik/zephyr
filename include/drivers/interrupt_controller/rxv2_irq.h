/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Driver for RXV2 Interrupt Controllers
 *
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_RXV2_IRQ_H_
#define ZEPHYR_INCLUDE_DRIVERS_RXV2_IRQ_H_

#ifndef _ASMLANGUAGE

#include <zephyr/types.h>
#include <device.h>
#include <drivers/gpio.h>

/**
 * @brief data structure specifying a callback for a group interrupt
 */
struct grp_intc_rx65n_callback {
	/* used for dynamic list management, do not modify */
	sys_snode_t node;
	/* callback function */
	void (*callback)(const void *param);
	/* parameter for callback function */
	const void *param;
	/* pin mask specifying which functions of the group interrupt to use */
	uint32_t pin_mask;
};

/**
 *  @brief configuration of a pin interrupt (IRQ0-IRQ15)
 *
 * @param dev          pointer to the interrupt controller device
 * @param irq          IRQ number (0-15, NOT the vector number)
 * @param priority     priority of the interrupt
 * @param mode         trigger mode (e.g. GPIO_INT_EDGE)
 * @param trig         trigger flags (e.g. GPIO_INT_LEVEL_HIGH,
 *                     GPIO_INT_LEVEL_LOW)
 * @param routine      function to call then the irq is triggered
 * @param parameter    parameter to use when calling routine
 * @param flags        interrupt specific flags
 *
 * @return             0 on success or error codes from errno.h
 *
 * Of the 256 interrupts in the RX65N vector table, 16 are "pin interrupts" that
 * can be set to be triggered by signals on different pins (which pins are
 * connected to which interrupts is configured by the pinmux). This function
 * configures those pin interrupts based on the options provided to the gpio
 * driver
 */
int rxv2_pin_intc_config(const struct device *dev, uint8_t irq,
			 uint8_t priority, enum gpio_int_mode mode,
			 enum gpio_int_trig trig,
			 void (*routine)(const void *parameter),
			 const void *parameter, uint32_t flags);

/**
 * @brief managing a callback function of a group interrupt
 *
 * @param dev		pointer to the group interrupt controller device
 * @param callback	callback structure specifying the callback
 * @param set		add (true) or remove (false) the callback
 *
 * @returm		0 on success or error codes from errno.h
 */
int grp_intc_rx65n_manage_callback(const struct device *dev,
			struct grp_intc_rx65n_callback *callback, bool set);
#endif /* !_ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_DRIVERS_RXV2_IRQ_H_ */
