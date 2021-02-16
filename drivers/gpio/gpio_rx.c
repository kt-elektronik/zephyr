/*
 * Copyright (c) 2020 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_gpio

#include <soc.h>
#include <errno.h>
#include <device.h>
#include <drivers/gpio.h>
#include <kernel.h>
#include <sys/util.h>
#include <drivers/pinmux.h>

#include "gpio_utils.h"

struct gpio_rx_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;

	struct {
		volatile uint8_t *pdr;
		volatile uint8_t *podr;
		volatile uint8_t *pidr;
		volatile uint8_t *pmr;
		volatile uint8_t *odr0;
		volatile uint8_t *odr1;
		volatile uint8_t *pcr;
		volatile uint8_t *dscr;
		volatile uint8_t *dscr2;
	} reg;

	/* list of callbacks */
	sys_slist_t cb;
};

struct gpio_rx_driver_cfg {

	volatile uint8_t *gpio_base_addr;
};

static int gpio_rx_init(const struct device *port)
{
	return 0;
}

static int gpio_rx_config(const struct device *port, gpio_pin_t pin,
			  gpio_flags_t flags)
{
	struct gpio_rx_data *data = port->data;

	if (pin >= RX_GPIO_PORT_PINS) {
		return -EINVAL;
	}

	/* Not supported */
	if ((flags & GPIO_PULL_DOWN) != 0) {
		return -ENOTSUP;
	}

	/* Set pull-up if requested */
	WRITE_BIT(*(data->reg.pcr), pin, flags & GPIO_PULL_UP);

	/* Open drain */
	if ((flags & GPIO_OPEN_DRAIN) != 0) {
		if (pin < 4) {
			(*data->reg.odr0) |= BIT(pin << 1);
		} else {
			(*data->reg.odr0) |= BIT((pin - 4) << 1);
		}
	} else {
		if (pin < 4) {
			(*data->reg.odr0) &= ~BIT(pin << 1);
		} else {
			(*data->reg.odr0) &= ~BIT((pin - 4) << 1);
		}
	}

	/* Set the initial output value before enabling output to avoid glitches
	 */
	if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
		*(data->reg.podr) |= BIT(pin);
	}
	if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
		*(data->reg.podr) &= ~BIT(pin);
	}

	/* Set to output */
	WRITE_BIT(*(data->reg.pdr), pin, flags & GPIO_OUTPUT);

	return 0;
}

static int gpio_rx_port_get_raw(const struct device *port, uint32_t *value)
{
	struct gpio_rx_data *data = port->data;

	*value = *(data->reg.pidr);
	return 0;
}

static int gpio_rx_port_set_masked_raw(const struct device *port, uint32_t mask,
				       uint32_t value)
{
	struct gpio_rx_data *data = port->data;
	uint32_t key;

	key = irq_lock();
	*(data->reg.podr) = ((*data->reg.podr) & ~mask) | (mask & value);
	irq_unlock(key);
	return 0;
}

static int gpio_rx_port_set_bits_raw(const struct device *port, uint32_t pins)
{
	struct gpio_rx_data *data = port->data;

	*(data->reg.podr) |= pins;
	return 0;
}

static int gpio_rx_port_clear_bits_raw(const struct device *port, uint32_t pins)
{
	struct gpio_rx_data *data = port->data;

	*(data->reg.podr) &= ~pins;
	return 0;
}

static int gpio_rx_port_toggle_bits(const struct device *port, uint32_t pins)
{
	struct gpio_rx_data *data = port->data;
	uint32_t key;

	key = irq_lock();
	*(data->reg.podr) = *(data->reg.podr) ^ pins;
	irq_unlock(key);
	return 0;
}

static int gpio_rx_pin_interrupt_configure(const struct device *port,
					   gpio_pin_t pin,
					   enum gpio_int_mode mode,
					   enum gpio_int_trig trig)
{
	// struct gpio_rx_data *data = port->data;
	// uint32_t io_pin = pin + data->port.pin_offset; /* Range from 0 - 39 */
	// uint32_t *reg = GET_GPIO_PIN_REG(io_pin);
	// int intr_trig_mode = convert_int_type(mode, trig);
	// uint32_t reg_val;
	// uint32_t key;

	// if (intr_trig_mode < 0) {
	// 	return intr_trig_mode;
	// }

	// key = irq_lock();

	// reg_val = *reg;
	// //reg_val &= ~(GPIO_PIN_INT_ENA_M | GPIO_PIN_INT_TYPE_M);
	// /* Enable Interrupt on CPU0 (PRO_CPU) */
	// //reg_val |= GPIO_CPU0_INT_ENABLE;
	// /* Interrupt triggering mode */
	// //reg_val |= intr_trig_mode << GPIO_PIN_INT_TYPE_S;
	// *reg = reg_val;

	// irq_unlock(key);

	return 0;
}

static int gpio_rx_manage_callback(const struct device *port,
				   struct gpio_callback *callback, bool set)
{
	struct gpio_rx_data *data = port->data;

	return gpio_manage_callback(&data->cb, callback, set);
}

static const struct gpio_driver_api gpio_rx_driver_api ={
	.pin_configure =           gpio_rx_config,
	.port_get_raw =            gpio_rx_port_get_raw,
	.port_set_masked_raw =     gpio_rx_port_set_masked_raw,
	.port_set_bits_raw =       gpio_rx_port_set_bits_raw,
	.port_clear_bits_raw =     gpio_rx_port_clear_bits_raw,
	.port_toggle_bits =        gpio_rx_port_toggle_bits,
	.pin_interrupt_configure = gpio_rx_pin_interrupt_configure,
	.manage_callback =         gpio_rx_manage_callback,
};


#define GPIO_DEVICE_INIT(_port) \
	static struct gpio_rx_data _port##_data = { \
		.reg = { \
			.pdr  = (uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(_port), PDR),  \
			.podr =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(_port), PODR), \
			.pidr =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(_port), PIDR), \
			.pmr  =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(_port), PMR),  \
			.odr0 =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(_port), ODR0), \
			.odr1 =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(_port), ODR1), \
			.pcr  =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(_port), PCR)   \
	} \
	}; \
	static struct gpio_rx_driver_cfg _port##_cfg = { \
		.gpio_base_addr = (uint8_t*)DT_REG_ADDR(DT_NODELABEL(_port)) \
	}; \
	DEVICE_DT_DEFINE(DT_NODELABEL(_port),			\
			    gpio_rx_init,						\
			    device_pm_control_nop,				\
			    &_port##_data, &_port##_cfg,		\
			    POST_KERNEL,			 			\
			    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			    &gpio_rx_driver_api)


#if DT_NODE_EXISTS(DT_NODELABEL(port0))
GPIO_DEVICE_INIT(port0);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(port1))
GPIO_DEVICE_INIT(port1);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(port2))
GPIO_DEVICE_INIT(port2);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(port3))
GPIO_DEVICE_INIT(port3);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(port4))
GPIO_DEVICE_INIT(port4);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(port5))
GPIO_DEVICE_INIT(port5);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(port6))
GPIO_DEVICE_INIT(port6);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(port7))
GPIO_DEVICE_INIT(port7);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(port8))
GPIO_DEVICE_INIT(port8);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(port9))
GPIO_DEVICE_INIT(port9);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(porta))
GPIO_DEVICE_INIT(porta);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(portb))
GPIO_DEVICE_INIT(portb);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(portc))
GPIO_DEVICE_INIT(portc);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(portd))
GPIO_DEVICE_INIT(portd);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(porte))
GPIO_DEVICE_INIT(porte);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(portf))
GPIO_DEVICE_INIT(portf);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(portg))
GPIO_DEVICE_INIT(portg);
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(portj))
GPIO_DEVICE_INIT(portj);
#endif

