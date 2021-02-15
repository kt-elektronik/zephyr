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


struct gpio_rx_data{
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


struct gpio_rx_cfg{

};


static int gpio_rx_init(const struct device *port){

	// struct gpio_rx_data *data = port->data;

	// data->reg.pdr  = (uint8_t*)DT_INST_REG_ADDR_BY_NAME(0, PDR);
	// data->reg.podr = (uint8_t*)DT_INST_REG_ADDR_BY_NAME(0, PODR);
	// data->reg.pidr = (uint8_t*)DT_INST_REG_ADDR_BY_NAME(0, PIDR);
	// data->reg.pmr  = (uint8_t*)DT_INST_REG_ADDR_BY_NAME(0, PMR);
	// data->reg.odr0 = (uint8_t*)DT_INST_REG_ADDR_BY_NAME(0, ODR0);
	// data->reg.odr1 = (uint8_t*)DT_INST_REG_ADDR_BY_NAME(0, ODR1);
	// data->reg.pcr  = (uint8_t*)DT_INST_REG_ADDR_BY_NAME(0, PCR);

	return 0;
}

static int gpio_rx_config(const struct device *port, 
				gpio_pin_t pin, gpio_flags_t flags)
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

static int gpio_rx_port_get_raw(const struct device *port, uint32_t *value){

	struct gpio_rx_data *data = port->data;

	*value = *(data->reg.pidr);
	return 0;
}

static int gpio_rx_port_set_masked_raw(const struct device *port, uint32_t mask, uint32_t value){

	struct gpio_rx_data *data = port->data;
	uint32_t key;

	key = irq_lock();
	*(data->reg.podr) = ((*data->reg.podr) & ~mask) | (mask & value);
	irq_unlock(key);
	return 0;
}

static int gpio_rx_port_set_bits_raw(const struct device *port, uint32_t pins){

	struct gpio_rx_data *data = port->data;

	*(data->reg.podr) |= pins;
	return 0;
}

static int gpio_rx_port_clear_bits_raw(const struct device *port, uint32_t pins){
	
	struct gpio_rx_data *data = port->data;

	*(data->reg.podr) &= ~pins;
	return 0;
}

static int gpio_rx_port_toggle_bits(const struct device *port, uint32_t pins){

	struct gpio_rx_data *data = port->data;
	uint32_t key;

	key = irq_lock();
	*(data->reg.podr) = *(data->reg.podr) ^ pins;
	irq_unlock(key);
	return 0;
}


static int gpio_rx_pin_interrupt_configure(const struct device *port, gpio_pin_t pin, enum gpio_int_mode mode, enum gpio_int_trig trig){
	
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

static int gpio_rx_manage_callback(const struct device *port, struct gpio_callback *callback, bool set){

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


// @todo: Use a macro for the following

static struct gpio_rx_data gpio_rx_0_data = {
	.reg = {
		.pdr  = (uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port0), PDR),
		.podr =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port0), PODR),
		.pidr =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port0), PIDR),
		.pmr  =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port0), PMR),
		.odr0 =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port0), ODR0),
		.odr1 =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port0), ODR1),
		.pcr  =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port0), PCR) 
	}
};
static const struct gpio_rx_cfg gpio_rx_0_config;
DEVICE_DT_DEFINE(DT_NODELABEL(port0),
			gpio_rx_init,
			device_pm_control_nop,
			&gpio_rx_0_data, &gpio_rx_0_config,
			POST_KERNEL,
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
			&gpio_rx_driver_api);


static struct gpio_rx_data gpio_rx_1_data = {
	.reg = {
		.pdr  = (uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port1), PDR),
		.podr =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port1), PODR),
		.pidr =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port1), PIDR),
		.pmr  =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port1), PMR),
		.odr0 =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port1), ODR0),
		.odr1 =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port1), ODR1),
		.pcr  =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port1), PCR) 
	}
};
static const struct gpio_rx_cfg gpio_rx_1_config;
DEVICE_DT_DEFINE(DT_NODELABEL(port1),
			gpio_rx_init,
			device_pm_control_nop,
			&gpio_rx_1_data, &gpio_rx_1_config,
			POST_KERNEL,
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
			&gpio_rx_driver_api);


static struct gpio_rx_data gpio_rx_7_data = {
	.reg = {
		.pdr  = (uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port7), PDR),
		.podr =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port7), PODR),
		.pidr =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port7), PIDR),
		.pmr  =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port7), PMR),
		.odr0 =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port7), ODR0),
		.odr1 =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port7), ODR1),
		.pcr  =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(port7), PCR) 
	}
};
static const struct gpio_rx_cfg gpio_rx_7_config;
DEVICE_DT_DEFINE(DT_NODELABEL(port7),
			gpio_rx_init,
			device_pm_control_nop,
			&gpio_rx_7_data, &gpio_rx_7_config,
			POST_KERNEL,
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
			&gpio_rx_driver_api);


static struct gpio_rx_data gpio_rx_g_data = {
	.reg = {
		.pdr  = (uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(portg), PDR),
		.podr =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(portg), PODR),
		.pidr =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(portg), PIDR),
		.pmr  =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(portg), PMR),
		.odr0 =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(portg), ODR0),
		.odr1 =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(portg), ODR1),
		.pcr  =	(uint8_t*)DT_REG_ADDR_BY_NAME(DT_NODELABEL(portg), PCR) 
	}
};
static const struct gpio_rx_cfg gpio_rx_g_config;
DEVICE_DT_DEFINE(DT_NODELABEL(portg),
			gpio_rx_init,
			device_pm_control_nop,
			&gpio_rx_g_data, &gpio_rx_g_config,
			POST_KERNEL,
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
			&gpio_rx_driver_api);