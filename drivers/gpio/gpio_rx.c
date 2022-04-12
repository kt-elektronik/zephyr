/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_gpio

#include <soc.h>
#include <errno.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/pinmux.h>
#include <drivers/interrupt_controller/rxv2_irq.h>
#include <kernel.h>
#include <sys/util.h>
#include "gpio_utils.h"

#define DEV_CFG(dev) ((const struct gpio_rx65n_driver_cfg *)((dev)->config))
#define DEV_DATA(dev) ((struct gpio_rx65n_data *)((dev)->data))

/* default priority for pin triggered interrupts */
#define RX_GPIO_IRQ_PRIORITY 14
/* Renesas RX processors have 16 pin irqs (0-15)*/
#define RX_GPIO_MAX_IRQ 15
/* each GPIO port has 8 pins */
#define RX_GPIO_PORT_PINS 8

/*
 * data structure supplied to gpio_rx65n_irq_callback (the isr) to identify
 * which gpio-port and pin have been triggered
 */
struct gpio_rx65n_cb {
	const struct device *port;
	uint8_t bitmask;
};

struct gpio_rx65n_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* list of callbacks */
	sys_slist_t cb;
	/* list of callback information*/
	struct gpio_rx65n_cb cb_params[RX_GPIO_PORT_PINS];
};

struct gpio_rx65n_driver_cfg {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* name of the associated pinmux driver*/
	const char *pinmux_name;
	/* name of the associated pin irq controller */
	const char *int_ctrl_name;
	/* irqs associated with the port pins */
	uint8_t irq[RX_GPIO_PORT_PINS];
	/* base address of the gpio port registers */
	volatile uint8_t *gpio_base_addr;
	/* registers associated with the gpio port */
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
};

/**
 * @brief isr for all gpio interrupts
 *
 * @param parameter    pointer to a struct gpio_rx65n_cb determining which port
 *                     and pin triggered the interrupt
 */
static void gpio_rx65n_irq_callback(const void *parameter)
{
	const struct gpio_rx65n_cb *cb =
		(const struct gpio_rx65n_cb *)parameter;
	struct gpio_rx65n_data *data = DEV_DATA(cb->port);

	gpio_fire_callbacks(&data->cb, cb->port, cb->bitmask);
}

/**
 * @brief initializer for a gpio port
 *
 * @param    Pointer to device structure for the driver instance.
 *
 * @return   always 0 (no error state)
 */
static int gpio_rx65n_init(const struct device *dev)
{
	struct gpio_rx65n_data *data = DEV_DATA(dev);

	for (uint8_t i = 0; i < RX_GPIO_PORT_PINS; i++) {
		data->cb_params[i].port = dev;
		data->cb_params[i].bitmask = BIT(i);
	}
	return 0;
}

static int gpio_rx65n_pin_configure(const struct device *dev, gpio_pin_t pin,
				    gpio_flags_t flags)
{
	const struct gpio_rx65n_driver_cfg *cfg = DEV_CFG(dev);

	if (pin >= RX_GPIO_PORT_PINS) {
		return -EINVAL;
	}

	/* Not supported */
	if ((flags & GPIO_PULL_DOWN) != 0) {
		return -ENOTSUP;
	}

	/* Set pull-up if requested */
	WRITE_BIT(*(cfg->reg.pcr), pin, flags & GPIO_PULL_UP);

	/* Set to peripheral function if requested */
	WRITE_BIT(*(cfg->reg.pmr), pin, flags & GPIO_PERIPHERAL);

	/* Open drain (pins 0-3: odr0, pins 4-8: odr1) */
	if (pin < 4) {
		WRITE_BIT(*(cfg->reg.odr0), pin << 1, flags & GPIO_OPEN_DRAIN);
	} else {
		WRITE_BIT(*(cfg->reg.odr1), ((pin - 4) << 1),
			  flags & GPIO_OPEN_DRAIN);
	}

	/* Set the initial output value before enabling output to avoid glitches
	 */
	WRITE_BIT(*(cfg->reg.podr), pin, flags & GPIO_OUTPUT_INIT_HIGH);

	/* Set to output */
	WRITE_BIT(*(cfg->reg.pdr), pin, flags & GPIO_OUTPUT);

	return 0;
}

static int gpio_rx65n_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_rx65n_driver_cfg *cfg = DEV_CFG(dev);

	*value = *(cfg->reg.pidr);
	return 0;
}

static int gpio_rx65n_port_set_masked_raw(const struct device *dev,
					  uint32_t mask, uint32_t value)
{
	const struct gpio_rx65n_driver_cfg *cfg = DEV_CFG(dev);

	uint32_t key = irq_lock();
	*(cfg->reg.podr) = ((*cfg->reg.podr) & ~mask) | (mask & value);
	irq_unlock(key);
	return 0;
}

static int gpio_rx65n_port_set_bits_raw(const struct device *dev, uint32_t pins)
{
	const struct gpio_rx65n_driver_cfg *cfg = DEV_CFG(dev);

	*(cfg->reg.podr) |= pins;
	return 0;
}

static int gpio_rx65n_port_clear_bits_raw(const struct device *dev,
					  uint32_t pins)
{
	const struct gpio_rx65n_driver_cfg *cfg = DEV_CFG(dev);

	*(cfg->reg.podr) &= ~pins;
	return 0;
}

static int gpio_rx65n_port_toggle_bits(const struct device *dev, uint32_t pins)
{
	const struct gpio_rx65n_driver_cfg *cfg = DEV_CFG(dev);

	uint32_t key = irq_lock();
	*(cfg->reg.podr) = *(cfg->reg.podr) ^ pins;
	irq_unlock(key);
	return 0;
}

static int gpio_rx65n_pin_interrupt_configure(const struct device *dev,
					      gpio_pin_t pin,
					      enum gpio_int_mode mode,
					      enum gpio_int_trig trig)
{
	struct gpio_rx65n_data *data = DEV_DATA(dev);
	const struct gpio_rx65n_driver_cfg *cfg = DEV_CFG(dev);

	uint8_t irq = cfg->irq[pin];

	if (irq >= RX_GPIO_MAX_IRQ) {
		/* the selected pin is not configured for an interrupt */
		return -EINVAL;
	}

	const struct device *pinmux = device_get_binding(cfg->pinmux_name);
	const struct device *int_ctrl = device_get_binding(cfg->int_ctrl_name);

	pinmux_pin_set(pinmux, pin, (mode & GPIO_INT_ENABLE) ? BIT(6) : 0);
	return rxv2_pin_intc_config(int_ctrl, irq, RX_GPIO_IRQ_PRIORITY, mode,
				    trig, gpio_rx65n_irq_callback,
				    (void *)&data->cb_params[pin], 0);
}

static int gpio_rx65n_manage_callback(const struct device *dev,
				      struct gpio_callback *callback, bool set)
{
	struct gpio_rx65n_data *data = DEV_DATA(dev);

	return gpio_manage_callback(&data->cb, callback, set);
}

static const struct gpio_driver_api gpio_rx_driver_api = {
	.pin_configure = gpio_rx65n_pin_configure,
	.port_get_raw = gpio_rx65n_port_get_raw,
	.port_set_masked_raw = gpio_rx65n_port_set_masked_raw,
	.port_set_bits_raw = gpio_rx65n_port_set_bits_raw,
	.port_clear_bits_raw = gpio_rx65n_port_clear_bits_raw,
	.port_toggle_bits = gpio_rx65n_port_toggle_bits,
	.pin_interrupt_configure = gpio_rx65n_pin_interrupt_configure,
	.manage_callback = gpio_rx65n_manage_callback,
};

#define INST_PHANDLE_LABEL_OR(id, ph, alt)                                     \
	DT_PROP_OR(DT_PHANDLE(DT_DRV_INST(id), ph), label, alt)

#define GPIO_DEVICE_INIT(id)                                                   \
	static struct gpio_rx65n_data port##id##_data = {                      \
	};                                                                     \
	static struct gpio_rx65n_driver_cfg port##id##_cfg = {                 \
		.common = { .port_pin_mask =                                   \
				    GPIO_PORT_PIN_MASK_FROM_DT_INST(id) },     \
		.gpio_base_addr = (uint8_t *)DT_INST_REG_ADDR(id),             \
		.pinmux_name = INST_PHANDLE_LABEL_OR(id, pinmux, ""),          \
		.int_ctrl_name = INST_PHANDLE_LABEL_OR(id, irq_ctrl, ""),      \
		.irq = DT_INST_PROP_OR(id, irqs, {}),                          \
		.reg = { .pdr = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, PDR),  \
			.podr = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, PODR), \
			.pidr = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id,  PIDR),\
			.pmr = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, PMR),   \
			.odr0 = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id,  ODR0),\
			.odr1 = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, ODR1), \
			.pcr = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, PCR) }  \
	};                                                                     \
	DEVICE_DT_INST_DEFINE(id, gpio_rx65n_init, NULL,      \
			      &port##id##_data, &port##id##_cfg, PRE_KERNEL_1, \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,              \
			      &gpio_rx_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_DEVICE_INIT)
