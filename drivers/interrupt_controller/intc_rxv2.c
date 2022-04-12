/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_pin_intc

#include <zephyr.h>
#include <device.h>
#include <dt-bindings/interrupt-controller/rxv2_irq_dt.h>
#include <drivers/interrupt_controller/rxv2_irq.h>

/* the pin IRQs 0-15 correspond to interrupts 64-79. This offset allows
 * to convert IRQ-number to interrupt number */
#define RX_PIN_IRQ_OFFSET 64

struct rx_pin_intc_data {
};

/* configuration of the pin interrupt controller */
struct rx_pin_intc_cfg {
	/* interrupt controll register */
	uint8_t *irqcr;
	/* digital filter enable register */
	uint8_t *irqflte;
	/* digital filter controll register */
	uint16_t *irqfltc;
};

struct rx_gpio_intc_api {
};

#define DEV_CFG(dev) ((const struct rx_pin_intc_cfg *)((dev)->config))

int rxv2_pin_intc_config(const struct device *dev, uint8_t irq,
			 uint8_t priority, enum gpio_int_mode mode,
			 enum gpio_int_trig trig,
			 void (*routine)(const void *parameter),
			 const void *parameter, uint32_t flags)
{
	const struct rx_pin_intc_cfg *cfg = DEV_CFG(dev);

	uint8_t interrupt = irq + RX_PIN_IRQ_OFFSET;

	if ((mode & GPIO_INT_MODE_LEVEL) == 0 &&
	    (trig & GPIO_INT_LEVEL_HIGH) != 0) {
		/* hight level trigger is not supported */
		return -ENOTSUP;
	}

	uint32_t key = irq_lock();

	/* reset IRQCR register */
	cfg->irqcr[irq] = 0;

	if (mode & GPIO_INT_ENABLE) {
		irq_connect_dynamic(interrupt, priority, routine, parameter,
				    flags);

		if (mode & GPIO_INT_EDGE) {
			if (trig & GPIO_INT_LOW_0) {
				cfg->irqcr[irq] |= BIT(2);
			}
			if (trig & GPIO_INT_HIGH_1) {
				cfg->irqcr[irq] |= BIT(3);
			}
		}
		/* TODO: configure digital filter based on flags */

		irq_enable(interrupt);
	} else {
		irq_disable(interrupt);
	}

	irq_unlock(key);

	return 0;
}

/**
 * @brief Initialize the device driver
 *
 * @return always 0
 */
int rxv2_irq_init(const struct device *unused)
{
	ARG_UNUSED(unused);
	return 0;
}

struct rx_pin_intc_data data;
const struct rx_pin_intc_cfg cfg = {
	.irqcr = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(0, IRQCR),
	.irqflte = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(0, IRQFLTE),
	.irqfltc = (uint16_t *)DT_INST_REG_ADDR_BY_NAME(0, IRQFLTC)
};
struct rx_gpio_intc_api api;

DEVICE_DT_INST_DEFINE(0, rxv2_irq_init, NULL, &data, &cfg,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &api);
