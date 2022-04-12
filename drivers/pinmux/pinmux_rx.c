/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
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

/* Renesas FIT module for iodefine.h data structures */
#include <platform.h>

#define DT_DRV_COMPAT renesas_rx_mpc

/* Enable writing to the PmnPFS registers */
#define MPC_ENABLE_WRITING  \
	((volatile struct st_mpc *)DT_REG_ADDR(DT_NODELABEL(mpc))) \
		->PWPR.BIT.B0WI = 0; \
	((volatile struct st_mpc *)DT_REG_ADDR(DT_NODELABEL(mpc))) \
		->PWPR.BIT.PFSWE = 1;

/* Disable writing to the PmnPFS registers */
#define MPC_DISABLE_WRITING \
	((volatile struct st_mpc *)DT_REG_ADDR(DT_NODELABEL(mpc))) \
		->PWPR.BIT.PFSWE = 0; \
	((volatile struct st_mpc *)DT_REG_ADDR(DT_NODELABEL(mpc))) \
		->PWPR.BIT.B0WI = 1;

#define DEV_CFG(dev) ((const struct pinmux_rx65_config *const)(dev)->config)
#define DEV_BASE(dev) ((volatile struct st_mpc *)(DEV_CFG(dev))->base)

struct pinmux_rx65_config {
	uintptr_t base;
};

/* Set pin function */
static int pinmux_rx65_set(const struct device *dev, uint32_t pin,
			   uint32_t func)
{
	uint8_t *mpc = (uint8_t *)DEV_BASE(dev);
	MPC_ENABLE_WRITING;

	/* set alternate function to corresponding pin register */
	mpc += pin;
	*mpc = func;

	MPC_DISABLE_WRITING;
	return 0;
}

/* Get pin function */
static int pinmux_rx65_get(const struct device *dev, uint32_t pin,
			   uint32_t *func)
{
	uint8_t *mpc = (uint8_t *)DEV_BASE(dev);
	mpc += pin;
	*func = *mpc;

	return 0;
}

static int pinmux_rx65_pullup(const struct device *dev, uint32_t pin,
			      uint8_t func)
{
	return -ENOTSUP;
}

static int pinmux_rx65_input(const struct device *dev, uint32_t pin,
			     uint8_t func)
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

#define RX_PINMUX_DEFINE(_id) \
static const struct pinmux_rx65_config pinmux_rx65_##_id##_config = { \
	.base = DT_INST_REG_ADDR(_id) \
}; \
DEVICE_DT_INST_DEFINE(_id, &pinmux_rx65_init, \
		NULL, NULL, &pinmux_rx65_##_id##_config, \
		PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY, \
		&pinmux_rx65_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RX_PINMUX_DEFINE)
