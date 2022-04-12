/*
 * Copyright (c) 2022 KT-Elektronik, Klaucke und Partner GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_mtu_counter

#include <zephyr.h>
#include <device.h>
#include <soc.h>

#include <drivers/counter.h>
#include <drivers/clock_control.h>
#include <drivers/clock_control/clock_control_rx65n.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(counter_rx_mtu, CONFIG_COUNTER_LOG_LEVEL);

struct tcr {
	/* time prescaler select */
	uint8_t tpsc : 3;
	/* input clock edge select */
	uint8_t ckeg : 2;
	/* counter clear source select */
	uint8_t cclr : 3;
};

struct pcnt_rx_config {
	/* timer control register */
	volatile struct tcr *tcr;
	/* timer general registers */
	volatile uint16_t *tgr;
	/* timer counter register */
	volatile uint16_t *tcnt;
	/* timer start register (common for all TPUs) */
	volatile uint8_t *tstr;
	/* determines which bit in TSTR corresponds to this TPU */
	uint8_t bit_idx;
	/* prescaler setting for TCR */
	uint8_t prescaler;
	/* clock divider - this is determined by the prescaler setting but depends on the TPU */
	uint16_t clock_divider;
	/* clock responsible for the TPU */
	const struct device *clock;
	/* clock subsystem */
	struct clock_control_rx65n_subsys clock_subsys;
};

static int pcnt_rx_start(const struct device *dev)
{
	LOG_ERR("%s is not supported", __func__);
	return -ENOTSUP;
}

static int pcnt_rx_stop(const struct device *dev)
{
	LOG_ERR("%s is not supported", __func__);
	return -ENOTSUP;
}

static int pcnt_rx_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct pcnt_rx_config *config = dev->config;

	*ticks = *config->tcnt;
	return 0;
}

static int pcnt_rx_set_top_value(const struct device *dev,
	const struct counter_top_cfg *top_cfg)
{
	const struct pcnt_rx_config *config = dev->config;

	config->tgr[0] = top_cfg->ticks;
	config->tgr[1] = top_cfg->ticks;

	if (!(top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET)) {
		*config->tcnt = 0;
	}

	return 0;
}

static uint32_t pcnt_rx_get_top_value(const struct device *dev)
{
	const struct pcnt_rx_config *config = dev->config;

	return config->tgr[0];
}

/**
 * @brief initialize a GPIO pulse counter device
 *
 * @param dev	the device structure
 *
 * @returns	0 on success, negative error code on failure
 */
static int pcnt_rx_init(const struct device *dev)
{
	const struct pcnt_rx_config *config = dev->config;
	int ret = 0;

	/* MSTP(MTU1) = 0U; // Release MTU channel 1 from stop state */
	ret = clock_control_on(config->clock, (clock_control_subsys_t *)&config->clock_subsys);
	/* MTU.TSTRA.BIT.CST1 = 0U; // Stop MTU channel 1 counter */
	WRITE_BIT(*config->tstr, config->bit_idx, 0);
	config->tcr->tpsc = config->prescaler;
	/* MTU.TSTRA.BIT.CST1 = 1U; // Start MTU channel 1 counter */
	WRITE_BIT(*config->tstr, config->bit_idx, 1);

	return ret;
}

static const struct counter_driver_api pcnt_rx_driver_api = {
	.start = pcnt_rx_start,
	.stop = pcnt_rx_stop,
	.get_value = pcnt_rx_get_value,
	.set_top_value = pcnt_rx_set_top_value,
	.get_top_value = pcnt_rx_get_top_value,
};

#define PARENT(id) DT_PARENT(DT_DRV_INST(id))

#define PCNT_RX_INST_INIT(id) \
	static const struct pcnt_rx_config pcnt_rx_config##id = { \
		.tcr = (struct tcr *)DT_INST_REG_ADDR_BY_NAME(id, TCR), \
		.tgr = (uint16_t *)DT_INST_REG_ADDR_BY_NAME(id, TGR), \
		.tcnt = (uint16_t *)DT_INST_REG_ADDR_BY_NAME(id, TCNT), \
		.tstr = (uint8_t *)DT_REG_ADDR_BY_NAME(PARENT(id), TSTR), \
		.bit_idx = DT_INST_PROP(id, bit_idx), \
		.prescaler = DT_INST_PROP(id, prescaler), \
		.clock_divider = DT_INST_PROP(id, clock_divider), \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(id, clock)), \
		.clock_subsys = DT_INST_CLOCK_RX65N_SUB_SYSTEM(id, clock_subsystems), \
	}; \
	static int pcnt_rx_init##id(const struct device *dev) \
	{ \
		int ret = 0; \
		DT_INST_FOREACH_PROP_ELEM(id, gpios, RX_INIT_PIN) \
		if (ret == 0) { \
			ret = pcnt_rx_init(dev); \
		} \
		return ret; \
	} \
	DEVICE_DT_INST_DEFINE(id, \
		&pcnt_rx_init##id, \
		NULL, \
		NULL, \
		&pcnt_rx_config##id, \
		POST_KERNEL, \
		CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
		&pcnt_rx_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PCNT_RX_INST_INIT)
