/*
 * Copyright (C) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Independent Watchdog (IWDT) Driver for Renesas RX65N Series
 */

#define DT_DRV_COMPAT renesas_rx65n_iwdt

#include <kernel.h>
#include <soc.h>
#include <drivers/watchdog.h>
#include <stdlib.h>
#include <sys/reboot.h>

/* Renesas FIT module for iodefine.h data structures */
#include <platform.h>

#define LOG_LEVEL CONFIG_WDT_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(wdt_iwdt_rx65n);

#define IWDT_CLOCK_FREQ_KHZ 120

struct wdt_rx65n_config {
	uintptr_t base;
};

struct wdt_rx65n_data {
	/* timeout callback used to handle watchdog event */
	wdt_callback_t callback;
	/* indicate whether a watchdog timeout is installed */
	bool timeout_installed;
	/* watchdog feed timeout in milliseconds */
	uint32_t timeout;
};

#define DEV_CFG(dev) ((const struct wdt_rx65n_config *const)(dev)->config)
#define DEV_BASE(dev) (DEV_CFG(dev)->base)
#define DEV_DATA(dev) ((struct wdt_rx65n_data *)(dev)->data)

static int wdt_rx65n_feed(const struct device *dev, int channel_id);

static void wdt_rx65n_isr(const struct device *dev)
{
	struct wdt_rx65n_data *data = DEV_DATA(dev);

	/* callback function */
	if (data->callback) {
		data->callback(dev, 0);
	}

	sys_reboot(0);
}

static int wdt_rx65n_disable(const struct device *dev)
{
	/* watchdog cannot be stopped once started */
	return -EPERM;
}

static int wdt_rx65n_setup(const struct device *dev, uint8_t options)
{
	volatile struct st_iwdt *iwdt = (volatile struct st_iwdt *)DEV_BASE(dev);
	struct wdt_rx65n_data *data = DEV_DATA(dev);

#if !CONFIG_IWDT_RX65N_AUTO_START_MODE || !CONFIG_IWDT_RX65N_OFS0_IWDTRSTIRQS

	/* In the case "Auto Start Mode without interrupt" there is no need to
	 * call	wdt_install_timeout(), so this check can be omitted.
	 */
	if (!data->timeout_installed) {
		LOG_ERR("No valid WDT timeout installed");
		return -EINVAL;
	}

#endif

	if (options & WDT_OPT_PAUSE_IN_SLEEP) {
		iwdt->IWDTCSTPR.BIT.SLCSTP = 1;
	} else {
		iwdt->IWDTCSTPR.BIT.SLCSTP = 0;
	}

	if (data->callback) {
		irq_enable(DT_INST_IRQN(0));

#if !CONFIG_IWDT_RX65N_AUTO_START_MODE
		/* Non-maskable interrupt request or interrupt request output
		 * is enabled.
		 */
		iwdt->IWDTRCR.BIT.RSTIRQS = 0;
#endif
	}

#if !CONFIG_IWDT_RX65N_AUTO_START_MODE
	/* Start counter by refresh when not in auto start mode */
	wdt_rx65n_feed(dev, 0);
#endif
	return 0;
}

static int wdt_rx65n_install_timeout(const struct device *dev,
	const struct wdt_timeout_cfg *cfg)
{
	struct wdt_rx65n_data *data = DEV_DATA(dev);

/* Following can be skipped if watchdog is already running in Auto Start Mode*/
#if !CONFIG_IWDT_RX65N_AUTO_START_MODE
	volatile struct st_iwdt *iwdt =	(struct st_iwdt *)DEV_BASE(dev);
	volatile struct st_iwdt iwdt_temp;

#define SIZE_TIMEOUT_PERIOD 4
	const uint16_t timeout_period[SIZE_TIMEOUT_PERIOD] = {1024, 4096, 8192, 16384};
#define SIZE_DIVIDE_RATIO 6
	const uint16_t divide_ratio[SIZE_DIVIDE_RATIO][2] = {{0, 1}, {2, 16},
		{3, 32}, {4, 64}, {15, 128}, {5, 256}};
#define SIZE_WINDOW_START 4
	const uint16_t window_start[SIZE_WINDOW_START] = {25, 50, 75, 0};

	uint16_t tops = 0;
	uint16_t cks = 0;
	uint16_t rpss = 0;
	int32_t error;
	int16_t last_error = INT16_MAX;
	uint16_t w_max = UINT16_MAX;

	if (cfg->window.min > cfg->window.max) {
		return -EINVAL;
	}

	/* get values for window max*/
	for (int i_p = 0; i_p < SIZE_TIMEOUT_PERIOD; i_p++) {
		for (int i_d = 0; i_d < SIZE_DIVIDE_RATIO; i_d++) {
			w_max = ((uint32_t)timeout_period[i_p] *
				(uint32_t)divide_ratio[i_d][1]) / IWDT_CLOCK_FREQ_KHZ;
			error = cfg->window.max - w_max;

			if (abs(error) < abs(last_error)) {
				last_error = error;
				tops = i_p;
				cks = divide_ratio[i_d][0];
				/* save watchdog timeout */
				data->timeout = w_max;
			}

			if (error < 0) {
				break;
			}
		}
	}

	/* get values for window min */
	last_error = INT16_MAX;
	for (int i_ws = 0; i_ws < SIZE_WINDOW_START; i_ws++) {
		uint16_t w_min = (uint32_t)w_max * (uint32_t)window_start[i_ws] / 100;

		error = cfg->window.min - w_min;

		if (abs(error) < abs(last_error)) {
			last_error = error;
			rpss = i_ws;
		}
	}
	iwdt_temp.IWDTCR.BIT.CKS = cks;
	iwdt_temp.IWDTCR.BIT.TOPS = tops;
	iwdt_temp.IWDTCR.BIT.RPES = 3;
	iwdt_temp.IWDTCR.BIT.RPSS = rpss;
	/* Write in one step as it becomes protected after first write */
	iwdt->IWDTCR.WORD = iwdt_temp.IWDTCR.WORD;

#endif /* !CONFIG_IWDT_RX65N_AUTO_START_MODE */

	/* mark installed */
	data->timeout_installed = true;

	/* install user timeout isr */
	data->callback = cfg->callback;

	return 0;
}

static int wdt_rx65n_feed(const struct device *dev, int channel_id)
{
	volatile struct st_iwdt *iwdt = (struct st_iwdt *)DEV_BASE(dev);

	iwdt->IWDTRR = 0;
	iwdt->IWDTRR = 0xFF;

	return 0;
}

static const struct wdt_driver_api wdt_rx65n_api = {
	.setup = wdt_rx65n_setup,
	.disable = wdt_rx65n_disable,
	.install_timeout = wdt_rx65n_install_timeout,
	.feed = wdt_rx65n_feed,
};

static int wdt_rx65n_init(const struct device *dev)
{
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), wdt_rx65n_isr,
		DEVICE_DT_INST_GET(0), 0);
	return 0;
}

static struct wdt_rx65n_data wdt_rx65n_dev_data;

static const struct wdt_rx65n_config wdt_rx65n_cfg = {
	.base = DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0, wdt_rx65n_init, NULL, &wdt_rx65n_dev_data, &wdt_rx65n_cfg,
	PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_rx65n_api);
