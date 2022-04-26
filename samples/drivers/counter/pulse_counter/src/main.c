/*
 * Copyright (c) 2022 KT-Elektronik, Klaucke und Partner GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * Example application to test counters with external pulse sources. The board configuration
 * overlay for the mimxrt1060_evk board configures three inputs for pulse counting.
 * The application tests for different counter devices:
 *
 * - TIMER1 with an alarm to trigger after 1000 pulses counted
 * - TIMER2 counts pulses
 * - TIMER3 should be configured to an internal clock with a known frequency that will trigger an
 *   alarm after 50 ms
 * - TIMER4 counts pulses
 *
 * The count values of TIMER1, TIMER2 and TIMER4 are read and written to the log-console every
 * second. TIMER4 is reset to 0 after reading out the value.
 */

#include <zephyr.h>
#include <sys/printk.h>

#include <drivers/counter.h>

#include <devicetree.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#if CONFIG_BOARD_MIMXRT1060_EVK
	#define TIMER1 DEVICE_DT_GET(DT_NODELABEL(tmr1_timer0))
	#define TIMER2 DEVICE_DT_GET(DT_NODELABEL(tmr1_timer1))
	#define TIMER3 DEVICE_DT_GET(DT_NODELABEL(tmr2_timer0))
	#define TIMER4 DEVICE_DT_GET(DT_NODELABEL(tmr3_timer0))
#endif

static void counter_alarm_callback(const struct device *dev, uint8_t chan_id, uint32_t ticks,
					void *user_data)
{
	LOG_INF("counter %s alarm at %u ticks", dev->name, ticks);
}

static void output_counter(const struct device *dev, bool reset)
{
	uint32_t counter_value;

	counter_get_value(dev, &counter_value);

	LOG_INF("counter %s value: %u", dev->name, counter_value);

	if (reset) {
		struct counter_top_cfg cfg;

		cfg.ticks = counter_get_top_value(dev);
		cfg.callback = NULL;
		cfg.user_data = NULL;
		/* crucially, the flag COUNTER_TOP_CFG_DONT_RESET is not set - if the driver
		 * supports this flag correctly, this should result in a reset of the counter
		 */
		cfg.flags = 0;

		counter_set_top_value(dev, &cfg);
	}
}

void main(void)
{
	static struct counter_alarm_cfg alarm_cfg;

	alarm_cfg.callback = counter_alarm_callback;
	alarm_cfg.ticks = 1000;
	alarm_cfg.user_data = NULL;
	alarm_cfg.flags = 0;

	counter_set_channel_alarm(TIMER1, 0, &alarm_cfg);

	alarm_cfg.ticks = counter_us_to_ticks(TIMER3, 50000);
	counter_set_channel_alarm(TIMER3, 0, &alarm_cfg);

	counter_start(TIMER1);
	counter_start(TIMER2);
	counter_start(TIMER3);
	counter_start(TIMER4);

	LOG_INF("Timers started.");

	while (true) {
		k_msleep(1000);
		output_counter(TIMER1, false);
		output_counter(TIMER2, false);
		output_counter(TIMER4, true);
	}
}
