/*
 * Copyright (c) 2022 KT-Elektronik, Klaucke und Partner GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief device driver for the Renesas RX RTC as counter
 */

#include <zephyr.h>
#include <platform.h>
#include <soc.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(rx_rtc, CONFIG_COUNTER_LOG_LEVEL);

#include <drivers/counter.h>
#include <drivers/rtc/rx.h>

#if CONFIG_MINIMAL_LIBC
/* counter_rx_rtc requires mktime(), which is not provided by the Zephyr minimal libc
 * However, timeutil_timegm() offers the same functionality in Zephyr.
 */
#include <sys/timeutil.h>

static inline time_t mktime(const struct tm *tm)
{
	return timeutil_timegm(tm);
}
#endif

#define DT_DRV_COMPAT renesas_rx_rtc

struct rx_rtc_cfg {
	/* 64-Hz Counter */
	volatile uint8_t *r64cnt;
	/* Second Counter */
	volatile uint8_t *rseccnt;
	/* Minute Counter */
	volatile uint8_t *rmincnt;
	/* Hour Counter */
	volatile uint8_t *rhrcnt;
	/* Date Counter */
	volatile uint8_t *rdaycnt;
	/* Month Counter */
	volatile uint8_t *rmoncnt;
	/* Year Counter */
	volatile uint16_t *ryrcnt;
	/* RTC Control Register 1 */
	volatile uint8_t *rcr1;
	/* RTC Control Register 2 */
	volatile uint8_t *rcr2;
	/* RTC Control Register 3 */
	volatile uint8_t *rcr3;
	/* RTC Control Register 4 */
	volatile uint8_t *rcr4;
	/* Time Error Adjustment Register */
	volatile uint8_t *radj;
};

#define DEV_CFG(cfg, dev) const struct rx_rtc_cfg *cfg = \
			(const struct rx_rtc_cfg *) dev->config;

/**
 * @brief convert the time representation in the RTC to numerical values
 *
 * @param to_convert	representation in RTC register
 *
 * @return numerical value
 */
static inline uint16_t rtc_bcd_to_bin(uint8_t to_convert)
{
	return (uint16_t) ((((to_convert & 0xF0) >> 4) * 10) + (to_convert & 0x0F));
}

/**
 * @brief convert numerical value to RTC time representation
 *
 * @param to_convert	numerical value to convert
 *
 * @return RTC representation of the value
 */
static inline uint8_t rtc_bin_to_bcd(uint8_t to_convert)
{
	return (uint8_t) ((((to_convert / 10) << 4) & 0xF0) | (to_convert % 10));
}

/**
 * @brief start the RTC
 *
 * @param dev	device data structure
 *
 * @return always 0
 */
static int counter_rx_rtc_start(const struct device *dev)
{
	DEV_CFG(cfg, dev);

	/* set the START bit (b0) of RCR2 */
	WRITE_BIT(*cfg->rcr2, 0, 1);
	/* wait until the bit is actually set */
	while ((*cfg->rcr2 & BIT(0)) != 1) {
		arch_nop();
	}
	return 0;
}

static int counter_rx_rtc_stop(const struct device *dev)
{
	DEV_CFG(cfg, dev);

	/* clear the START bit (b0) of RCR2 */
	WRITE_BIT(*cfg->rcr2, 0, 0);
	/* wait until the bit is actually cleared */
	while ((*cfg->rcr2 & BIT(0)) != 0) {
		arch_nop();
	}
	return 0;
}

/**
 * @brief get the value of the RCT counter which is unix time since epoch in seconds (API function)
 *
 * @param dev		RCT device data structure
 * @param ticks		variable to write the unix time to
 *
 * @return 0 on success or -EINVAL if no valid time could be read from the RTC
 */
static int counter_rx_rtc_get_value(const struct device *dev, uint32_t *ticks)
{
	DEV_CFG(cfg, dev);
	struct tm current_time;
	time_t utime;

	current_time.tm_sec = rtc_bcd_to_bin(*cfg->rseccnt & 0x7fu);
	current_time.tm_min = rtc_bcd_to_bin(*cfg->rmincnt & 0x7fu);
	current_time.tm_hour = rtc_bcd_to_bin(*cfg->rhrcnt & 0x3fu);
	current_time.tm_mday = rtc_bcd_to_bin(*cfg->rdaycnt);
	/* libc struct tm counts months 0-11, RX (and apparently 55xx) 1-12 */
	current_time.tm_mon = rtc_bcd_to_bin(*cfg->rmoncnt) - 1;
	/* libc struct tm counts years since 1900, RX since 2000, 55xx since 0 */
	current_time.tm_year = rtc_bcd_to_bin(*cfg->ryrcnt) + 100;

	utime = mktime(&current_time);

	if (utime < 0) {
		return -EINVAL;
	}

	*ticks = (uint32_t) utime;

	return 0;
}

static int counter_rx_rtc_set_alarm(const struct device *dev, uint8_t chan_id,
					const struct counter_alarm_cfg *alarm_cfg)
{
	/* while the RX RTC does support alarms, they are not needed in the current
	 * application and hence not implemented here
	 */
	return -ENOTSUP;
}

static int counter_rx_rtc_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	/* while the RX RTC does support alarms, they are not needed in the current
	 * application and hence not implemented here
	 */
	return 0;
}

static int counter_rx_rtc_set_top_value(const struct device *dev, const struct counter_top_cfg *cfg)
{
	/* this function is not supported by the RTC */
	return -ENOTSUP;
}

static uint32_t counter_rx_rtc_get_pending_int(const struct device *dev)
{
	return 0;
}

static uint32_t counter_rx_rtc_get_top_value(const struct device *dev)
{
	return UINT32_MAX;
}

static uint32_t counter_rx_rtc_get_guard_period(const struct device *dev, uint32_t flags)
{
	return 0;
}

static int counter_rx_rtc_set_guard_period(const struct device *dev, uint32_t ticks, uint32_t flags)
{
	return -ENOTSUP;
}

static const struct counter_driver_api counter_rx_rtc_api = {
	.start = counter_rx_rtc_start,
	.stop = counter_rx_rtc_stop,
	.get_value = counter_rx_rtc_get_value,
	.set_alarm = counter_rx_rtc_set_alarm,
	.cancel_alarm = counter_rx_rtc_cancel_alarm,
	.set_top_value = counter_rx_rtc_set_top_value,
	.get_pending_int = counter_rx_rtc_get_pending_int,
	.get_top_value = counter_rx_rtc_get_top_value,
	.get_guard_period = counter_rx_rtc_get_guard_period,
	.set_guard_period = counter_rx_rtc_set_guard_period,
};

/* according to Renesas RX66N user manual section 33.6.5, "The value written to the count registers,
 * alarm registers, year alarm enable register, bits RCR2.AADJE, AADJP, and HR24, RCR3, RCR4, RFRH,
 * or RFRL register is reflected when four read operations are performed after writing.", so after
 * writing to one of these registers, read four times.
 */
#define RTC_DUMMY_READ          (3)

/**
 * @brief write 8 bit RTC register and read it (RTC_DUMMY_READ + 1) times to ensure it is set
 *
 * @param reg	pointer to the register
 * @param value	value to set
 */
static inline void set_rtc_reg_8(volatile uint8_t *reg, uint8_t value)
{
	uint8_t dummy_value;

	*reg = value;
	for (uint8_t i = 0; i < RTC_DUMMY_READ; i++) {
		dummy_value = *reg;
	}
}

/**
 * @brief write 16 bit RTC register and read it (RTC_DUMMY_READ + 1) times to ensure it is set
 *
 * @param reg	pointer to the register
 * @param value	value to set
 */
static inline void set_rtc_reg_16(volatile uint16_t *reg, uint16_t value)
{
	uint16_t dummy_value;

	*reg = value;
	for (uint8_t i = 0; i < RTC_DUMMY_READ; i++) {
		dummy_value = *reg;
	}
}


/**
 * @see zephyr\include\drivers\rtc\rx.h
 */
int rx_rtc_set_time(const struct device *dev, time_t unix_time)
{
	DEV_CFG(cfg, dev);
	struct tm set_time;
	uint8_t clock_state;

	if (gmtime_r(&unix_time, &set_time) == NULL) {
		return -EINVAL;
	}

	/* store initial clock state and stop if it was running */
	clock_state = *cfg->rcr2 & BIT(0);
	if (clock_state) {
		counter_rx_rtc_stop(dev);
	}

	/* set RTC to 24h mode by setting RCR2.HR24 bit (b6) */
	WRITE_BIT(*cfg->rcr2, 6, 1);

	set_rtc_reg_8(cfg->rseccnt, rtc_bin_to_bcd(set_time.tm_sec));
	set_rtc_reg_8(cfg->rmincnt, rtc_bin_to_bcd(set_time.tm_min));
	set_rtc_reg_8(cfg->rhrcnt, rtc_bin_to_bcd(set_time.tm_hour));

	set_rtc_reg_8(cfg->rdaycnt, rtc_bin_to_bcd(set_time.tm_mday));
	set_rtc_reg_8(cfg->rmoncnt, rtc_bin_to_bcd(set_time.tm_mon + 1));

	set_rtc_reg_16(cfg->ryrcnt, rtc_bin_to_bcd(set_time.tm_year - 100));

	/* restart clock (if it was running at the start of the function) */
	if (clock_state) {
		counter_rx_rtc_start(dev);
	}
	return 0;
}

#define COUNTER_RX_RTC_INIT(id) \
	static struct rx_rtc_cfg rx_rtc_cfg##id = { \
		.r64cnt = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, R64CNT), \
		.rseccnt = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, RSECCNT), \
		.rmincnt = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, RMINCNT), \
		.rhrcnt = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, RHRCNT), \
		.rdaycnt = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, RDAYCNT), \
		.rmoncnt = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, RMONCNT), \
		.ryrcnt = (volatile uint16_t *)DT_INST_REG_ADDR_BY_NAME(id, RYRCNT), \
		.rcr1 = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, RCR1), \
		.rcr2 = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, RCR2), \
		.rcr3 = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, RCR3), \
		.rcr4 = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, RCR4), \
		.radj = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, RADJ), \
	}; \
	static int counter_rx_rtc_init##id(const struct device *dev) \
		{ \
			DEV_CFG(cfg, dev); \
			counter_rx_rtc_stop(dev); \
			/* reset the clock */ \
			WRITE_BIT(*cfg->rcr2, 1, 1); \
			/* and wait until reset is complete*/ \
			while ((*cfg->rcr2 & BIT(1)) != 0) { \
				arch_nop(); \
			} \
			COND_CODE_1(DT_INST_PROP(id, subclock_drive_capacity_low), \
				(set_rtc_reg_8(cfg->rcr3, (*cfg->rcr3 & 0xf1) | 0x02);), \
				(set_rtc_reg_8(cfg->rcr3, (*cfg->rcr3 & 0xf1) | 0x0c);)) \
			COND_CODE_1(DT_INST_PROP(id, use_subclock), \
				(WRITE_BIT(*cfg->rcr4, 0, 0); \
				WRITE_BIT(*cfg->rcr3, 0, 1);), \
				(WRITE_BIT(*cfg->rcr4, 0, 1); \
				WRITE_BIT(*cfg->rcr3, 0, 0);)) \
			/* this implementation assumes calendar mode */ \
			while ((*cfg->rcr2 & BIT(7)) != 0) { \
				WRITE_BIT(*cfg->rcr2, 7, 0); \
			} \
			WRITE_BIT(*cfg->rcr2, 4, DT_INST_PROP(id, automatic_adjustment)); \
			while (((*cfg->rcr2 & BIT(4)) >> 4) != \
					DT_INST_PROP(id, automatic_adjustment)) { \
				arch_nop(); \
			} \
			WRITE_BIT(*cfg->rcr2, 5, \
				DT_INST_PROP(id, automatic_adjustment_10s_period)); \
			while (((*cfg->rcr2 & BIT(5)) >> 5) != \
					DT_INST_PROP(id, automatic_adjustment_10s_period)) { \
				arch_nop(); \
			} \
			set_rtc_reg_8(cfg->radj, DT_INST_PROP_OR(id, adjustment_value, 0)); \
			/* start RTC */ \
			counter_rx_rtc_start(dev); \
			return 0; \
		} \
	DEVICE_DT_INST_DEFINE(id, \
		&counter_rx_rtc_init##id, \
		NULL, \
		NULL, \
		&rx_rtc_cfg##id, \
		POST_KERNEL, \
		CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
		&counter_rx_rtc_api);

DT_INST_FOREACH_STATUS_OKAY(COUNTER_RX_RTC_INIT)
