/*
 * Clock configuration for RX65N series.
 * Main clock, HOCO, LOCO, sub clock and PLL are implemented.
 *
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_system_clock

#include <errno.h>
#include <soc.h>
#include <drivers/clock_control.h>
#include <drivers/clock_control/clock_control_rx65n.h>

/* Renesas FIT module for iodefine.h data structures */
#include <platform.h>

/* Enables writing to the registers related to
 *  - the clock generation circuit (bit 0)
 * - operating modes, clock generation circuit, low power consumption,
 *   and software reset (bit 1)
 * - the LVD (bit 3)
 */
#define REGISTER_WRITE_ENABLE(val) \
	((volatile struct st_system *)DT_REG_ADDR(DT_NODELABEL(system_clock)))->PRCR.WORD = \
	0xA500u | val<<3 | val<<1 | val<<0;

#define RX_CLK_CTRL_PCLKD 0
#define RX_CLK_CTRL_PCLKC 1
#define RX_CLK_CTRL_PCLKB 2
#define RX_CLK_CTRL_PCLKA 3
#define RX_CLK_CTRL_BCLK 4
#define RX_CLK_CTRL_ICLK 5
#define RX_CLK_CTRL_FCLK 6

#define RX_CLK_CTRL_LOCO 0
#define RX_CLK_CTRL_HOCO 1
#define RX_CLK_CTRL_MAIN 2
#define RX_CLK_CTRL_SUB 3
#define RX_CLK_CTRL_PLL 4

/* 9.2.4 System Clock Control Register 3 (SCKCR3), R01UH0590EJ0230 Rev.2.30 */
#define RX_CLK_FREQ_LOCO_HZ 240000	/* 000: LOCO (fixed: 240 kHz) */
#define RX_CLK_FREQ_HOCO_16_HZ 16000000 /* 001: HOCO (fixed: 16 MHz) */
#define RX_CLK_FREQ_HOCO_18_HZ 18000000 /* 001: HOCO (fixed: 18 MHz) */
#define RX_CLK_FREQ_HOCO_20_HZ 20000000 /* 001: HOCO (fixed: 20 MHz) */
					/* 010: Main clk (dev tree)) */
#define RX_CLK_FREQ_MAIN_HZ DT_PROP(DT_NODELABEL(system_clock), main_clock_frequency)
#define RX_CLK_SUB 32768 /* 011: Sub-clock (fixed: 32.768 kHz) */
/* 100: PLL (Main clk (dev tree) or HOCO (fixed, HOCOCR2: 16, 18 or 20 MHz)) */

struct clk_ctrl_rx65n_cfg {
	uint32_t base; /* base address */
};

struct clock_control_rx65n_subclock_cfg {
	uint8_t prescale;
};

#define DEV_CFG(dev) ((struct clk_ctrl_rx65n_cfg *)(dev->config))
#define SUBCLOCK_CFG(dev) ((struct clock_control_rx65n_subclock_cfg *)(dev->config))
#define DEV_BASE(dev) (DEV_CFG(dev)->base)

/* main clock source frequency - this frequency is determined from the
 * configuration during initialization and will not change afterwards
 */
static uint32_t main_clk_src_freq_hz;

/**
 * @brief Calculates prescale factor for sub-clocks PCKD, PCKC, PCKB, PCKA, BCK, ICK and FCK to be
 * set in the System Clock Control Register (SCKCR) for Peripheral Module Clock (PCLK) selection
 *
 * @param prescale    prescale divider value (1, 2, 4, 8, 16, 32 or 64), derived from
 * 0 0 0 0: 1/1 --> 0
 * 0 0 0 1: 1/2 --> 1
 * 0 0 1 0: 1/4 --> 2
 * 0 0 1 1: 1/8 --> 3
 * 0 1 0 0: 1/16 --> 4
 * 0 1 0 1: 1/32 --> 5
 * 0 1 1 0: 1/64 --> 6
 *
 * @returns Integer prescale settings (divider index to be used in SCKCR)
 */
static uint8_t clock_control_rx65n_get_sckcr_prescale_value(uint8_t prescale)
{
	switch (prescale) {
	case BIT(0):
		return 0;
	case BIT(1):
		return 1;
	case BIT(2):
		return 2;
	case BIT(3):
		return 3;
	case BIT(4):
		return 4;
	case BIT(5):
		return 5;
	case BIT(6):
		return 6;
	default:
		return 0;
	}
}

/**
 * @brief starts the clock for a specific subsystem
 *
 * @param *dev		pointer to a sub clock device driver structure
 * @param sub_system    identifier to clock controller sub-system
 *
 * @returns 0
 */
static int clock_control_rx65n_on(const struct device *dev, clock_control_subsys_t sub_system)
{
	ARG_UNUSED(dev);
	struct clock_control_rx65n_subsys *subsys =
			(struct clock_control_rx65n_subsys *)sub_system;

	REGISTER_WRITE_ENABLE(1);
	/* clear the stop bit to start the clock */
	WRITE_BIT(*subsys->mstpcr, subsys->stop_bit, false);
	REGISTER_WRITE_ENABLE(0);
	return 0;
}

/**
 * @brief stops the clock for a specific subsystem
 *
 * @param *dev		pointer to a sub clock device driver structure
 * @param sub_system    identifier to clock controller sub-system
 *
 * @returns 0
 */
static int clock_control_rx65n_off(const struct device *dev, clock_control_subsys_t sub_system)
{
	ARG_UNUSED(dev);
	struct clock_control_rx65n_subsys *subsys =
			(struct clock_control_rx65n_subsys *)sub_system;

	REGISTER_WRITE_ENABLE(1);
	/* set the stop bit to stop the clock */
	WRITE_BIT(*subsys->mstpcr, subsys->stop_bit, true);
	REGISTER_WRITE_ENABLE(0);
	return 0;
}

/**
 * @brief Calculates frequency of a sub-clock
 *
 * @param *dev		pointer to a sub clock device driver structure
 * @param sub_system    identifier to clock controller sub-system
 * @param *rate    pointer to address that will contain sub-clock frequency (in Hz)
 *
 * @returns 0
 */
static int clock_control_rx65n_get_rate(const struct device *dev, clock_control_subsys_t sub_system,
					uint32_t *rate)
{
	ARG_UNUSED(sub_system);

	*rate = main_clk_src_freq_hz / SUBCLOCK_CFG(dev)->prescale;

	return 0;
}

/**
 * @brief initializes a sub clock device driver
 */
static int clock_control_rx65n_init_subclock(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

/**
 * @brief Initializes the systems main clock source
 *
 * @param *dev	pointer to the main clock source device driver structure
 *
 * @returns 0
 */
static int clock_control_rx65n_init(const struct device *dev)
{
	volatile struct st_system *system = (struct st_system *)DEV_BASE(dev);

#if CONFIG_CLOCK_RX65N_ENABLE_HOCO
	uint32_t hoco_freq_hz = 0;
#endif
	uint32_t pll_freq_hz = 0;

	REGISTER_WRITE_ENABLE(1);

	/* Set prescaler */
	system->SCKCR.BIT.FCK = clock_control_rx65n_get_sckcr_prescale_value(
		DT_PROP(DT_NODELABEL(fclk), prescale));
	system->SCKCR.BIT.PCKA = clock_control_rx65n_get_sckcr_prescale_value(
		DT_PROP(DT_NODELABEL(pclka), prescale));
	system->SCKCR.BIT.PCKB = clock_control_rx65n_get_sckcr_prescale_value(
		DT_PROP(DT_NODELABEL(pclkb), prescale));
	system->SCKCR.BIT.PCKC = clock_control_rx65n_get_sckcr_prescale_value(
		DT_PROP(DT_NODELABEL(pclkc), prescale));
	system->SCKCR.BIT.PCKD = clock_control_rx65n_get_sckcr_prescale_value(
		DT_PROP(DT_NODELABEL(pclkd), prescale));
	system->SCKCR.BIT.BCK =	clock_control_rx65n_get_sckcr_prescale_value(
		DT_PROP(DT_NODELABEL(bclk), prescale));
	system->SCKCR.BIT.ICK =	clock_control_rx65n_get_sckcr_prescale_value
		(DT_PROP(DT_NODELABEL(iclk), prescale));

#if CONFIG_CLOCK_RX65N_ENABLE_MAINCLK
#if CONFIG_CLOCK_RX65N_MAINCLK_SRC_EXT
	system->MOFCR.BIT.MOSEL = 1;
#endif
	/* Set up Main Clock Oscillator driving ability */
	system->MOFCR.BIT.MODRV2 = CONFIG_CLOCK_RX65N_MAINCLK_DRIVEABILITY;
	/* Set up Main Clock Oscillator waiting time */
	system->MOSCWTCR.BYTE = CONFIG_CLOCK_RX65N_MOSCWTCR;
	/* Start main clock */
	system->MOSCCR.BIT.MOSTP = 0;
	/* Wait for main clock oscillator wait counter overflow */
	while (system->OSCOVFSR.BIT.MOOVF != 1)
		;
#endif /* CONFIG_CLOCK_RX65N_ENABLE_MAINCLK */

#if CONFIG_CLOCK_RX65N_ENABLE_HOCO
	const uint32_t hoco_freq[3] = { RX_CLK_FREQ_HOCO_16_HZ, RX_CLK_FREQ_HOCO_18_HZ,
					RX_CLK_FREQ_HOCO_20_HZ };

	hoco_freq_hz = hoco_freq[CONFIG_CLOCK_RX65N_HOCO_FREQ];
	/* Set HOCO frequency */
	system->HOCOCR2.BIT.HCFRQ = CONFIG_CLOCK_RX65N_HOCO_FREQ;
	/* Turn on and start HOCO (is normally already done per default) */
	system->HOCOPCR.BIT.HOCOPCNT = 0;
	system->HOCOCR.BIT.HCSTP = 0;
	while (system->OSCOVFSR.BIT.HCOVF != 1)
		;
#else
	system->HOCOPCR.BIT.HOCOPCNT = 1;
	system->HOCOCR.BIT.HCSTP = 1;
#endif /* CONFIG_CLOCK_RX65N_ENABLE_HOCO */

#if CONFIG_CLOCK_RX65N_SYSCLK_SRC_PLL
		/* Set PLL source */
#if CONFIG_CLOCK_RX65N_PLL_SRC_HOCO
	pll_freq_hz = hoco_freq_hz;
	system->PLLCR.BIT.PLLSRCSEL = 1;
#else
	pll_freq_hz = RX_CLK_FREQ_MAIN_HZ;
	system->PLLCR.BIT.PLLSRCSEL = 0;
#endif
	/* Set PLL circuit */
	system->PLLCR.BIT.PLIDIV = CONFIG_CLOCK_RX65N_PLL_DIVISOR - 1;
	system->PLLCR.BIT.STC = (CONFIG_CLOCK_RX65N_PLL_MULTIPLIER << 1) - 1;
	/* Start PLL */
	system->PLLCR2.BIT.PLLEN = 0;
	/* Wait for PLL wait counter overflow */
	while (system->OSCOVFSR.BIT.PLOVF != 1)
		;
	pll_freq_hz = pll_freq_hz / CONFIG_CLOCK_RX65N_PLL_DIVISOR
		* CONFIG_CLOCK_RX65N_PLL_MULTIPLIER;
#endif /* CONFIG_CLOCK_RX65N_SYSCLK_SRC_PLL */

	/* Disable clock outputs */
	system->SCKCR.BIT.PSTOP0 = 1; /* SDCLCK */
	system->SCKCR.BIT.PSTOP0 = 1; /* BCLCK */

	/* CONFIG_CLOCK_RX65N_SYSCLK_SRC is a build-time define */
#if CONFIG_CLOCK_RX65N_SYSCLK_SRC == RX_CLK_CTRL_LOCO
	main_clk_src_freq_hz = RX_CLK_FREQ_LOCO_HZ;
#elif CONFIG_CLOCK_RX65N_SYSCLK_SRC == RX_CLK_CTRL_HOCO
	main_clk_src_freq_hz = hoco_freq_hz;
#elif CONFIG_CLOCK_RX65N_SYSCLK_SRC ==  RX_CLK_CTRL_MAIN
	main_clk_src_freq_hz = RX_CLK_FREQ_MAIN_HZ;
#elif CONFIG_CLOCK_RX65N_SYSCLK_SRC ==  RX_CLK_CTRL_SUB
	main_clk_src_freq_hz = RX_CLK_SUB;
#elif CONFIG_CLOCK_RX65N_SYSCLK_SRC ==  RX_CLK_CTRL_PLL
	main_clk_src_freq_hz = pll_freq_hz;
#else
#error "Invalid clock configuration"
#endif

#if CONFIG_SOC_SERIES_RX65N
	/* Set number of access wait cycles of the Flash memory according to
	 * the frequency of the system clock ICLK (according to RX651 Group
	 * User's Manual section 9.2.2)
	 * This appears not to be necessary for RX66N MCUs.
	 */
	if (main_clk_src_freq_hz / DT_PROP(DT_NODELABEL(iclk), prescale) < 50000000ul) {
		system->ROMWT.BYTE = 0;
	} else if (main_clk_src_freq_hz / DT_PROP(DT_NODELABEL(iclk), prescale) <= 100000000ul) {
		system->ROMWT.BYTE = 1;
	} else {
		system->ROMWT.BYTE = 2;
	}
#endif

	/* Set System Clock source */
	system->SCKCR3.BIT.CKSEL = CONFIG_CLOCK_RX65N_SYSCLK_SRC;

#if CONFIG_CLOCK_RX65N_ENABLE_LOCO
	/* Start Low-Speed On-Chip Oscillator (LOCO) */
	system->LOCOCR.BIT.LCSTP = 0;
#endif

#if CONFIG_CLOCK_RX65N_ENABLE_IWDT_CLOCK
	/* Enabling IWDT Clock in this way is only effective when not set in
	 * OFS0 register
	 */
	system->ILOCOCR.BIT.ILCSTP = 0;
	while (system->OSCOVFSR.BIT.ILCOVF != 1)
		;
#endif

	REGISTER_WRITE_ENABLE(0);
	return 0;
}

static const struct clock_control_driver_api clock_control_rx65n_api = {
	.on = clock_control_rx65n_on,
	.off = clock_control_rx65n_off,
	.get_rate = clock_control_rx65n_get_rate,
};

static struct clk_ctrl_rx65n_cfg clock_control_rx65n_0_config = { .base = DT_INST_REG_ADDR(0) };

#define INIT_SUBCLOCK(node_id) \
	IF_ENABLED(DT_NODE_HAS_COMPAT(node_id, renesas_rx_sub_clock), \
		(static const struct clock_control_rx65n_subclock_cfg node_id##_cfg = { \
			.prescale = DT_PROP(node_id, prescale) \
		}; \
		DEVICE_DT_DEFINE(node_id, clock_control_rx65n_init_subclock, \
			NULL, NULL, &node_id##_cfg, PRE_KERNEL_1, \
			CONFIG_KERNEL_INIT_PRIORITY_OBJECTS, \
			&clock_control_rx65n_api) \
	));

DEVICE_DT_DEFINE(DT_NODELABEL(system_clock), &clock_control_rx65n_init, NULL,
			NULL, &clock_control_rx65n_0_config,
			PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_OBJECTS, NULL);

DT_FOREACH_CHILD_STATUS_OKAY(DT_NODELABEL(system_clock), INIT_SUBCLOCK)
