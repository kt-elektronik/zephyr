/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_timer

#include <device.h>
#include <irq.h>
#include <soc.h>
#include <sys_clock.h>
#include <drivers/clock_control.h>
#include <drivers/timer/system_timer.h>
#include <drivers/clock_control/clock_control_rx65n.h>

#if !DT_HAS_CHOSEN(rx65n_tick_timer) || !DT_HAS_CHOSEN(rx65n_cycle_timer)
#error rx65n,tick-timer and rx65n,cycle-timer have to be chosen
#else

#define TICK_TIMER	DT_CHOSEN(rx65n_tick_timer)
#define CYCLE_TIMER	DT_CHOSEN(rx65n_cycle_timer)

#if DT_REG_SIZE_BY_NAME(TICK_TIMER, CMCNT) == 4
#define TICK_TIMER_32BIT
#define TICK_TIMER_CMCR tick_timer_cfg.cmwcr
#elif DT_REG_SIZE_BY_NAME(TICK_TIMER, CMCNT) == 2
#define TICK_TIMER_16BIT
#define TICK_TIMER_CMCR tick_timer_cfg.cmcr
#else
#error tick timer has to be 16-bit or 32-bit (as determined by size of CMCNT register)
#endif /* DT_REG_SIZE_BY_NAME(TICK_TIMER, CMCNT) == 4 */

#if DT_REG_SIZE_BY_NAME(CYCLE_TIMER, CMCNT) == 4
#define CYCLE_TIMER_32BIT
#define CYCLE_TIMER_CMCR cycle_timer_cfg.cmwcr
#elif DT_REG_SIZE_BY_NAME(CYCLE_TIMER, CMCNT) == 2
#define CYCLE_TIMER_16BIT
#define CYCLE_TIMER_CMCR cycle_timer_cfg.cmcr
#else
#error tick timer has to be 16-bit or 32-bit (as determined by size of CMCNT register)
#endif /* DT_REG_SIZE_BY_NAME(CYCLE_TIMER, CMCNT) == 4 */

#define TICKLESS (IS_ENABLED(CONFIG_TICKLESS_KERNEL))

#define CYCLES_PER_SEC (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC)
#define TICKS_PER_SEC (CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define CYCLES_PER_TICK (CYCLES_PER_SEC / TICKS_PER_SEC)

/* Naming convention regarding cycles:
 * cmt timer: clock cycles
 * Zephyr OS: hw cycles
 */

#if defined(CYCLE_TIMER_16BIT) || DT_SAME_NODE(CYCLE_TIMER, TICK_TIMER)
	/* elapsed hw cycles since system boot */
	static uint32_t accumulated_hw_cycles;
#endif /* defined(CYCLE_TIMER_16BIT) || DT_SAME_NODE(CYCLE_TIMER, TICK_TIMER) */

/* Maximum ticks allowed for 16-bit cmt0 for TICKLESS. It is calculated once in
 * sys_clock_driver_init() to avoid repeated division.
 */
static uint32_t max_ticks;

/* clock_cycles_per_tick is calculated once in
 * sys_clock_driver_init() to avoid repeated division of sub_clk_freq_hz
 * and TICKS_PER_SEC. This is justified as clock frequency should not
 * change during operation without undesired side effects on peripheral
 * devices.
 */
static uint32_t clock_cycles_per_tick;

/* HW cycles per cmt0 clock cycles calculated once in
 * sys_clock_driver_init() to avoid repeated calculation
 */
static uint32_t hw_cycles_per_clock_cycles;

struct timer_ctrl_rx65n_cfg {
	uint32_t base; /* base address */
};

/**
 * @brief structure of the CMCR register for a 16 bit timer
 */
struct cmcr {
	uint16_t cks : 2;
	uint16_t r0 : 4;
	uint16_t cmie : 1;
	uint16_t r1 : 9;
};

/**
 * @brief structure of the CMWCR register for a 32 bit timer
 */
struct cmwcr {
	uint16_t cks : 2;
	uint16_t r0 : 1; /* 1 bit reserved */
	uint16_t cmie : 1;
	uint16_t ic0ie : 1;
	uint16_t ic1ie : 1;
	uint16_t oc0ie : 1;
	uint16_t oc1ie : 1;
	uint16_t r1 : 1; /* 1 bit reserved */
	uint16_t cms : 1;
	uint16_t r2 : 3; /* 3 bits reserved */
	uint16_t cclr : 3;
};

/**
 * @brief structure of the CMWIOR register for a 32 bit timer
 */
struct cmwior {
	uint16_t ic0 : 2;
	uint16_t ic1 : 2;
	uint16_t ic0e : 1;
	uint16_t ic1e : 1;
	uint16_t r0 : 2;
	uint16_t oc0 : 2;
	uint16_t oc1 : 2;
	uint16_t oc0e : 1;
	uint16_t oc1e : 1;
	uint16_t r2 : 1;
	uint16_t cmwe : 1;
};

/**
 * @brief structure containing the configuration of a timer
 */
struct timer_rx65n_cfg {
	volatile uint32_t *cmcnt;
	volatile uint32_t *cmcor;
	volatile uint16_t *cmstr;
	volatile struct cmcr *cmcr;
	volatile struct cmwcr *cmwcr;
	volatile struct cmwior *cmwior;
	uint8_t start_bit;
	uint32_t max_value;
	uint8_t cks;
	uint8_t irq;
};

/**
 * @brief configuration of the "tick timer", i.e. the timer responsible for
 *   supplying regular "ticks" to the kernel.
 */
static const struct timer_rx65n_cfg tick_timer_cfg = {
	.cmcnt = (uint32_t *)DT_REG_ADDR_BY_NAME(TICK_TIMER, CMCNT),
	.cmcor = (uint32_t *)DT_REG_ADDR_BY_NAME(TICK_TIMER, CMCOR),
	.cmstr = (uint16_t *)DT_REG_ADDR_BY_NAME(TICK_TIMER, CMSTR),
	.start_bit = DT_PROP_OR(TICK_TIMER, start_bit, 0),
	.cks = DT_PROP_OR(TICK_TIMER, cks, 0),
#if defined(TICK_TIMER_32BIT)
	.cmcr = NULL,
	.cmwcr = (struct cmwcr *)DT_REG_ADDR_BY_NAME(TICK_TIMER, CMCR),
	.cmwior = (struct cmwior *)DT_REG_ADDR_BY_NAME(TICK_TIMER, CMWIOR),
	.max_value = 0xffffffff,
#else
	.cmcr = (struct cmcr *)DT_REG_ADDR_BY_NAME(TICK_TIMER, CMCR),
	.cmwcr = NULL,
	.cmwior = NULL,
	.max_value = 0xffff,
	.irq = DT_IRQN(TICK_TIMER),
#endif /* defined(TICK_TIMER_32BIT) */
};

/**
 * @brief configuration of the high precision "cycle timer", i.e. the timer
 *   supplying the result of z_timer_cycle_get_32
 */
static const struct timer_rx65n_cfg cycle_timer_cfg = {
	.cmcnt = (uint32_t *)DT_REG_ADDR_BY_NAME(CYCLE_TIMER, CMCNT),
	.cmcor = (uint32_t *)DT_REG_ADDR_BY_NAME(CYCLE_TIMER, CMCOR),
	.cmstr = (uint16_t *)DT_REG_ADDR_BY_NAME(CYCLE_TIMER, CMSTR),
	.start_bit = DT_PROP_OR(CYCLE_TIMER, start_bit, 0),
#if defined(CYCLE_TIMER_32BIT)
	.cmwcr = (struct cmwcr *)DT_REG_ADDR_BY_NAME(CYCLE_TIMER, CMCR),
	.cmwior = (struct cmwior *)DT_REG_ADDR_BY_NAME(CYCLE_TIMER, CMWIOR),
	.max_value = 0xffffffff,
#else
	.cmcr = (struct cmcr *)DT_REG_ADDR_BY_NAME(CYCLE_TIMER, CMCR),
	.cmwior = NULL,
	.max_value = 0xffff,
#endif /* defined(CYCLE_TIMER_32BIT) */
};

/**
 * @brief convert counts (the values of the cmcnt and cmcor registers) to "clock
 *   cycles" depending on the timer clock divider cks
 *
 * @param counts	number of timer counts to convert
 * @param cks		value of the timer clock divider setting
 *
 * @returns		number of clock cycles
 */
#define COUNTS_TO_CLOCK_CYCLES(counts, cks) ((counts) << (2 * (cks)))

/**
 * @brief convert clock cycles into counts
 *
 * @param clock_cycles	number of clock cycles to convert
 * @param cks		value of the clock divider setting
 */
#define CLOCK_CYCLES_TO_COUNTS(cycles, cks) ((cycles) >> (2 * (cks)))

/**
 * @brief determine the clock divider setting cks for a timer
 *
 * @param cfg	configuration structure of the timer
 *
 * @returns	the value of cks
 */
static inline uint8_t get_cks(const struct timer_rx65n_cfg *cfg)
{
	return cfg->cmcr == NULL ? cfg->cmwcr->cks : cfg->cmcr->cks;
}

/**
 * @brief set the clock divider setting cks for a timer
 *
 * @param cfg	configuration structure of the timer
 * @param cks	value to set for cks
 */
static inline void set_cks(const struct timer_rx65n_cfg *cfg, uint8_t cks)
{
	if (cfg->cmcr == NULL) {
		cfg->cmwcr->cks = cks;
	} else {
		cfg->cmcr->cks = cks;
	}
}

/**
 * @brief get the value of a timer count register (CMCNT)
 *
 * @param cfg	configuration structure of the timer
 *
 * @returns	value of the CMCNT register in counts
 */
static inline uint32_t get_cmcnt(const struct timer_rx65n_cfg *cfg)
{
	if (cfg->max_value == 0xffff) {
		return *((uint16_t *) cfg->cmcnt);
	} else {
		return *cfg->cmcnt;
	}
}

/**
 * @brief set the value of a timer count register (CMCNT)
 *
 * @param cfg	configuration structure to the timer
 * @param cmcnt	value to set CMCNT to
 */
static inline void set_cmcnt(const struct timer_rx65n_cfg *cfg, uint32_t cmcnt)
{
	if (cfg->max_value == 0xffff) {
		*((uint16_t *) cfg->cmcnt) = (uint16_t) cmcnt;
	} else {
		*cfg->cmcnt = cmcnt;
	}
}

/**
 * @brief get the value of the compare match constant register of a timer
 *
 * @param cfg	the configuration structure of the timer
 *
 * @returns	value of CMCOR in counts
 */
static inline uint32_t get_cmcor(const struct timer_rx65n_cfg *cfg)
{
	if (cfg->max_value == 0xffff) {
		return *((uint16_t *) cfg->cmcor);
	} else {
		return *cfg->cmcor;
	}
}

/**
 * @brief set the value of the compare match constsn register of a timer
 *
 * @param cfg	the configuration structure of the timer
 * @param cmcor	the value to set CMCOR to
 */
static inline void set_cmcor(const struct timer_rx65n_cfg *cfg, uint32_t cmcor)
{
	if (cfg->max_value == 0xffff) {
		*((uint16_t *) cfg->cmcor) = (uint16_t) cmcor;
	} else {
		*cfg->cmcor = cmcor;
	}
}

/**
 * @brief read the current value of a timer in clock cycles
 *
 * @param cfg	configuration structure of the timer
 *
 * @returns	current value of the timer in clock cycles
 */
static inline uint32_t get_clock_cycles(const struct timer_rx65n_cfg *cfg)
{
	return COUNTS_TO_CLOCK_CYCLES(get_cmcnt(cfg), get_cks(cfg));
}

/**
 * @brief Starts a timer. Must be called while IRQ-locked.
 *
 * @param cmcor    timer overflow (in counts)
 * @param cks      clock divider index (in clock cycles per second)
 */
static inline void timer_rx65n_start(const struct timer_rx65n_cfg *cfg, uint32_t cmcor, uint8_t cks)
{

	/* stop timer */
	WRITE_BIT(*cfg->cmstr, cfg->start_bit, false);

	set_cmcor(cfg, cmcor);
	set_cks(cfg, cks);
	/* clear counter */
	set_cmcnt(cfg, 0);

	/* Start timer */
	WRITE_BIT(*cfg->cmstr, cfg->start_bit, true);
}

/**
 * @brief Callback function to be started at cmt0 timer overflow. Triggers a
 * mandatory sys_clock_announce() for announcing ticks elapsed since last
 * announce call or system boot. Announces only one tick when configured
 * TICKFULL.
 *
 * @param unused unused
 */
static void timer_rx65n_irq_handler(void *unused)
{
	const int key = irq_lock();

	ARG_UNUSED(unused);

	uint32_t unannounced_clock_cycles =
		COUNTS_TO_CLOCK_CYCLES(get_cmcor(&tick_timer_cfg), get_cks(&tick_timer_cfg));

	#if DT_SAME_NODE(CYCLE_TIMER, TICK_TIMER)
		accumulated_hw_cycles += unannounced_clock_cycles * hw_cycles_per_clock_cycles;
	#endif

	/* update of accumulated cycles after cmt0 reset */

	sys_clock_announce(TICKLESS ? (unannounced_clock_cycles / clock_cycles_per_tick) : 1);

	irq_unlock(key);
}

#if defined(CYCLE_TIMER_16BIT) && !DT_SAME_NODE(CYCLE_TIMER, TICK_TIMER)
/**
 * @brief isr for 16-bit cycle timer
 *
 * as the hw_cycle timer is supposed to overflow only after 0xffffffff hw cycles,
 * if a 16-bit timer is chosen for the cycle timer hw cycles have to be
 * accumulated then the timer overflows. This can result in inaccurate cycle
 * counts between the overflow and the execution of the isr (this can happen),
 * so using a 32-bit timer is strongly recommended
 */
static void cycle_timer_isr(void *unused)
{
	accumulated_hw_cycles +=
		COUNTS_TO_CLOCK_CYCLES(get_cmcor(&cycle_timer_cfg), get_cks(&cycle_timer_cfg))
		* hw_cycles_per_clock_cycles;
}
#endif /* defined(CYCLE_TIMER_16BIT) && !DT_SAME_NODE(CYCLE_TIMER, TICK_TIMER) */

int sys_clock_driver_init(const struct device *device)
{
	ARG_UNUSED(device);

	uint32_t tick_timer_sub_clk_freq_hz;
	uint32_t cycle_timer_sub_clk_freq_hz;
	struct clock_control_rx65n_subsys tick_timer_subsys =
		DT_CLOCK_RX65N_SUB_SYSTEM(TICK_TIMER, clock_subsystems);
	struct clock_control_rx65n_subsys cycle_timer_subsys =
		DT_CLOCK_RX65N_SUB_SYSTEM(CYCLE_TIMER, clock_subsystems);

	#if DT_NODE_HAS_PROP(TICK_TIMER, interrupts)
		IRQ_CONNECT(DT_IRQN(TICK_TIMER), DT_IRQ(TICK_TIMER, priority),
			timer_rx65n_irq_handler, NULL, DT_IRQ(TICK_TIMER, flags));
		irq_enable(DT_IRQN(TICK_TIMER));
	#elif DT_NODE_HAS_PROP(TICK_TIMER, sw_ints)
		#error software interrupt controller not yet implemented
	#else
		#error tick timer needs either interrupts or sw_ints
	#endif /* DT_NODE_HAS_PROP(TICK_TIMER, interrupts) */

	#if defined(CYCLE_TIMER_16BIT) && !DT_SAME_NODE(CYCLE_TIMER, TICK_TIMER)
		#if DT_NODE_HAS_PROP(CYCLE_TIMER, interrupts)
			IRQ_CONNECT(DT_IRQN(CYCLE_TIMER), DT_IRQ(CYCLE_TIMER, priority),
				cycle_timer_isr, NULL, DT_IRQ(CYCLE_TIMER, flags));
			irq_enable(DT_IRQN(CYCLE_TIMER));
		#elif DT_NODE_HAS_PROP(CYCLE_TIMER, sw_ints)
			#error software interrupt controller not yet implemented
		#else
			#error 16-bit cycle timer needs either interrupts or sw_ints
		#endif /* DT_NODE_HAS_PROP(CYCLE_TIMER, interrupts) */
	#endif /* defined(CYCLE_TIMER_16BIT) && !DT_SAME_NODE(CYCLE_TIMER, TICK_TIMER) */

	const int key = irq_lock();

	clock_control_get_rate(DEVICE_DT_GET(DT_PHANDLE(TICK_TIMER, clock)),
		&tick_timer_subsys, (clock_control_subsys_t)&tick_timer_sub_clk_freq_hz);
	clock_control_get_rate(DEVICE_DT_GET(DT_PHANDLE(TICK_TIMER, clock)),
		&cycle_timer_subsys, (clock_control_subsys_t)&cycle_timer_sub_clk_freq_hz);

	/* clock cycles per tick to avoid repeated divisions. Since the smallest
	 * possible clock divider is 8 (cks = 0), one clock cycle is defined as
	 * 8 cycles of the clock source
	 */
	clock_cycles_per_tick = (uint32_t)(tick_timer_sub_clk_freq_hz / TICKS_PER_SEC / 8);
	max_ticks = COUNTS_TO_CLOCK_CYCLES((uint64_t)tick_timer_cfg.max_value, tick_timer_cfg.cks)
			/ clock_cycles_per_tick;

	/* HW cycles per cmt0 clock cycles to avoid repeated calculation */
	/* [os cycles/clock cycles]=(os cycles/sec)/(clock cycles/sec)*/
	hw_cycles_per_clock_cycles = CYCLES_PER_SEC / cycle_timer_sub_clk_freq_hz * 8;

	clock_control_on(DEVICE_DT_GET(DT_PHANDLE(TICK_TIMER, clock)),
		(clock_control_subsys_t)&tick_timer_subsys);
	#if !DT_SAME_NODE(CYCLE_TIMER, TICK_TIMER)
		clock_control_on(DEVICE_DT_GET(DT_PHANDLE(CYCLE_TIMER, clock)),
			(clock_control_subsys_t)&cycle_timer_subsys);
	#endif /* !DT_SAME_NODE(CYCLE_TIMER, TICK_TIMER) */

	/* enable compare match interrupt for tick timer overflow */
	TICK_TIMER_CMCR->cmie = 1;
	if (tick_timer_cfg.cmwior != NULL) {
		tick_timer_cfg.cmwior->cmwe = 1;
	}

	#if DT_SAME_NODE(CYCLE_TIMER, TICK_TIMER)
		/* no need for separate cycle timer configuration */
	#elif defined(CYCLE_TIMER_32BIT)
		/* to be sure: disable compare match interrupt for cycle timer overflow */
		CYCLE_TIMER_CMCR->cmie = 0;
		cycle_timer_cfg.cmwior->cmwe = 1;
	#elif defined(CYCLE_TIMER_16BIT)
		/* a 16-bit cycle timer actually needs the interrupt */
		CYCLE_TIMER_CMCR->cmie = 1;
	#endif /* DT_SAME_NODE(CYCLE_TIMER, TICK_TIMER) */

	/* start tick timer to trigger first after one tick - this works both
	 * for TICKLESS, where the kernel will set a timeout when
	 * sys_clock_announce() is called the first time and for non-TICKLESS,
	 * where this is the correct interval anyway.
	 */
	timer_rx65n_start(&tick_timer_cfg,
			CLOCK_CYCLES_TO_COUNTS(clock_cycles_per_tick, tick_timer_cfg.cks),
			tick_timer_cfg.cks);
	#if !DT_SAME_NODE(CYCLE_TIMER, TICK_TIMER)
		timer_rx65n_start(&cycle_timer_cfg, cycle_timer_cfg.max_value,
				cycle_timer_cfg.cks);
	#endif /* !DT_SAME_NODE(CYCLE_TIMER, TICK_TIMER) */

	irq_unlock(key);

	return 0;
}

uint32_t sys_clock_elapsed(void)
{
	/* elapsed ticks since last sys_clock_announce() */
	return TICKLESS ?
		get_clock_cycles(&tick_timer_cfg) / clock_cycles_per_tick : 0;
}

uint32_t z_timer_cycle_get_32(void)
{
	#if defined(CYCLE_TIMER_32BIT)
		return get_clock_cycles(&cycle_timer_cfg) * hw_cycles_per_clock_cycles;
	#elif defined(CYCLE_TIMER_16BIT) || DT_SAME_NODE(CYCLE_TIMER, TICK_TIMER)
		return accumulated_hw_cycles
			+ get_clock_cycles(&cycle_timer_cfg) * hw_cycles_per_clock_cycles;
	#else
		#error cycle timer not configured properly
	#endif
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

	#if TICKLESS
		const int key = irq_lock();

		uint32_t ticks_elapsed = sys_clock_elapsed();
		uint32_t max_remaining_ticks = max_ticks - ticks_elapsed;

		ticks = ticks <= max_remaining_ticks ? ticks : max_remaining_ticks;

		set_cmcor(&tick_timer_cfg,
			CLOCK_CYCLES_TO_COUNTS((ticks_elapsed + ticks) * clock_cycles_per_tick, 0));


		if (get_cmcnt(&tick_timer_cfg) > get_cmcor(&tick_timer_cfg)) {
			/* if ticks is 0 and the timer was close to another
			 * tick before this function was called, cmcor could
			 * have been set to a value smaller than cmcnt. In this
			 * case, the timer will continue without interrupt which
			 * can "freeze" the system in idle. Instead, put the
			 * timer into the "overflow state" by resetting and
			 * triggering the interrupt
			 */
			set_cmcnt(&tick_timer_cfg,
				get_cmcnt(&tick_timer_cfg) % clock_cycles_per_tick);
			trigger_irq(tick_timer_cfg.irq);
		}
		irq_unlock(key);
	#endif
}

#endif /* !DT_HAS_CHOSEN(rx65n_tick_timer) || !DT_HAS_CHOSEN(rx65n_cycle_timer) */
