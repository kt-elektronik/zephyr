/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief device driver to use Reneasas RX MCU TPUs or MTUs to control PWM channels
 *
 * @note while Reneasas RX Timer Pulse Units (TPUa) and Multi-Function Timer Puse Unit 3 (MTU3a)
 *   are not identical, they can be treated (almost) the same when used as a PWM controller as
 *   far as the Zephyr API is concerned
 * @note currently, only PWM output is supported by this driver, even though the TPUa
 *   also should be able to support PWM capture
 */

#define DT_DRV_COMPAT renesas_rx_pwm

#include <zephyr.h>
#include <device.h>
#include <errno.h>
#include <soc.h>

#include <drivers/pwm.h>
#include <drivers/clock_control.h>
#include <drivers/clock_control/clock_control_rx65n.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(pwm_rx, CONFIG_PWM_LOG_LEVEL);

enum rx_pwm_type_t {
	/* TPU (always operating in PWM mode 2) */
	PWM_RX_TYPE_TPU,
	/* MTU operating in PWM mode 1 */
	PWM_RX_TYPE_MTU1,
	/* MTU operating in PWM mode 2 (not available for all MTUs) */
	PWM_RX_TYPE_MTU2
};

struct tcr {
	/* time prescaler select */
	uint8_t tpsc : 3;
	/* input clock edge select */
	uint8_t ckeg : 2;
	/* counter clear source select */
	uint8_t cclr : 3;
};

struct pwm_rx_cfg {
	/* device type (TPU or MTU) */
	enum rx_pwm_type_t dev_type;
	/* timer control register */
	volatile struct tcr *tcr;
	/* timer mode register */
	volatile uint8_t *tmdr;
	/* timer I/O control register (16 bit or 8 bit depending on number of channels) */
	volatile uint8_t *tior;
	/* timer general registers */
	volatile uint16_t *tgr;
	/* timer counter register */
	volatile uint16_t *tcnt;
	/* timer start register (common for all TPUs) */
	volatile uint8_t *tstr;
	/* timer synchronous register (common for all TPUs) */
	volatile uint8_t *tsyr;
	/* determines which bit in TSTR and TSYR corresponds to this TPU */
	uint8_t bit_idx;
	/* supported number of channels (not necessarily number of used channels) */
	uint8_t max_num_channels;
	/* which TGR is used as counter clear register ? */
	int8_t counter_clear_channel;
	/* operate the device in synchronous mode ? */
	bool synchronous;
	/* prescaler setting for TCR */
	uint8_t prescaler;
	/* clock divider - this is determined by the prescaler setting but depends on the TPU */
	uint16_t clock_divider;
	/* clock responsible for the TPU */
	const struct device *clock;
	/* clock subsystem */
	struct clock_control_rx65n_subsys subsys;
};

#define DEV_CFG(cfg, dev) const struct pwm_rx_cfg *cfg = \
		(const struct pwm_rx_cfg *) dev->config;

struct pwm_rx_data {
	/* device is active (used by power management) */
	bool active;
};

#define DEV_DATA(data, dev) struct pwm_rx_data *data = (struct pwm_rx_data *) dev->data;

#define COUNTER_CLEAR_CHANNEL_VALID(cfg) \
	((cfg->counter_clear_channel >= 0) && (cfg->counter_clear_channel < cfg->max_num_channels))

#define TMDR_MD_PWM_MODE_1	(2)
#define TMDR_MD_PWM_MODE_2	(3)

/* if TPUs are operated in synchronous mode, one of them has to handle the counter clear channel */
static const struct device *synchronous_counter_clear_tpu;
/* if MTUs are operated in synchronous mode, one of them has to handle the counter clear channel */
static const struct device *synchronous_counter_clear_mtu;

/**
 * @brief set the PWM period
 *
 * @param dev			device structure of a PWM device
 * @param period_cylces		new value of the PWM period
 *
 * @return 0 on success or negative error code
 */
static int pwm_rx_set_period(const struct device *dev, uint16_t period_cycles)
{
	DEV_CFG(cfg, dev);

	if (!COUNTER_CLEAR_CHANNEL_VALID(cfg)) {
		return -EINVAL;
	}

	cfg->tgr[cfg->counter_clear_channel] = period_cycles;

	/* for synchronous PWM, the device with the counter clear register has to be started
	 * so that all other synchronous PWMs can work. For non-synchronous PWMs, the clock
	 * has to be started anyway.
	 */
	WRITE_BIT(*cfg->tstr, cfg->bit_idx, true);

	return 0;
}

/* the different pwm states correspond to the value of the four TIOR bits for each state.
 * See also Renesas RX MUC User's manuals. At the beginning of the period (after timer reset),
 * the signal starts in the first state and switches to the second state on compare match with
 * the corresponding TGR register.
 */
/* output always low (0% duty cycle). low -> low (0b0101) */
#define PWM_STATE_0			0x11
/* output switches (1% - 99% duty cycle). high -> low (0b0010)*/
#define PWM_STATE_SWITCHING		0x65
/* output switches (1% - 99% duty cycle) but is inverted. low -> high (0b0001)*/
#define PWM_STATE_SWITCHING_INVERTED	0x12
/* output always high (100% duty cycle). high -> high (0b0110) */
#define PWM_STATE_100			0x66

/**
 * @brief set the timer I/O control register (TIOR) for a PWM in mode 1
 *
 * @param dev		device structure
 * @param pwm		pwm channel to set
 * @param pwm_state	pwm state according to the PWM_STATE_* defines
 */
static void pwm_rx_set_tior_pwm1(const struct device *dev, int pwm, uint8_t pwm_state)
{
	DEV_CFG(cfg, dev);

	/* In PWM mode 1, the lower four bits of the TIOR (IOA/IOC)  determine the signal. We will
	 * always choose the second TGR associated with a PWM channel (TGRB/TGRD) to be the counter
	 * clear register.
	 */

	cfg->tior[pwm] = pwm_state;
}

/**
 * @brief set the timer I/O control register (TIOR) for a PWM in mode 2
 *
 * @param dev		device structure
 * @param pwm		pwm channel to set
 * @param pwm_state	pwm state according to the PWM_STATE_* defines
 */
static void pwm_rx_set_tior_pwm2(const struct device *dev, int pwm, uint8_t pwm_state)
{
	DEV_CFG(cfg, dev);

	/* in pwm mode 2, the higher 4 bits of the pwm_state byte are not used */
	pwm_state &= 0xf;

	if (pwm % 2 == 0) {
		/* I/O settings for even numbered channels are encoded in the lower 4 bytes
		 * of the timer I/O control register
		 */
		cfg->tior[pwm / 2] = (cfg->tior[pwm / 2] & 0xf0) + pwm_state;
	} else {
		/* I/O settings for even numbered channels are encoded in the higher 4 bytes
		 * of the timer I/O control register
		 */
		cfg->tior[pwm / 2] = (cfg->tior[pwm / 2] & 0x0f) + (pwm_state << 4);
	}
}

/**
 * @brief Callback API upon setting the pin
 * See @a pwm_pin_set_cycles() (pwm.h) for argument description
 *
 * @note if the device controls more than one PWM output channel, the given period will be
 *   applied to all channels of this device. If the device is in synchronous mode, the period
 *   will be applied to all PWMs of the same type (TPU/MTU) in synchronous mode.
 */
static int pwm_rx_pin_set(const struct device *dev, uint32_t pwm,
			uint32_t period_cycles, uint32_t pulse_cycles,
			pwm_flags_t flags)
{
	DEV_CFG(cfg, dev);
	uint8_t pwm_state = PWM_STATE_SWITCHING;

	if ((flags & PWM_POLARITY_INVERTED) != 0) {
		pwm_state = PWM_STATE_SWITCHING_INVERTED;
	}

	if ((period_cycles > 0xffffu) || (pulse_cycles  > 0xffffu)) {
		LOG_ERR("invalid period (%u) or pulse_cycles (%u) for 16-bit PWM %s",
				period_cycles, pulse_cycles, dev->name);
		return -EINVAL;
	}

	if (pwm >= cfg->max_num_channels) {
		LOG_ERR("invalid channel %u, %s only has %u channels > ",
				pwm, dev->name, cfg->max_num_channels);
		return -EINVAL;
	}

	if (pulse_cycles == period_cycles) {
		/* 100% duty cycle */
		if (flags & PWM_POLARITY_INVERTED) {
			pwm_state = PWM_STATE_0;
		} else {
			pwm_state = PWM_STATE_100;
		}

		/* The PWM device apparently does not change state if pulse_cycles == period_cycles,
		 * so we have to reduce pulse_cylces by one. Due to the value of pwm_state, the
		 * signal will remain constant at compare match
		 */
		pulse_cycles--;
	}

	if (pulse_cycles == 0) {
		/* 0% duty cycle */
		if (flags & PWM_POLARITY_INVERTED) {
			pwm_state = PWM_STATE_100;
		} else {
			pwm_state = PWM_STATE_0;
		}
	}

	if (cfg->dev_type == PWM_RX_TYPE_MTU1) {
		pwm_rx_set_tior_pwm1(dev, pwm, pwm_state);
	} else {
		pwm_rx_set_tior_pwm2(dev, pwm, pwm_state);
	}

	cfg->tgr[pwm] = (uint16_t)pulse_cycles;
	if (cfg->synchronous) {
		switch (cfg->dev_type) {
		case PWM_RX_TYPE_TPU:
			pwm_rx_set_period(synchronous_counter_clear_tpu, (uint16_t)period_cycles);
			break;
		case PWM_RX_TYPE_MTU1:
		case PWM_RX_TYPE_MTU2:
			pwm_rx_set_period(synchronous_counter_clear_mtu, (uint16_t)period_cycles);
			break;
		default:
			LOG_ERR("Unknown PWM device type %u", cfg->dev_type);
			break;
		}
	} else {
		pwm_rx_set_period(dev, (uint16_t)period_cycles);
	}

	if (*cfg->tcnt > period_cycles) {
		/* if the period has been reduced with this call, it could happen that
		 * the counter is already higher than the new counter clear value and
		 * would continue to run until the 16 bit overrun. In this case, explicitly
		 * reset the counter.
		 */
		WRITE_BIT(*cfg->tstr, cfg->bit_idx, false);
		*cfg->tcnt = 0;
	}

	WRITE_BIT(*cfg->tstr, cfg->bit_idx, true);
	return 0;
}

/**
 * @brief Callback API upon getting cycles per second
 * See @a pwm_get_cycles_per_sec() (pwm.h) for argument description
 *
 * @note for the return value of this function to be correct, the correct clock divider
 *       value has to be given in the device tree.
 */
static int pwm_rx_get_cycles_per_sec(const struct device *dev,
					uint32_t pwm,
					uint64_t *cycles)
{
	DEV_CFG(cfg, dev);
	uint32_t freq_hz;
	int ret = 0;

	ret = clock_control_get_rate(cfg->clock,
				       (clock_control_subsys_t)&cfg->subsys,
				       &freq_hz);

	if (ret == 0) {
		*cycles = freq_hz / cfg->clock_divider;
	}

	return ret;
}

static const struct pwm_driver_api pwm_rx_api = {
	.pin_set = pwm_rx_pin_set,
#ifdef CONFIG_PWM_CAPTURE
	.pin_configure_capture = NULL,
	.pin_enable_capture = NULL,
	.pin_disable_capture = NULL,
#endif /* CONFIG_PWM_CAPTURE */
	.get_cycles_per_sec = pwm_rx_get_cycles_per_sec,
};

/**
 * @brief deactivate a PWM device
 *
 * @param dev		device driver structure
 * @param num_active	pointer to the number of active devices of this type (TPU/MTU)
 *
 * @return 0 on success or negative error code
 */
static int pwm_rx_pm_deactivate(const struct device *dev, uint8_t *num_active)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);
	int ret = 0;

	if (data->active) {
		if (*num_active <= 1) {
			/* last active device of this type - turn off the common clock */
			ret = clock_control_off(cfg->clock,
				(clock_control_subsys_t *)&cfg->subsys);
		}
		data->active = false;
		*num_active -= *num_active ? 1 : 0;
	}

	return ret;
}

/**
 * @brief activate a PWM device
 *
 * @param dev		device driver structure
 * @param num_active	pointer to the number of active devices of this type (TPU/MTU)
 *
 * @return 0 on success or negative error code
 */
static int pwm_rx_pm_activate(const struct device *dev, uint8_t *num_active)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);
	int ret = 0;

	if (!data->active) {
		if (*num_active == 0) {
			/* the first TPU to be activated - turn on the common clock */
			ret = clock_control_on(cfg->clock,
				(clock_control_subsys_t *)&cfg->subsys);
		}
		data->active = true;
		(*num_active)++;
	}

	return ret;
}


/**
 * @brief power management for RX PWMs
 *
 * @param dev		device to control
 * @param action	power management action
 *
 * @returns		0 on success or negative error code
 *
 * @note all TPUs and all MTUs share the same clock, so the clock has to be on as long as at least
 *       one TPU (or one MTU for the MTU clock) is active
 */
static int pwm_rx_pm_control(const struct device *dev, enum pm_device_action action)
{
	DEV_CFG(cfg, dev);
	int ret = 0;
	static uint8_t active_tpus;
	static uint8_t active_mtus;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
	case PM_DEVICE_ACTION_TURN_OFF:
	case PM_DEVICE_ACTION_LOW_POWER:
		if (cfg->dev_type == PWM_RX_TYPE_TPU) {
			pwm_rx_pm_deactivate(dev, &active_tpus);
		} else if (cfg->dev_type == PWM_RX_TYPE_MTU1 || cfg->dev_type == PWM_RX_TYPE_MTU2) {
			pwm_rx_pm_deactivate(dev, &active_mtus);
		}
		break;
	case PM_DEVICE_ACTION_RESUME:
		if (cfg->dev_type == PWM_RX_TYPE_TPU) {
			pwm_rx_pm_activate(dev, &active_tpus);
		} else if (cfg->dev_type == PWM_RX_TYPE_MTU1 || cfg->dev_type == PWM_RX_TYPE_MTU2) {
			pwm_rx_pm_activate(dev, &active_mtus);
		}
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

/**
 * @brief initialize a PWM device
 *
 * @param dev	the device structure
 *
 * @returns	0 on success, negative error code on failure
 */
static int pwm_rx_init(const struct device *dev)
{
	DEV_CFG(cfg, dev);
	bool counter_clear_channel_set = false;

	pwm_rx_pm_control(dev, PM_DEVICE_ACTION_RESUME);

	/* for the functionality provided by the zephyr PWM API, PWM mode 2 is sufficient,
	 * but some MTUs only support PWM mode 1
	 */
	if (cfg->dev_type == PWM_RX_TYPE_MTU1) {
		*cfg->tmdr = TMDR_MD_PWM_MODE_1;
	} else {
		*cfg->tmdr = TMDR_MD_PWM_MODE_2;
	}

	cfg->tcr->tpsc = cfg->prescaler;
	/* internal input clock default setting (falling edge) */
	cfg->tcr->ckeg = 0;

	switch (cfg->counter_clear_channel) {
	case 0:
		cfg->tcr->cclr = 1;
		counter_clear_channel_set = true;
		break;
	case 1:
		cfg->tcr->cclr = 2;
		counter_clear_channel_set = true;
		break;
	case 3:
		if (cfg->max_num_channels > 2) {
			cfg->tcr->cclr = 5;
			counter_clear_channel_set = true;
		}
		break;
	case 4:
		if (cfg->max_num_channels > 2) {
			cfg->tcr->cclr = 6;
			counter_clear_channel_set = true;
		}
		break;
	}

	if (cfg->synchronous) {
		const struct device **counter_clear_device;

		switch (cfg->dev_type) {
		case PWM_RX_TYPE_TPU:
			counter_clear_device = &synchronous_counter_clear_tpu;
			break;
		case PWM_RX_TYPE_MTU1:
		case PWM_RX_TYPE_MTU2:
			counter_clear_device = &synchronous_counter_clear_mtu;
			break;
		default:
			LOG_ERR("Unknown device type of %s", dev->name);
			return -EINVAL;
		}

		WRITE_BIT(*cfg->tsyr, cfg->bit_idx, true);
		if (counter_clear_channel_set) {
			if (*counter_clear_device != NULL) {
				LOG_WRN("both %s and %s are set to synchronous operation and have a"
					" counter clear channel set, which is redundant.",
					dev->name, (*counter_clear_device)->name);
				cfg->tcr->cclr = 3;
			} else {
				*counter_clear_device = dev;
			}
		} else {
			cfg->tcr->cclr = 3;
		}
	} else {
		WRITE_BIT(*cfg->tsyr, cfg->bit_idx, false);
	}

	return 0;
}

#define PARENT(id) DT_PARENT(DT_DRV_INST(id))

#define PWM_RX_INST_INIT(id) \
	static const struct pwm_rx_cfg pwm_rx_cfg##id = { \
		IF_ENABLED(DT_NODE_HAS_COMPAT(DT_DRV_INST(id), renesas_rx_tpu), \
			(.dev_type = PWM_RX_TYPE_TPU,)) \
		IF_ENABLED(DT_NODE_HAS_COMPAT(DT_DRV_INST(id), renesas_rx_mtu1), \
			(.dev_type = PWM_RX_TYPE_MTU1,)) \
		IF_ENABLED(DT_NODE_HAS_COMPAT(DT_DRV_INST(id), renesas_rx_mtu2), \
			(.dev_type = PWM_RX_TYPE_MTU2,)) \
		.tcr = (struct tcr *)DT_INST_REG_ADDR_BY_NAME(id, TCR), \
		.tmdr = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, TMDR), \
		.tior = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, TIOR), \
		.tgr = (uint16_t *)DT_INST_REG_ADDR_BY_NAME(id, TGR), \
		.tcnt = (uint16_t *)DT_INST_REG_ADDR_BY_NAME(id, TCNT), \
		.tstr = (uint8_t *)DT_REG_ADDR_BY_NAME(PARENT(id), TSTR), \
		.tsyr = (uint8_t *)DT_REG_ADDR_BY_NAME(PARENT(id), TSYR), \
		.bit_idx = DT_INST_PROP(id, bit_idx), \
		.max_num_channels = DT_INST_REG_SIZE_BY_NAME(id, TIOR) * 2, \
		.counter_clear_channel = DT_INST_PROP_OR(id, counter_clear_channel, -1), \
		.synchronous = DT_INST_PROP_OR(id, synchronous, false), \
		.prescaler = DT_INST_PROP(id, prescaler), \
		.clock_divider = DT_INST_PROP(id, clock_divider), \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(id, clock)), \
		.subsys = DT_INST_CLOCK_RX65N_SUB_SYSTEM(id, clock_subsystems), \
	}; \
	static struct pwm_rx_data pwm_rx_data##id; \
	static int pwm_rx_init##id(const struct device *dev) \
	{ \
		int ret = 0; \
		DT_INST_FOREACH_PROP_ELEM(id, gpios, RX_INIT_PIN) \
		if (ret == 0) { \
			ret = pwm_rx_init(dev); \
		} \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(id, toer_mask), ( \
			uint8_t *toer = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, TOER); \
			*toer |= DT_INST_PROP(id, toer_mask); \
		)) \
		return ret; \
	} \
	DEVICE_DT_INST_DEFINE(id, \
		&pwm_rx_init##id, \
		&pwm_rx_pm_control, \
		&pwm_rx_data##id, \
		&pwm_rx_cfg##id, \
		POST_KERNEL, \
		CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
		&pwm_rx_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_RX_INST_INIT)
