/*
 * Copyright (c) 2021-2022 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx600_adc

#include <errno.h>

#include <drivers/adc.h>
#include <device.h>
#include <kernel.h>
#include <init.h>
#include <soc.h>
#include <sys/util.h>
#include <drivers/clock_control.h>
#include <drivers/clock_control/clock_control_rx65n.h>
#include <drivers/interrupt_controller/rxv2_irq.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define REG_8_BIT(addr) *((uint8_t *)(addr))
#define REG_16_BIT(addr) *((uint16_t *)(addr))
#define REG_32_BIT(addr) *((uint32_t *)(addr))

struct adc_rx600_cfg {
	/* A/D Control Register (ADCSR) */
	volatile uint16_t *adcsr;
	/* A/D Channel Select Register A0 (ADANSA0) */
	volatile uint16_t *adansa0;
	/* A/D-Converted Value Addition/Average Count Select Register (ADADC) */
	volatile uint8_t *adadc;
	/* A/D Control Extended Register (ADCER) */
	volatile uint16_t *adcer;
	/* A/D Data Registers y (ADDRy) (y = 0 to 20) */
	volatile uint16_t *addr;
	/* A/D Comparison Function Control Register (ADCMPCR) */
	volatile uint16_t *adcmpcr;
	/* A/D-Converted Value Addition/Average Function Channel Select Register */
	volatile uint16_t *adads;
	/* Software Configurable Interrupt B Source Select Register n (SLIBRn) (n = 144 to 207) */
	volatile uint8_t *slibr;
	/* clock divider */
	const struct device *clock;
	/* clock subsystem */
	struct clock_control_rx65n_subsys subsys;
	/* number of channels provided by ADC unit */
	volatile uint8_t num_channels;
};

struct adc_rx600_data {
	struct adc_context ctx;
	const struct device *dev;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint32_t channels;
	uint8_t channel_id;
};

/**
 * @brief Checks if the buffer size is large enough
 * to hold the results of all requested samplings.
 *
 * @param sequence	pointer to a structure specifying requested
 * sequence of samplings.
 * @retval 0        on success
 * @retval -ENOMEM  If the provided buffer is too small to hold the results
 * of all requested samplings.
 */
static int check_buffer_size(const struct adc_sequence *sequence)
{
	size_t needed_buffer_size;
	size_t num_active_channels = __builtin_popcount(sequence->channels);

	needed_buffer_size = num_active_channels * sizeof(uint16_t);

	if (sequence->options) {
		needed_buffer_size *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed_buffer_size) {
		/* provided buffer is too small */
		return -ENOMEM;
	}

	return 0;
}

/**
 * @brief This required API function should set up ADC channel
 * which is currently done in adc_rx600_init().
 *
 * @param dev		pointer to an ADC device driver structure
 * @param sequence	pointer to a structure specifying channel
 * configuration
 *
 * @retval 0        on success
 */
static int adc_rx600_channel_setup(const struct device *dev,
				   const struct adc_channel_cfg *channel_cfg)
{
	return 0;
}

/**
 * @brief called when sampling of one or more channels is to be started
 *
 * @param ctx		pointer to an ADC context structure elements such as
 * adc sampling, sequence, kernel timer and semaphores
 *
 */
static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_rx600_data *data =
		CONTAINER_OF(ctx, struct adc_rx600_data, ctx);

	const struct adc_rx600_cfg *cfg = data->dev->config;

	/* A/D Control Register (ADCSR): stops A/D conversion
	 * A/D Conversion Start (ADST, b15 = 1: Starts A/D conversion process)
	 */
	WRITE_BIT(REG_16_BIT(cfg->adcsr), 15, true);

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;
}

/**
 * @brief called when the sample buffer pointer should be prepared
 * for writing of next sampling results
 *
 * @param ctx		pointer to an ADC context structure elements such as
 * adc sampling, sequence, kernel timer and semaphores
 * @param repeat	bool, indicates if the results should be written in the same place
 * as before (when true) or as consecutive ones (otherwise)
 *
 */
static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat)
{
	struct adc_rx600_data *data =
		CONTAINER_OF(ctx, struct adc_rx600_data, ctx);

	if (repeat) {
		data->buffer = data->repeat_buffer;
	}
}

static int start_read(const struct device *dev,
			const struct adc_sequence *sequence)
{
	struct adc_rx600_data *data = dev->data;

	int error = 0;

	const struct adc_rx600_cfg *cfg = dev->config;

	/* check if number of ADC channels to be sampled is correct */
	if (sequence->channels >= BIT(cfg->num_channels)) {
		return -EINVAL;
	}

	/* The default resolution of the RX MCU's ADC is 12 bit, but the resolution can be
	 * reduced to 8 or 10 bit (not implemented yet) or increased by oversampling (assuming
	 * the signal contains sufficient white noise). By summing up 2, 4 or 16 measurements,
	 * the resolution of the output value is increased from 12 bit (0..0xfff) to 13 bit
	 * (0..0x1ffe), 14 bit (0..0x3ffc) or 16 bit (0..0xfff0). Note that this might not be
	 * the actual resolution of the measurement, but it describes the range of the output
	 * values.
	 */
	switch (sequence->resolution) {
	case 8:
		/* 8 bit, no oversampling */
		*cfg->adads = 0;
		*cfg->adadc = 0;
		/* ADCER.ADPRC (b2,b1) = 0b10 */
		*cfg->adcer = (*cfg->adcer & 0xfff8) | 4;
		break;
	case 10:
		/* 10 bit, no oversampling */
		*cfg->adads = 0;
		*cfg->adadc = 0;
		/* ADCER.ADPRC (b2,b1) = 0b01 */
		*cfg->adcer = (*cfg->adcer & 0xfff8) | 2;
		break;
	case 12:
		/* default, no oversampling */
		*cfg->adads = 0;
		*cfg->adadc = 0;
		/* ADCER.ADPRC (b2,b1) = 0 */
		*cfg->adcer = *cfg->adcer & 0xfff8;
		break;
	case 13:
		/* sum of 2 measurements, increases the range to 0..0x1ffe (13 bit) */
		*cfg->adads = (uint16_t)sequence->channels;
		*cfg->adadc = 1;
		/* ADCER.ADPRC (b2,b1) = 0 */
		*cfg->adcer = *cfg->adcer & 0xfff8;
		break;
	case 14:
		/* sum of 4 measurements, increases the range to 0..0x3ffc (14 bit)*/
		*cfg->adads = (uint16_t)sequence->channels;
		*cfg->adadc = 3;
		/* ADCER.ADPRC (b2,b1) = 0 */
		*cfg->adcer = *cfg->adcer & 0xfff8;
		break;
	case 16:
		/* sum of 16 measurements, increases the range to 0..0xfff0 (16 bit)*/
		*cfg->adads = (uint16_t)sequence->channels;
		*cfg->adadc = 5;
		/* ADCER.ADPRC (b2,b1) = 0 */
		*cfg->adcer = *cfg->adcer & 0xfff8;
		break;
	default:
		/* no other resolution currently supported. */
		return -EINVAL;
	}

	error = check_buffer_size(sequence);
	if (error) {
		return error;
	}

	/* align ADC driver addresses from HW and OS */
	data->buffer = (uint16_t *)sequence->buffer;

	/* set channel via A/D Channel Select Register A0 */
	REG_16_BIT(cfg->adansa0) = (uint16_t)sequence->channels;

	/* A/D Control Register (ADCSR)
	 * b12 ADIE Scan End Interrupt Enable
	 * 1: Enables interrupt generation upon scan completion
	 */
	WRITE_BIT(REG_16_BIT(cfg->adcsr), 12, true);

	adc_context_start_read(&data->ctx, sequence);

	/* ADC completion is signalled via a semaphore */
	error = adc_context_wait_for_completion(&data->ctx);

	/* A/D Control Register (ADCSR): stops A/D conversion
	 * A/D Conversion Start (ADST, b15 = 0: Stops A/D conversion process)
	 */
	WRITE_BIT(REG_16_BIT(cfg->adcsr), 15, false);

	/* A/D Control Register (ADCSR)
	 * b12 ADIE Scan End Interrupt Enable
	 * 1: Enables interrupt generation upon scan completion
	 */
	WRITE_BIT(REG_16_BIT(cfg->adcsr), 12, false);

	return error;
}

/**
 * @brief Locks ADC device before reading ADC samples
 * with a subsequent unlock of the ADC device.
 *
 * @param dev		pointer to an ADC device driver structure
 * @param sequence	pointer to a structure specifying requested
 * sequence of samplings.
 *
 * @retval 0        on success
 * @retval -ENOMEM  If the provided buffer is too small to hold the results
 * of all requested samplings.
 * @retval -EBUSY   Returned without waiting for semaphore.
 * @retval -EAGAIN  Waiting period timed out,
 * or the semaphore was reset during the waiting period.
 */
static int adc_rx600_read(const struct device *dev,
			 const struct adc_sequence *sequence)
{
	struct adc_rx600_data *data = dev->data;
	int error = 0;

	adc_context_lock(&data->ctx, false, NULL);
	error = start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}

/**
 * @brief ISR for ADC read.
 */
static void adc_rx600_isr(const struct device *dev)
{
	struct adc_rx600_data *data = (struct adc_rx600_data *)dev->data;
	const struct adc_rx600_cfg *cfg = dev->config;

	uint32_t channels = data->channels;

	while (channels) {
		uint8_t channel = find_lsb_set(channels) - 1;

		/* handing S12AD A/D Data Register over to OS */
		*data->buffer++ = cfg->addr[channel];

		WRITE_BIT(channels, channel, 0);
	}

	adc_context_on_sampling_done(&data->ctx, dev);
}

static int adc_rx600_init(const struct device *dev)
{
	struct adc_rx600_data *data = dev->data;
	const struct adc_rx600_cfg *cfg = dev->config;

	data->dev = dev;

	/* Module Stop Control Register A (MSTPCRA)
	 * b17 MSTPA17 Module Stop Target module for S12AD
	 * 0: Release from the module-stop state
	 */
	clock_control_on(cfg->clock,
		(clock_control_subsys_t *)&cfg->subsys);

	/* A/D Control Register (ADCSR)
	 * b12 ADIE Scan End Interrupt Enable
	 * 1: Enables interrupt generation upon scan completion
	 */
	WRITE_BIT(REG_16_BIT(cfg->adcsr), 12, true);

	/* A/D Control Register (ADCSR): stops A/D conversion
	 * A/D Conversion Start (ADST, b15 = 0: Stops A/D conversion process)
	 * ADSSTRn register should be set while ADCSR.ADST bit is 0
	 */
	WRITE_BIT(REG_16_BIT(cfg->adcsr), 15, false);

	/* A/D Comparison Function Control Register (ADCMPCR) */
	REG_8_BIT(cfg->adcmpcr) = 0x0U;

	/* Software Configurable Interrupt B Source Select Register n (SLIBRn) */
	REG_8_BIT(cfg->slibr) = 0x40U;

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct adc_driver_api adc_rx600_api = {
	.channel_setup = adc_rx600_channel_setup,
	.read = adc_rx600_read,
};

#define CREATE_ADC_DEV(node_id) \
	static const struct adc_rx600_cfg adc##node_id##_rx600_cfg = { \
		.adcsr = (uint16_t *)DT_INST_REG_ADDR_BY_NAME(node_id, ADCSR), \
		.adansa0 = (uint16_t *)DT_INST_REG_ADDR_BY_NAME(node_id, ADANSA0), \
		.adadc = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(node_id, ADADC), \
		.adcer = (uint16_t *)DT_INST_REG_ADDR_BY_NAME(node_id, ADCER), \
		.addr = (uint16_t *)DT_INST_REG_ADDR_BY_NAME(node_id, ADDR), \
		.adcmpcr = (uint16_t *)DT_INST_REG_ADDR_BY_NAME(node_id, ADCMPCR), \
		.slibr = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(node_id, SLIBR), \
		.adads = (uint16_t *)DT_INST_REG_ADDR_BY_NAME(node_id, ADADS), \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(node_id, clock)), \
		.subsys = DT_INST_CLOCK_RX65N_SUB_SYSTEM(node_id, clock_subsystems), \
		.num_channels = (uint8_t) DT_INST_PROP(node_id, num_channels), \
	}; \
	\
	static struct adc_rx600_data adc##node_id##_rx600_data = { \
		ADC_CONTEXT_INIT_TIMER(adc##node_id##_rx600_data, ctx), \
		ADC_CONTEXT_INIT_LOCK(adc##node_id##_rx600_data, ctx), \
		ADC_CONTEXT_INIT_SYNC(adc##node_id##_rx600_data, ctx), \
	}; \
	\
	static int adc##node_id##_rx600_irq_init(const struct device *dev) \
	{ \
		int ret = 0; \
		IRQ_CONNECT( \
			DT_INST_IRQN(node_id), \
			DT_INST_IRQ(node_id, priority), \
			adc_rx600_isr, \
			DEVICE_DT_INST_GET(node_id), 0); \
		irq_enable(DT_INST_IRQN(node_id)); \
		DT_INST_FOREACH_PROP_ELEM(node_id, pinmuxs, RX_INIT_PIN) \
		if (ret == 0) { \
			ret = adc_rx600_init(dev); \
		} \
		return ret; \
	} \
	\
	DEVICE_DT_INST_DEFINE(node_id, \
		adc##node_id##_rx600_irq_init, \
		NULL, \
		&adc##node_id##_rx600_data, \
		&adc##node_id##_rx600_cfg, \
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
		&adc_rx600_api); \

DT_INST_FOREACH_STATUS_OKAY(CREATE_ADC_DEV)
