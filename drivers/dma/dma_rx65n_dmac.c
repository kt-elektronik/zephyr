/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief direct memory access (DMA) controller driver for Renesas RX65N
 *
 * @note Being mainly aimed at use with serial peripheral interfaces (SPI) at the
 *	moment, not all features of the RX65N DMAC have been implemented (yet).
 *	In addition to the normal mode currently used, the DMAC supports
 *	repeated transfer and block transfer modes. Also, the offset addition
 *	(only supported by channel 0) is not implemented.
 *	Similarly, several configuration and block configuration options are
 *	currently ignored.
 */

#include <devicetree.h>
#include <device.h>
#include <errno.h>
#include <soc.h>
#include <kernel.h>
#include <drivers/dma.h>
#include <drivers/clock_control.h>
#include <drivers/clock_control/clock_control_rx65n.h>

#include <logging/log.h>

#define DT_DRV_COMPAT renesas_rx65n_dmac
#define DMAC_NODE_ID DT_NODELABEL(dmac)
#define DMA_CHANNELS 8

#define DEV_CFG(cfg, dev) const struct dma_rx65n_dmac_cfg *cfg = \
			(const struct dma_rx65n_dmac_cfg *) dev->config;
#define DEV_DATA(data, dev) struct dma_rx65n_dmac_data *data = \
			(struct dma_rx65n_dmac_data *) dev->data;

#define REG_ADDR(base, channel) (base + 0x40 * channel)
#define REG8(base, channel) (*(uint8_t *)REG_ADDR(base, channel))
#define REG16(base, channel) (*(uint16_t *)REG_ADDR(base, channel))
#define REG32(base, channel) (*(uint32_t *)REG_ADDR(base, channel))

LOG_MODULE_REGISTER(dma_rx65n_dmac, CONFIG_DMA_LOG_LEVEL);

/**
 * @brief configuration structure for the DMAC device
 *
 * @note most of the register addresses actually represent the base address of a
 *	group of 8 registers for the 8 channels that repeat with an offset of
 *	0x40 bytes per channel
 */
struct dma_rx65n_dmac_cfg {
	/* DMA Source Address Register (x8) */
	uint32_t dmsar;
	/* DMA Destination Address Register (x8) */
	uint32_t dmdar;
	/* DMA Transfer Count Register (x8) */
	uint32_t dmcra;
	/* DMA Block Transfe Count Register (x8) */
	uint32_t dmcrb;
	/* DMA Transfer Mode Register (x8) */
	uint32_t dmtmd;
	/* DMA Interrupt Setting Register (x8) */
	uint32_t dmint;
	/* DMA Address Mode Register (x8) */
	uint32_t dmamd;
	/* DMA Offset Register (x8) */
	uint32_t dmofr;
	/* DMA Transfer Enable Register (x8) */
	uint32_t  dmcnt;
	/* DMA Software Start Register (x8) */
	uint32_t dmreq;
	/* DMA Status Register (x8) */
	uint32_t dmsts;
	/* DMA Request Source Flag Control Register (x8) */
	uint32_t dmcsl;
	/* DMA Module Start Register */
	volatile uint8_t *dmast;
	/* DMAC74 Interrupt Status Monitor Register */
	volatile uint8_t *dmist;
	/* DMAC Trigger Select Registers (to be handled as uint32_t dmrsr[8]) */
	volatile uint32_t *dmrsr;
};

/**
 * @brief data structure for a dma device driver
 */
struct dma_rx65n_dmac_data {
	/* dma context required by API */
	struct dma_context ctx;
	/* required by API */
	ATOMIC_DEFINE(channels_atomic, DMA_CHANNELS);
	/* active DMA configuration for each channel */
	struct dma_config *dma_config[DMA_CHANNELS];
	/* currently transferred block for each channel */
	struct dma_block_config *dma_block[DMA_CHANNELS];
};

/* The dma_config struct has 6 bits to define a dma_slot, which allows for 64
 * possible DMA slots (HW specific identifier which peripheral to use). On the
 * RX65N, this corresponds to which interrupt starts the DMAC, and while not all
 * 255 interrupts are valid choices, at least in theory there are more than 64
 * possibilities.
 * For easier use, the enum rx65n_dma_slots has been defined in
 * include\drivers\dma\dma_rx65n.h - it should be updated at the same time as
 * this array constant.
 */
static const uint32_t dma_slot_vectors[] = {
	38, /* RSPI0 SPRI0 */
	39, /* RSPI0 SPTI0 */
	40, /* RSPI1 SPRI1 */
	41, /* RSPI1 SPRI1 */
	42, /* QSPI SPRI */
	43, /* QSPI SPRI */
};

/**
 * @brief set up a DMA channel based on a block configuration
 *
 * @param dev		the DMA device driver
 * @param channel	the DMA channel to set up
 * @param block		the block configuration of the data block to transfer
 *
 * @returns		0 on success, negative error code on failure
 */
static int dma_rx65n_dmac_setup_block(const struct device *dev, uint32_t channel,
					struct dma_block_config *block)
{
	DEV_CFG(cfg, dev);

	if (block->block_size > 0xffff) {
		LOG_ERR("Block size (%u) too big (maximum 0xffff)",
				block->block_size);
		return -EINVAL;
	}

	REG32(cfg->dmsar, channel) = block->source_address;
	REG32(cfg->dmdar, channel) = block->dest_address;
	REG32(cfg->dmcra, channel) = block->block_size;

	switch (block->dest_addr_adj) {
	case DMA_ADDR_ADJ_INCREMENT:
		WRITE_BIT(REG16(cfg->dmamd, channel), 7, true);
		WRITE_BIT(REG16(cfg->dmamd, channel), 6, false);
		break;
	case DMA_ADDR_ADJ_DECREMENT:
		WRITE_BIT(REG16(cfg->dmamd, channel), 7, true);
		WRITE_BIT(REG16(cfg->dmamd, channel), 6, true);
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		WRITE_BIT(REG16(cfg->dmamd, channel), 7, false);
		WRITE_BIT(REG16(cfg->dmamd, channel), 6, false);
		break;
	default:
		return -EINVAL;
	}

	switch (block->source_addr_adj) {
	case DMA_ADDR_ADJ_INCREMENT:
		WRITE_BIT(REG16(cfg->dmamd, channel), 15, true);
		WRITE_BIT(REG16(cfg->dmamd, channel), 14, false);
		break;
	case DMA_ADDR_ADJ_DECREMENT:
		WRITE_BIT(REG16(cfg->dmamd, channel), 15, true);
		WRITE_BIT(REG16(cfg->dmamd, channel), 14, true);
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		WRITE_BIT(REG16(cfg->dmamd, channel), 15, false);
		WRITE_BIT(REG16(cfg->dmamd, channel), 14, false);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/**
 * @brief configure a dma channel (API function)
 *
 * @param dev		device driver
 * @param channel	DMA channel to configure
 * @param config	DMA configuration structure
 *
 * @return		0 on success, negative error codes else
 *
 * @note the following members of the dma_config structure are ignored for now:
 *		- channel_priority (priority is linked to channel number)
 *		- linked_channel
 *		- source_burst_length
 *		- dest_burst_length
 *		- error_callback_en
 */
static int dma_rx65n_dmac_configure(const struct device *dev, uint32_t channel,
					struct dma_config *config)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);

	if (channel > DMA_CHANNELS) {
		LOG_ERR("Invalid channel %u", channel);
		return -EINVAL;
	}

	if (config->channel_direction != MEMORY_TO_MEMORY &&
			config->dma_slot >= ARRAY_SIZE(dma_slot_vectors)) {
		LOG_ERR("Invalid dma slot %u", config->dma_slot);
		return -EINVAL;
	}

	if (config->source_data_size != config->dest_data_size) {
		LOG_ERR("unequal data sizes not supported");
		return -EINVAL;
	}

	if (config->source_burst_length != config->dest_burst_length) {
		LOG_ERR("uneqal burst lengths not supported");
		return -EINVAL;
	}

	/* disable channel during configuration (DMCNT.DTE = 0) */
	REG8(cfg->dmcnt, channel) = 0;
	/* reset software request in case it had been set */
	REG8(cfg->dmreq, channel) = 0;

	/* configure the channel_direction and (if appropriate) dma slot */
	switch (config->channel_direction) {
	case MEMORY_TO_PERIPHERAL:
	case PERIPHERAL_TO_MEMORY:
		/* set the activation interrupt based on the dma slot */
		cfg->dmrsr[channel] = dma_slot_vectors[config->dma_slot];
		/* if peripheral is involved, transfer is interrupt controlled */
		WRITE_BIT(REG16(cfg->dmtmd, channel), 0, true);
		break;
	case MEMORY_TO_MEMORY:
		/* for memory to memory, the transfer has to be started by SW */
		WRITE_BIT(REG16(cfg->dmtmd, channel), 0, false);
		break;
	default:
		LOG_ERR("Unsupported DMA channel direction %u", config->channel_direction);
		return -EINVAL;
	}

	/* it has been ensured before that source and destination have the same
	 * data width as this architecture does not support different widths.
	 * now the source width is used to set the transfer data sie select
	 * (DMTMD.SZ)
	 */
	switch (config->source_data_size) {
	case 1:
		/* 8 bit transfer data size */
		WRITE_BIT(REG16(cfg->dmtmd, channel), 8, false);
		WRITE_BIT(REG16(cfg->dmtmd, channel), 9, false);
		break;
	case 2:
		/* 16 bit transfer data size */
		WRITE_BIT(REG16(cfg->dmtmd, channel), 8, true);
		WRITE_BIT(REG16(cfg->dmtmd, channel), 9, false);
		break;
	case 4:
		/* 16 bit transfer data size */
		WRITE_BIT(REG16(cfg->dmtmd, channel), 8, false);
		WRITE_BIT(REG16(cfg->dmtmd, channel), 9, true);
		break;
	default:
		LOG_ERR("Invalid data size: %u", config->source_data_size);
		return -EINVAL;
	}

	/* enable the transfer end interrupt for this channel */
	WRITE_BIT(REG8(cfg->dmint, channel), 4, true);
	/* enable the transfer escape end interrupt */
	WRITE_BIT(REG8(cfg->dmint, channel), 3, true);

	data->dma_config[channel] = config;

	return 0;
}

/**
 * @brief reload buffer(s) for a DMA channel (API function)
 *
 * @param dev     Pointer to the device structure for the driver instance.
 * @param channel Numeric identification of the channel to configure
 *                selected channel
 * @param src     source address for the DMA transfer
 * @param dst     destination address for the DMA transfer
 * @param size    size of DMA transfer
 *
 * @retval 0 if successful.
 * @retval Negative errno code if failure.
 */
static int dma_rx65n_dmac_reload(const struct device *dev, uint32_t channel,
					uint32_t src, uint32_t dst, size_t size)
{
	DEV_CFG(cfg, dev);

	if (channel > DMA_CHANNELS) {
		LOG_ERR("Invalid channel %u", channel);
		return -EINVAL;
	}

	REG32(cfg->dmsar, channel) = src;
	REG32(cfg->dmdar, channel) = dst;

	return 0;
}

/**
 * @brief enable DMA channel and start transfer (API function)
 *
 * @param dev     Pointer to the device structure for the driver instance.
 * @param channel Numeric identification of the channel where the transfer will
 *                be processed
 *
 * @retval 0 if successful.
 * @retval Negative errno code if failure.
 */
static int dma_rx65n_dmac_start(const struct device *dev, uint32_t channel)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);

	if (channel > DMA_CHANNELS) {
		LOG_ERR("Invalid channel %u", channel);
		return -EINVAL;
	}

	if (data->dma_config[channel] == NULL) {
		LOG_ERR("Channel %u not configured", channel);
		return -EINVAL;
	}

	data->dma_block[channel] = data->dma_config[channel]->head_block;

	dma_rx65n_dmac_setup_block(dev, channel, data->dma_block[channel]);

	REG8(cfg->dmcnt, channel) = 1;

	if (data->dma_config[channel]->channel_direction == MEMORY_TO_MEMORY) {
		/* Enable DMA (DMCNT.DTE = 1) */
		REG8(cfg->dmreq, channel) = BIT(4) | BIT(0);
	}

	return 0;
}

/**
 * @brief disable DMA transfer and disable channel (API function)
 *
 * @param dev     Pointer to the device structure for the driver instance.
 * @param channel Numeric identification of the channel where the transfer was
 *                being processed
 *
 * @retval 0 if successful.
 * @retval Negative errno code if failure.
 */
static int dma_rx65n_dmac_stop(const struct device *dev, uint32_t channel)
{
	DEV_CFG(cfg, dev);

	if (channel > DMA_CHANNELS) {
		LOG_ERR("Invalid channel %u", channel);
		return -EINVAL;
	}

	/* Disable DMA (DMCNT.DTE = 0) */
	REG8(cfg->dmcnt, channel) = 0;

	return 0;
}

/**
 * @brief get status of a DMA channel (API function)
 *
 * @param dev     Pointer to the device structure for the driver instance.
 * @param channel Numeric identification of the channel where the transfer was
 *                being processed
 * @param stat   a non-NULL dma_status object for storing DMA status
 *
 * @retval 0 if successful.
 * @retval Negative errno code if failure.
 */
static int dma_rx65n_dmac_status(const struct device *dev, uint32_t channel,
				  struct dma_status *status)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);

	if (channel > DMA_CHANNELS) {
		LOG_ERR("Invalid channel %u", channel);
		return -EINVAL;
	}

	status->busy = (REG8(cfg->dmsts, channel) & BIT(7)) != 0;
	status->dir = data->dma_config[channel]->channel_direction;
	status->pending_length = REG32(cfg->dmcra, channel);

	return 0;
}

/**
 * @brief DMA channel filter (API function)
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param channel channel number
 * @param filter_param filter attribute
 *
 * @retval true if channel fulfills the filter attribute
 * @retval false if channel does not fulfull the filter attribute
 */
static bool dma_rx65n_dmac_filter(const struct device *dev, int channel,
					void *filter_param)
{
	if (channel > DMA_CHANNELS) {
		return false;
	}

	return true;
}

/**
 * @brief interrupt handler for all DMAC interrupts
 *
 * @param dev		DMAC device driver
 * @param channel	channel for which the interrupt was triggered
 */
static void dma_rx65n_dmac_isr(const struct device *dev, int channel)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);

	if (data->dma_block[channel]->next_block == NULL) {
		/* all blocks have been transferred */
		if (data->dma_config[channel]->dma_callback) {
			data->dma_config[channel]->dma_callback(dev,
				data->dma_config[channel]->user_data, channel, 0);
		}
		return;
	}
	/* there is still data to transfer */
	if (data->dma_config[channel]->complete_callback_en &&
		data->dma_config[channel]->dma_callback) {
		/* configured to callback after each block */
		data->dma_config[channel]->dma_callback(dev,
			data->dma_config[channel]->user_data, channel, 0);
	}

	/* transfer the next block */
	data->dma_block[channel] = data->dma_block[channel]->next_block;

	if (data->dma_config[channel]->channel_direction != MEMORY_TO_MEMORY) {
		/* for peripheral destination or source the interrupt that would
		 * have triggered the next transfer has just been generated and
		 * will not be treated by the dmac. Hence the first byte of the
		 * next buffer has to be transferred here to cause the next
		 * interrupt to resume regular DMA operation.
		 */
		switch (data->dma_config[channel]->source_data_size) {
		case 1:
			*(uint8_t *)data->dma_block[channel]->dest_address =
				*(uint8_t *)data->dma_block[channel]->source_address;
			break;
		case 2:
			*(uint16_t *)data->dma_block[channel]->dest_address =
				*(uint16_t *)data->dma_block[channel]->source_address;
			break;
		case 4:
			*(uint32_t *)data->dma_block[channel]->dest_address =
				*(uint32_t *)data->dma_block[channel]->source_address;
			break;
		}
		data->dma_block[channel]->block_size -=
			data->dma_config[channel]->source_data_size;
		switch (data->dma_block[channel]->dest_addr_adj) {
		case DMA_ADDR_ADJ_INCREMENT:
			data->dma_block[channel]->dest_address +=
				data->dma_config[channel]->source_data_size;
			break;
		case DMA_ADDR_ADJ_DECREMENT:
			data->dma_block[channel]->dest_address -=
				data->dma_config[channel]->source_data_size;
			break;
		}
		switch (data->dma_block[channel]->source_addr_adj) {
		case DMA_ADDR_ADJ_INCREMENT:
			data->dma_block[channel]->source_address +=
				data->dma_config[channel]->source_data_size;
			break;
		case DMA_ADDR_ADJ_DECREMENT:
			data->dma_block[channel]->source_address -=
				data->dma_config[channel]->source_data_size;
			break;
		}
		if (data->dma_block[channel]->block_size == 0) {
			data->dma_block[channel] = data->dma_block[channel]->next_block;
			if (data->dma_block[channel] == NULL) {
				/* end of transfer treatment */
				dma_rx65n_dmac_isr(dev, channel);
			}
		}
	}

	dma_rx65n_dmac_setup_block(dev, channel, data->dma_block[channel]);

	REG8(cfg->dmcnt, channel) = 1;

	/* restart transfer for the next block */
	if (data->dma_config[channel]->channel_direction == MEMORY_TO_MEMORY) {
		/* Enable DMA (DMCNT.DTE = 1) */
		REG8(cfg->dmreq, channel) = BIT(4) | BIT(0);
	} else {
	}
}

/**
 * @brief interrupt handler for channels 0
 *
 * @param arg void pointer to the DMAC device structure
 */
static void dma_rx65n_dmac_dmac0i_isr(void *arg)
{
	dma_rx65n_dmac_isr((struct device *)arg, 0);
}

/**
 * @brief interrupt handler for channel 1
 *
 * @param arg void pointer to the DMAC device structure
 */
static void dma_rx65n_dmac_dmac1i_isr(void *arg)
{
	dma_rx65n_dmac_isr((struct device *)arg, 1);
}

/**
 * @brief interrupt handler for channel 2
 *
 * @param arg void pointer to the DMAC device structure
 */
static void dma_rx65n_dmac_dmac2i_isr(void *arg)
{
	dma_rx65n_dmac_isr((struct device *)arg, 2);
}

/**
 * @brief interrupt handler for channel 3
 *
 * @param arg void pointer to the DMAC device structure
 */
static void dma_rx65n_dmac_dmac3i_isr(void *arg)
{
	dma_rx65n_dmac_isr((struct device *)arg, 3);
}

/**
 * @brief interrupt handler for channels 4-7
 *
 * @param arg void pointer to the DMAC device structure
 *
 * @note channels 4-7 all trigger the same interrupt, so this function has to
 *	distinguish which channel triggered the interrupt by checing the DMIST
 *	register.
 */
static void dma_rx65n_dmac_dmac74i_isr(void *arg)
{
	const struct device *dev = (struct device *) arg;

	DEV_CFG(cfg, dev);

	if (*cfg->dmist & BIT(4)) {
		dma_rx65n_dmac_isr((struct device *)arg, 4);
	}
	if (*cfg->dmist & BIT(5)) {
		dma_rx65n_dmac_isr((struct device *)arg, 5);
	}
	if (*cfg->dmist & BIT(6)) {
		dma_rx65n_dmac_isr((struct device *)arg, 6);
	}
	if (*cfg->dmist & BIT(7)) {
		dma_rx65n_dmac_isr((struct device *)arg, 7);
	}
}

/**
 * @brief initialize the DMAC driver
 *
 * @param dev Pointer to the device structure
 *
 * @retval 0 if successful
 * @retval Negative error code if failure
 */
static int dma_rx65n_dmac_init(const struct device *dev)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);

	IRQ_CONNECT(DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac0i, irq),
		DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac0i, priority),
		dma_rx65n_dmac_dmac0i_isr,
		DEVICE_DT_GET(DMAC_NODE_ID),
		DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac0i, flags));
	irq_enable(DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac0i, irq));

	IRQ_CONNECT(DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac1i, irq),
		DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac1i, priority),
		dma_rx65n_dmac_dmac1i_isr,
		DEVICE_DT_GET(DMAC_NODE_ID),
		DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac1i, flags));
	irq_enable(DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac1i, irq));

	IRQ_CONNECT(DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac2i, irq),
		DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac2i, priority),
		dma_rx65n_dmac_dmac2i_isr,
		DEVICE_DT_GET(DMAC_NODE_ID),
		DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac2i, flags));
	irq_enable(DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac2i, irq));

	IRQ_CONNECT(DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac3i, irq),
		DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac3i, priority),
		dma_rx65n_dmac_dmac3i_isr,
		DEVICE_DT_GET(DMAC_NODE_ID),
		DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac3i, flags));
	irq_enable(DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac3i, irq));

	IRQ_CONNECT(DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac74i, irq),
		DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac74i, priority),
		dma_rx65n_dmac_dmac74i_isr,
		DEVICE_DT_GET(DMAC_NODE_ID),
		DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac74i, flags));
	irq_enable(DT_IRQ_BY_NAME(DMAC_NODE_ID, dmac74i, irq));

	data->ctx.magic = DMA_MAGIC;
	data->ctx.dma_channels = DMA_CHANNELS;
	data->ctx.atomic = data->channels_atomic;

	struct clock_control_rx65n_subsys subsys =
			DT_CLOCK_RX65N_SUB_SYSTEM(DMAC_NODE_ID, clock_subsystems);

	clock_control_on(DEVICE_DT_GET(DT_PHANDLE(DMAC_NODE_ID, clock)),
						(clock_control_subsys_t)&subsys);

	/* enable DMAC */
	*cfg->dmast = 1;

	return 0;
}

static const struct dma_driver_api dma_rx65n_dmac_api = {
	.reload = dma_rx65n_dmac_reload,
	.config = dma_rx65n_dmac_configure,
	.start = dma_rx65n_dmac_start,
	.stop = dma_rx65n_dmac_stop,
	.get_status = dma_rx65n_dmac_status,
	.chan_filter = dma_rx65n_dmac_filter,
};

static const struct dma_rx65n_dmac_cfg dma_rx65n_dmac_cfg = {
	.dmsar = DT_REG_ADDR_BY_NAME(DMAC_NODE_ID, DMSAR),
	.dmdar = DT_REG_ADDR_BY_NAME(DMAC_NODE_ID, DMDAR),
	.dmcra = DT_REG_ADDR_BY_NAME(DMAC_NODE_ID, DMCRA),
	.dmcrb = DT_REG_ADDR_BY_NAME(DMAC_NODE_ID, DMCRB),
	.dmtmd = DT_REG_ADDR_BY_NAME(DMAC_NODE_ID, DMTMD),
	.dmint = DT_REG_ADDR_BY_NAME(DMAC_NODE_ID, DMINT),
	.dmamd = DT_REG_ADDR_BY_NAME(DMAC_NODE_ID, DMAMD),
	.dmofr = DT_REG_ADDR_BY_NAME(DMAC_NODE_ID, DMOFR),
	.dmcnt = DT_REG_ADDR_BY_NAME(DMAC_NODE_ID, DMCNT),
	.dmreq = DT_REG_ADDR_BY_NAME(DMAC_NODE_ID, DMREQ),
	.dmsts = DT_REG_ADDR_BY_NAME(DMAC_NODE_ID, DMSTS),
	.dmcsl = DT_REG_ADDR_BY_NAME(DMAC_NODE_ID, DMCSL),
	.dmast = (uint8_t *)DT_REG_ADDR_BY_NAME(DMAC_NODE_ID, DMAST),
	.dmist = (uint8_t *)DT_REG_ADDR_BY_NAME(DMAC_NODE_ID, DMIST),
	.dmrsr = (uint32_t *)DT_REG_ADDR_BY_NAME(DMAC_NODE_ID, DMRSR),
};

static struct dma_rx65n_dmac_data dma_rx65n_dmac_data;

DEVICE_DT_DEFINE(DMAC_NODE_ID, dma_rx65n_dmac_init, NULL, &dma_rx65n_dmac_data,
	&dma_rx65n_dmac_cfg, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
	&dma_rx65n_dmac_api);
