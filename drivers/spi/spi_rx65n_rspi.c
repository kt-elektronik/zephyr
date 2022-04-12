/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief device driver for the RSPI modules of the Renesas RX65N MCU
 *
 * @note currently, only part of the RSPI features are supported, e.g. the
 *	device can only operate in (single) master mode and no use is made of
 *	the sequence feature of the RSPI.
 */

#define DT_DRV_COMPAT renesas_rx65n_rspi

#include <soc.h>
#include <errno.h>
#include <device.h>
#include <kernel.h>
#include <irq.h>
#include <drivers/gpio.h>
#include <drivers/pinmux.h>
#include <drivers/spi.h>
#include <drivers/clock_control.h>
#include <drivers/clock_control/clock_control_rx65n.h>
#if CONFIG_DMA
#include <drivers/dma.h>
#endif

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_rx65n_rspi);

#include "spi_context.h"

#define DEV_CFG(cfg, dev) const struct rspi_rx65n_cfg *cfg = \
			(const struct rspi_rx65n_cfg *) dev->config;
#define DEV_DATA(data, dev) struct rspi_rx65n_data *data = \
			(struct rspi_rx65n_data *) dev->data;

struct rspi_rx65n_spcmd {
	uint16_t cpha : 1;
	uint16_t cpol : 1;
	uint16_t brdv : 2;
	uint16_t ssla : 3;
	uint16_t sslkp : 1;
	uint16_t spb : 4;
	uint16_t lsbf : 1;
	uint16_t spnden : 1;
	uint16_t slnden : 1;
	uint16_t sckden : 1;
};

struct rspi_rx65n_cfg {
	/* pointer to RSPI control register */
	volatile uint8_t *spcr;
	/* pointer to slave select polarity register */
	volatile uint8_t *sslp;
	/* pointer to RSPI pin control register */
	volatile uint8_t *sppcr;
	/* pointer to RSPI status register */
	volatile uint8_t *spsr;
	/* pointer to rspi data register */
	volatile uint8_t *spdr;
	/* pointer to RSPI sequence control register */
	volatile uint8_t *spscr;
	/* pointer to RSPI sequence status register */
	volatile uint8_t *spssr;
	/* pointer to RSPI bitrate register */
	volatile uint8_t *spbr;
	/* pointer to RSPI data control register */
	volatile uint8_t *spdcr;
	/* pointer to RSPI clock delay register */
	volatile uint8_t *spckd;
	/* pointer to RSPI slave select negation delay register */
	volatile uint8_t *sslnd;
	/* pointer to RSPI next-access delay register*/
	volatile uint8_t *spnd;
	/* pointer to RSPI control register 2 */
	volatile uint8_t *spcr2;
	/* pointer to first RSPI command register (of 8 sequential command registers) */
	volatile struct rspi_rx65n_spcmd *spcmd;
	/* pointer to RSPI data control register 2 */
	volatile uint8_t *spdcr2;
	/* DMA controller to use for data reception */
	const struct device *rx_dma;
	/* DMA slot for data reception */
	const uint8_t rx_dma_slot;
	/* DMA controller to use for data transmission */
	const struct device *tx_dma;
	/* DMA slot for data transmission */
	const uint8_t tx_dma_slot;
	/* clock responsible for the rspi */
	const struct device *clock;
	/* clock subsystem */
	const struct clock_control_rx65n_subsys subsys;
};

struct rspi_rx65n_data {
	uint32_t clk_src_freq_hz;
	struct spi_context ctx;
#if CONFIG_DMA
	int tx_dma_channel;
	struct dma_config tx_dma_config;
	struct dma_block_config tx_dma_block;
	size_t add_tx_dummy;
	int rx_dma_channel;
	struct dma_config rx_dma_config;
	struct dma_block_config rx_dma_block;
	size_t add_rx_dummy;
#endif
};

const static uint32_t tx_dummy;
static uint32_t rx_dummy;

/**
 * @brief find the best bitrate configuration (spbr and brdv)
 *
 * @param f		frequency of the clock source in Hz
 * @param target	target bitrate in bps
 * @param spbr		pointer to variable to store best spbr value (0..255)
 * @param brdv		pointer to variable to store best brdv value (0..3)
 *
 * @return		difference between target and possible bitrate in % of
 *			target * 10 (1000 if no solution was found)
 * @note		The bitrate of the channel is calculated as
 *
 *				f / (2 * (spbr + 1) * 2^brdv)
 *
 *			the function searches the combination of spbr and brdv
 *			that results in the closest bitrate to the target that
 *			is smaller than or equal to the target
 */
static uint16_t rspi_rx65n_find_bitrate(uint32_t f, uint32_t target,
					uint16_t *spbr, uint8_t *brdv)
{
	uint16_t best_err = 1000;
	uint16_t err;
	uint32_t br;
	/* the smallest value of spbr which can result in a bitrate smaller than
	 * the target bitrate (using brdv = 3). No value of spbr smaller than
	 * this will result in a valid bitrate (i.e. smaller than the target)
	 */
	uint32_t min_spbr = f/target/16;
	/* the smallest value of spbr which can only result in bitrates smaller
	 * than the target (using brdv = 0). Any value of spbr larger than this
	 * will result in bitrates that are smaller than the bitrate resulting
	 * from max_spbr and brdv = 0 which is valid (i.e. smaller than the
	 * target) and hence would be preferred over any bitrate resuling from
	 * a value of spbr greater than max_spbr
	 */
	uint32_t max_spbr = f/target/2;

	max_spbr = MIN(0xff, max_spbr);
	*spbr = 0xff;
	*brdv = 3;

	for (uint16_t s = min_spbr; s <= max_spbr && best_err != 0; s++) {
		for (uint8_t b = 0; b < 4; b++) {
			br = f/(2 * (s + 1) * (1<<b));
			if (br <= target) {
				err = 1000 - (br / (target / 1000));
				if (err < best_err) {
					best_err = err;
					*spbr = s;
					*brdv = b;
				}
				/* increasing brdv will not improve the result */
				break;
			}
		}
	}
	return best_err;
}

/**
 * @brief configure the RSPI for a transception
 *
 * @param dev		RSPI device structure
 * @param spi_cfg	transception parameters
 *
 * @return		0 on success or negative error code
 */
static int rspi_rx65n_setup(const struct device *dev, const struct spi_config *spi_cfg)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);

	uint16_t spbr;
	uint8_t brdv;

	if (spi_context_configured(&data->ctx, spi_cfg)) {
		return 0;
	}

	data->ctx.config = spi_cfg;

	if (rspi_rx65n_find_bitrate(data->clk_src_freq_hz, spi_cfg->frequency,
					&spbr, &brdv) == 1000) {
		LOG_ERR("Bitrate %u impossible for %s running at clock frequency %u",
				spi_cfg->frequency,
				dev->name,
				data->clk_src_freq_hz);
		/* it was not possible to set the bitrate to a valid value */
		return -EINVAL;
	}

	/* disable the SPI channel to change the bitrate setting register */
	WRITE_BIT(*cfg->spcr, 6, false);
	*cfg->spbr = spbr;

	cfg->spcmd[0].brdv = brdv;
	cfg->spcmd[0].ssla = spi_cfg->slave;
	cfg->spcmd[0].cpol = ((spi_cfg->operation & SPI_MODE_CPOL) != 0);
	cfg->spcmd[0].cpha = ((spi_cfg->operation & SPI_MODE_CPHA) != 0);
	cfg->spcmd[0].lsbf = ((spi_cfg->operation & SPI_TRANSFER_LSB) != 0);
	cfg->spcmd[0].sslkp = ((spi_cfg->operation & SPI_HOLD_ON_CS) != 0);

	uint8_t word_size = SPI_WORD_SIZE_GET(spi_cfg->operation);

	if (word_size < 8) {
		return -EINVAL;
	} else if (word_size <= 16) {
		/* 8-16 bit */
		cfg->spcmd[0].spb = word_size - 1;
	} else if (word_size == 20) {
		cfg->spcmd[0].spb = 0;
	} else if (word_size == 24) {
		cfg->spcmd[0].spb = 1;
	} else if (word_size == 32) {
		cfg->spcmd[0].spb = 2;
	} else {
		LOG_ERR("Invalid word size (%u) for %s", word_size, dev->name);
		return -EINVAL;
	}
	return 0;
}

#if CONFIG_DMA
void rspi_rx65n_cancel_dma_transception(const struct device *dev, int status)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);

	dma_stop(cfg->tx_dma, data->tx_dma_channel);
	dma_stop(cfg->rx_dma, data->rx_dma_channel);

	dma_release_channel(cfg->tx_dma, data->tx_dma_channel);
	dma_release_channel(cfg->rx_dma, data->rx_dma_channel);

	spi_context_cs_control(&data->ctx, false);
	spi_context_complete(&data->ctx, status);

	/* disabled SPI channel and interrupts */
	WRITE_BIT(*cfg->spcr, 5, false);
	WRITE_BIT(*cfg->spcr, 6, false);
	WRITE_BIT(*cfg->spcr, 7, false);
}

/**
 * @brief callback function on completion of the transmission DMA channel
 *
 * @param dev		device structure of the transmission DMA
 * @param user_data	device structure of the RSPI device (as void pointer)
 * @param channel	DMA channel number
 * @param status	status of the DMA channel
 *
 * @note if the transception is finished, the tx DMA channel will be released.
 *       Most of the handling of the completion will happen in the rx callback
 *       function.
 *       Otherwise the next transmission block will be set up for DMA and
 *       started
 */
void rspi_rx65n_dma_tx_callback(const struct device *dev, void *user_data,
			uint32_t channel, int status)
{
	const struct device *spi_dev = (struct device *)user_data;

	DEV_CFG(cfg, spi_dev);
	DEV_DATA(data, spi_dev);
	struct spi_context *ctx = &data->ctx;

	/* clear SPE and SPTIE so transmission can be triggered by setting them
	 * at the end of this function
	 */
	WRITE_BIT(*cfg->spcr, 5, false);
	WRITE_BIT(*cfg->spcr, 6, false);

	if (status != 0) {
		LOG_ERR("tx-DMA %s channel %u returned error %d to %s. Cancelling transception.",
			dev->name, channel, status, spi_dev->name);
		/* an error has occurred - cancel the transception */
		rspi_rx65n_cancel_dma_transception(spi_dev, status);
		return;
	}

	if (ctx->sync_status != 0) {
		return;
	}

	if (ctx->tx_len == 0 && data->add_tx_dummy == 0) {
		dma_release_channel(cfg->tx_dma, data->tx_dma_channel);
		/* all finished. rspi_rx65n_dma_tx_callback() will trigger the
		 * transmission complete signal once the last byte has
		 * been copied to the receive buffer. Just to be sure, re-enable
		 * the SPE to make sure the last byte is received
		 */
		WRITE_BIT(*cfg->spcr, 6, true);
		return;
	}

	memset(&data->tx_dma_block, 0, sizeof(struct dma_block_config));
	data->tx_dma_block.dest_address = (uint32_t)cfg->spdr;
	data->tx_dma_block.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	if (ctx->tx_buf == NULL) {
		data->tx_dma_block.source_address = (uint32_t)&tx_dummy;
		data->tx_dma_block.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		data->tx_dma_block.source_address = (uint32_t)ctx->tx_buf;
		data->tx_dma_block.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}

	if (ctx->tx_len == 0) {
		data->tx_dma_block.block_size = data->add_tx_dummy;
		data->add_tx_dummy = 0;
	} else {
		data->tx_dma_block.block_size = ctx->tx_len;
		spi_context_update_tx(ctx, 1, ctx->tx_len);
	}

	dma_start(cfg->tx_dma, data->tx_dma_channel);

	/* To generate a transmission buffer empty interrupt request (which
	 * starts data transmission by the dma), either both SPE (bit 6 of SPCR)
	 * and SPTIE (bit 5 of SPCR) have to be set at the same time or SPE has
	 * to be set after SPTIE.
	 */
	WRITE_BIT(*cfg->spcr, 5, true);
	WRITE_BIT(*cfg->spcr, 6, true);
}

/**
 * @brief callback function on completion of the reception DMA channel
 *
 * @param dev		device structure of the reception DMA channel
 * @param user_data	device structure of the RSPI device (as void pointer)
 * @param channel	DMA channel number
 * @param status	status of the DMA channel
 *
 * @note if the transception is finished, the rx DMA channel will be released,
 *       the RSPI channel will be disabled and the completion signal of the spi
 *       context will be triggered.
 *       Otherwise the next reception block will be set up for DMA and
 *       started
 */
void rspi_rx65n_dma_rx_callback(const struct device *dev, void *user_data,
			uint32_t channel, int status)
{
	const struct device *spi_dev = (struct device *)user_data;

	DEV_CFG(cfg, spi_dev);
	DEV_DATA(data, spi_dev);
	struct spi_context *ctx = &data->ctx;

	if (status != 0) {
		LOG_ERR("rx-DMA %s channel %u returned error %d to %s. Cancelling transception.",
			dev->name, channel, status, spi_dev->name);
		/* an error has occurred - cancel the transception */
		rspi_rx65n_cancel_dma_transception(spi_dev, status);
		return;
	}

	if (ctx->sync_status != 0) {
		return;
	}

	if (ctx->rx_len == 0 && data->add_rx_dummy == 0) {
		dma_release_channel(cfg->rx_dma, data->rx_dma_channel);

		/* disable channel (SPCR.SPE) */
		WRITE_BIT(*cfg->spcr, 6, false);
		/* disable receive buffer full interrupts (SPCR.SPRIE) */
		WRITE_BIT(*cfg->spcr, 7, false);

		spi_context_cs_control(&data->ctx, false);
		spi_context_complete(&data->ctx, 0);
		return;
	}

	memset(&data->rx_dma_block, 0, sizeof(struct dma_block_config));
	data->rx_dma_block.source_address = (uint32_t)cfg->spdr;
	data->rx_dma_block.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	if (ctx->rx_buf == NULL) {
		data->rx_dma_block.dest_address = (uint32_t)&rx_dummy;
		data->rx_dma_block.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		data->rx_dma_block.dest_address = (uint32_t)ctx->rx_buf;
		data->rx_dma_block.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}

	if (ctx->rx_len == 0) {
		/* the tx buffer was longer than the rx buffer, so add a dummy
		 * section to receive the responses to the remaining tx bytes
		 */
		data->rx_dma_block.block_size = data->add_rx_dummy;
		data->add_rx_dummy = 0;
	} else {
		data->rx_dma_block.block_size = ctx->rx_len;
		spi_context_update_rx(ctx, 1, ctx->rx_len);
	}

	dma_start(cfg->rx_dma, data->rx_dma_channel);

	/* enable receive buffer full interrupts (SPCR.SPRIE) */
	WRITE_BIT(*cfg->spcr, 7, true);
}

/**
 * @brief start the transception using DMA
 *
 * @param dev	RSPI device structure
 *
 * @returns	0 on success or negative error code
 */
static int rspi_rx65n_transceive_dma(const struct device *dev)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);
	struct spi_context *ctx = &data->ctx;

	data->tx_dma_channel = dma_request_channel(cfg->tx_dma, NULL);
	if (data->tx_dma_channel < 0) {
		LOG_INF("No available tx-DMA channels on %s.", cfg->tx_dma->name);
		return -EBUSY;
	}
	data->rx_dma_channel = dma_request_channel(cfg->rx_dma, NULL);
	if (data->rx_dma_channel < 0) {
		LOG_INF("No available rx-DMA channels on %s.", cfg->tx_dma->name);
		dma_release_channel(cfg->tx_dma, data->tx_dma_channel);
		return -EBUSY;
	}

	data->rx_dma_config.dma_slot = cfg->rx_dma_slot;
	data->rx_dma_config.channel_direction = PERIPHERAL_TO_MEMORY;
	data->rx_dma_config.source_data_size = 1;
	data->rx_dma_config.dest_data_size = 1;
	data->rx_dma_config.block_count = 1;
	data->rx_dma_config.dma_callback = rspi_rx65n_dma_rx_callback;
	data->rx_dma_config.user_data = (void *)dev;
	data->rx_dma_config.head_block = &data->rx_dma_block;

	data->tx_dma_config.dma_slot = cfg->tx_dma_slot;
	data->tx_dma_config.channel_direction = MEMORY_TO_PERIPHERAL;
	data->tx_dma_config.source_data_size = 1;
	data->tx_dma_config.dest_data_size = 1;
	data->tx_dma_config.block_count = 1;
	data->tx_dma_config.dma_callback = rspi_rx65n_dma_tx_callback;
	data->tx_dma_config.user_data = (void *)dev;
	data->tx_dma_config.head_block = &data->tx_dma_block;

	dma_config(cfg->rx_dma, data->rx_dma_channel, &data->rx_dma_config);
	dma_config(cfg->tx_dma, data->tx_dma_channel, &data->tx_dma_config);

	/* compare the total size of transmission and reception blocks. Since
	 * SPI only can perform transception, the other channel will have to
	 * add a block of appropriate length to allow for the whole transception
	 * to finish
	 */
	size_t tx_len = spi_context_total_tx_len(ctx);
	size_t rx_len = spi_context_total_rx_len(ctx);

	if (rx_len > tx_len) {
		data->add_tx_dummy = rx_len - tx_len;
		data->add_rx_dummy = 0;
	} else {
		data->add_rx_dummy = tx_len - rx_len;
		data->add_tx_dummy = 0;
	}

	/* directly call the callback functions to load the first buffers and
	 * start transmission
	 */
	rspi_rx65n_dma_rx_callback(cfg->rx_dma, (void *)dev, data->rx_dma_channel, 0);
	rspi_rx65n_dma_tx_callback(cfg->tx_dma, (void *)dev, data->tx_dma_channel, 0);

	return 0;
}
#endif

/**
 * @brief transceive the next data packet
 *
 * @param dev	RSPI device structure
 *
 * @return	0 on success or negative error code
 */
static int rspi_rx65n_transceive_next_packet(const struct device *dev)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);
	struct spi_context *ctx = &data->ctx;
	int ret = 0;

	if (ctx->tx_len == 0 && ctx->rx_len == 0) {
		/* disable channel (SPCR.SPE) */
		WRITE_BIT(*cfg->spcr, 6, false);
		/* disable transmit buffer empty interrupt (SPCR.SPTIE) */
		WRITE_BIT(*cfg->spcr, 5, false);
		/* disable receive buffer full interrupts (SPCR.SPRIE) */
		WRITE_BIT(*cfg->spcr, 7, false);

		spi_context_cs_control(&data->ctx, false);
		spi_context_complete(&data->ctx, 0);
	}

	uint32_t key = irq_lock();

	/* if no DMA is available, transmit bytewise - the reception buffer will
	 * be filled in rspi_rx65n_spri_isr which will be triggered after
	 * receiving the answer to this transmission
	 */
	if (data->ctx.tx_buf == NULL) {
		*cfg->spdr = tx_dummy;
	} else {
		*cfg->spdr = *data->ctx.tx_buf;
	}
	spi_context_update_tx(&data->ctx, 1, 1);

	irq_unlock(key);

	return ret;
}

/**
 * @brief start the transception on a RSPI channel
 *
 * @param dev		RSPI device structure
 * @param spi_cfg	spi connection parameters
 * @param tx_bufs	transmission buffers
 * @param rx_bufs	reception buffers
 * @param asynchronous	is this an asynchronous call ?
 * @param signal	poll signal for asynchronous call
 *
 * @return		0 on success or negative error code
 *
 * @note if DMA has been configured, the function will first attempt to
 *       transceive using DMA, but fall back to non-DMA transception if it is
 *       unabled to secure DMA channels for transmission and reception
 */
static int rspi_rx65n_transceive(const struct device *dev,
			const struct spi_config *spi_cfg,
			const struct spi_buf_set *tx_bufs,
			const struct spi_buf_set *rx_bufs,
			bool asynchronous,
			struct k_poll_signal *async)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);
	struct spi_context *ctx = &data->ctx;
	int ret = 0;

	spi_context_lock(ctx, asynchronous, async, spi_cfg);

	ret = rspi_rx65n_setup(dev, spi_cfg);
	if (ret != 0) {
		spi_context_release(ctx, ret);
		return ret;
	}

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_configure(ctx);
	spi_context_cs_control(ctx, true);

	#if CONFIG_DMA
		if (rspi_rx65n_transceive_dma(dev) != 0) {
			/* rspi_rx65n_transceive_dma changes the state of the buffers */
			spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);

			WRITE_BIT(*cfg->spcr, 7, true);
			WRITE_BIT(*cfg->spcr, 6, true);

			ret = rspi_rx65n_transceive_next_packet(dev);
		}
	#else
		WRITE_BIT(*cfg->spcr, 7, true);
		WRITE_BIT(*cfg->spcr, 6, true);

		ret = rspi_rx65n_transceive_next_packet(dev);
	#endif

	if (ret == 0) {
		/* spi_context_wait_for_completion does not block asynchronous */
		ret = spi_context_wait_for_completion(ctx);
	}

	spi_context_release(ctx, ret);

	return ret;
}

/**
 * @brief transceive data on a RSPI channel (API function)
 *
 * @param dev		RSPI device structure
 * @param spi_cfg	rpi connection parameters
 * @param tx_bufs	transmission data buffers
 * @param rx_bufs	reception data buffers
 *
 * @return		0 on success or negative error code
 */
static int rspi_rx65n_transceive_sync(const struct device *dev,
				const struct spi_config *spi_cfg,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	return rspi_rx65n_transceive(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL);
}


#ifdef CONFIG_SPI_ASYNC
/**
 * @brief start transceiving data on a RSPI channel asynchronously (API function)
 *
 * @param dev		RSPI device structure
 * @param spi_cfg	rpi connection parameters
 * @param tx_bufs	transmission data buffers
 * @param rx_bufs	reception data buffers
 * @param async		poll signal to trigger once transception was successful
 *
 * @return		0 on success or negative error code
 */
static int rspi_rx65n_transceive_async(const struct device *dev,
					const struct spi_config *spi_cfg,
					const struct spi_buf_set *tx_bufs,
					const struct spi_buf_set *rx_bufs,
					struct k_poll_signal *async)
{
	return rspi_rx65n_transceive(dev, spi_cfg, tx_bufs, rx_bufs, true, async);
}
#endif /* CONFIG_SPI_ASYNC*/

/**
 * @brief release a blocked SPI channel
 *
 * @param dev		RSPI device structure
 * @param spi_cfg	pointer to the same structure that blocked the device
 *
 * @return	always 0 (success)
 */
static int rspi_rx65n_release(const struct device *dev,
				const struct spi_config *spi_cfg)
{
	DEV_DATA(data, dev);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct spi_driver_api rspi_rx65n_api = {
	.transceive = rspi_rx65n_transceive_sync,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = rspi_rx65n_transceive_async,
#endif
	.release = rspi_rx65n_release,
};

/**
 * @brief interrupt handler for the reception buffer full interrupt
 *
 * @param arg	RSPI device driver structure (as void pointer)
 */
static void rspi_rx65n_spri_isr(void *arg)
{
	struct device *dev = (struct device *)arg;

	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);

#if CONFIG_DMA
	if (data->tx_dma_channel >= 0) {
		return;
	}
#endif

	if (data->ctx.rx_buf != NULL) {
		*data->ctx.rx_buf = *cfg->spdr;
	} else {
		/* data has to be read to clear the interrupt request */
		rx_dummy = *cfg->spdr;
	}
	spi_context_update_rx(&data->ctx, 1, 1);

	rspi_rx65n_transceive_next_packet(dev);
}

/**
 * @brief interrupt handler for the transmission buffer empty interrupt
 *
 * @note the function is currently unused, but it is necessary to set up the
 *       SPTI if DMA is used
 */
static void rspi_rx65n_spti_isr(void *arg)
{
}

/**
 * @brief initialize a RSPI device
 *
 * @param dev	RSPI device structure
 *
 * @return	0 on success or negative error code
 */
static int rspi_rx65n_init(const struct device *dev)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);

	clock_control_on(cfg->clock, (clock_control_subsys_t)&cfg->subsys);

	/* set SPI interface to master mode */
	WRITE_BIT(*cfg->spcr, 3, true);

	/* set data register to 8 bit TODO: dts ? */
	WRITE_BIT(*cfg->spdcr, 6, true);

	clock_control_get_rate(cfg->clock,
			       (clock_control_subsys_t)&cfg->subsys,
			       &data->clk_src_freq_hz);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

/**
 * @brief initialize a chip select gpio pin based on a device tree phandle array entry
 *
 * @param node_id	node id of the spi node
 * @param prop		property id of the cs-gpios phandle array
 * @param flags		flags to use for pin configuration
 *
 * After the initialization, the pin should be in inactive state.
 */
#define INIT_CS_GPIO(node_id, prop, idx) \
	if (ret == 0) {\
		ret = gpio_pin_configure(DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)), \
					DT_PHA_BY_IDX(node_id, prop, idx, pin), \
					DT_PHA_BY_IDX(node_id, prop, idx, \
							flags | GPIO_OUTPUT_INACTIVE)); \
	}

/* Each SPI channel requires at least 3 pins to be configured
 * through pinmux and allows for 0-4 chip select pins.
 *
 * If at least 4 pins are configured, the device runs using the
 * 4-wire SPI mode, if only 3 pins are configured, it runs with
 * clock-synchronous 3-wire mode
 */
#define RSPI_RX65N_INST_INIT(id) \
	static int rspi_rx65n_init##id(const struct device *dev) \
	{ \
		DEV_CFG(cfg, dev); \
		int ret = 0; \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(id, 0, irq), \
			DT_INST_IRQ_BY_IDX(id, 0, priority), \
			rspi_rx65n_spri_isr, DEVICE_DT_INST_GET(id), \
			DT_INST_IRQ_BY_IDX(id, 0, flags)); \
		irq_enable(DT_INST_IRQ_BY_IDX(id, 0, irq)); \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(id, 1, irq), \
			DT_INST_IRQ_BY_IDX(id, 1, priority), \
			rspi_rx65n_spti_isr, DEVICE_DT_INST_GET(id), \
			DT_INST_IRQ_BY_IDX(id, 1, flags)); \
		irq_enable(DT_INST_IRQ_BY_IDX(id, 1, irq)); \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(id, cs_gpios), \
			(DT_INST_FOREACH_PROP_ELEM(id, cs_gpios, INIT_CS_GPIO))); \
		DT_INST_FOREACH_PROP_ELEM(id, pinmuxs, RX_INIT_PIN)\
		WRITE_BIT(*cfg->spcr, 0, DT_INST_PHA_HAS_CELL_AT_IDX(id, pinmuxs, 3, pin)); \
		if (ret == 0) { \
			ret = rspi_rx65n_init(dev); \
		} \
		return ret; \
	} \
	static const struct rspi_rx65n_cfg rspi_rx65n_cfg##id = { \
		.spcr = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SPCR), \
		.sslp = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SSLP), \
		.sppcr = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SPPCR), \
		.spsr = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SPSR), \
		.spdr = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SPDR), \
		.spscr = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SPSCR), \
		.spssr = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SPSSR), \
		.spbr = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SPBR), \
		.spdcr = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SPDCR), \
		.spckd = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SPCKD), \
		.sslnd = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SSLND), \
		.spnd = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SPND), \
		.spcr2 = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SPCR2), \
		.spcmd = (struct rspi_rx65n_spcmd *)DT_INST_REG_ADDR_BY_NAME(id, SPCMD), \
		.spdcr2 = (uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SPDCR2), \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(id, clock)), \
		.subsys = DT_INST_CLOCK_RX65N_SUB_SYSTEM(id, clock_subsystems), \
		IF_ENABLED(CONFIG_DMA, \
			(.rx_dma = DEVICE_DT_GET(DT_INST_PHANDLE_BY_NAME(id, dmas, rx)), \
			.rx_dma_slot = DT_INST_PHA_BY_NAME(id, dmas, rx, slot), \
			.tx_dma = DEVICE_DT_GET(DT_INST_PHANDLE_BY_NAME(id, dmas, tx)), \
			.tx_dma_slot = DT_INST_PHA_BY_NAME(id, dmas, tx, slot),)) \
	}; \
	static struct rspi_rx65n_data rspi_rx65n_data##id = { \
		SPI_CONTEXT_INIT_LOCK(rspi_rx65n_data##id, ctx), \
		SPI_CONTEXT_INIT_SYNC(rspi_rx65n_data##id, ctx), \
	}; \
	DEVICE_DT_INST_DEFINE(id,\
			&rspi_rx65n_init##id, \
			NULL, \
			&rspi_rx65n_data##id, \
			&rspi_rx65n_cfg##id, \
			POST_KERNEL, \
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			&rspi_rx65n_api);

DT_INST_FOREACH_STATUS_OKAY(RSPI_RX65N_INST_INIT)
