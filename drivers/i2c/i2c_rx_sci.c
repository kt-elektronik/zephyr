/*
 * Copyright (c) 2022 KT-Elektronik, Klaucke und Partner GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * Driver for the Renesas RX SCI module in simple I2C mode. Specifically, this
 * is not a driver for the dedicated I2C-bus (RIICa).
 */

#define DT_DRV_COMPAT renesas_rx_sci_i2c

#include <stdlib.h>
#include <kernel.h>
#include <soc.h>
#include <drivers/i2c.h>
#include <device.h>
#include <drivers/clock_control.h>
#include <drivers/interrupt_controller/rxv2_irq.h>
#include <drivers/clock_control/clock_control_rx65n.h>
#include <sys/util.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_rx_sci, CONFIG_I2C_LOG_LEVEL);

/**
 * @brief configuration data structure for sci device in simple i2c mode
 */
struct i2c_rx_sci_cfg {
	/* Transmit Data Register */
	volatile uint8_t *tdr;
	/* Receive Data Register */
	volatile uint8_t *rdr;
	/* Serial Status Register */
	volatile uint8_t *ssr;
	/* Smart Card Mode Register */
	volatile uint8_t *scmr;
	/* I2C status register */
	volatile uint8_t *sisr;
	/* SPI Mode Register */
	volatile uint8_t *spmr;
	/* Serial Control Register */
	volatile uint8_t *scr;
	/* Serial Mode Register */
	volatile uint8_t *smr;
	/* Bit Rate Register */
	volatile uint8_t *brr;
	/* Modulation Duty Register */
	volatile uint8_t *mddr;
	/* Serial Extended Mode Register */
	volatile uint8_t *semr;
	/* Noise Filter Setting Register */
	volatile uint8_t *snfr;
	/* I2C Mode Register 1 */
	volatile uint8_t *simr1;
	/* I2C Mode Register 2 */
	volatile uint8_t *simr2;
	/* I2C Mode Register 3 */
	volatile uint8_t *simr3;
	const struct device *clock;
	/* clock subsystem id */
	const struct clock_control_rx65n_subsys subsys;
};

#define DEV_CFG(cfg, dev) const struct i2c_rx_sci_cfg *cfg = \
	(const struct i2c_rx_sci_cfg *) dev->config;

/**
 * @brief states of the I2C device
 */
enum i2c_rx_sci_state {
	I2C_RX_SCI_IDLE,
	I2C_RX_SCI_GENERATING_START_COND,
	I2C_RX_SCI_GENERATING_RESTART_COND,
	I2C_RX_SCI_GENERATING_STOP_COND,
	I2C_RX_SCI_WAITING_FOR_ACK,
	I2C_RX_SCI_NACK,
	I2C_RX_SCI_TRANSFERRING,
};

/**
 * @brief dynamic data structure for the SCI device in simple I2C mode
 */
struct i2c_rx_sci_data {
	/* current state of the device */
	enum i2c_rx_sci_state state;
	/* semaphore to lock the device during operation (transfer/configuration)*/
	struct k_sem lock;
	/* semaphore to synchronize device with interrupts */
	struct k_sem sync;
	/* callback structure for the Start/Restart/Stop Condition generated Interrupt (STI) */
	struct grp_intc_rx65n_callback sti_callback;
	/* callback structure for the Error Interrupt */
	struct grp_intc_rx65n_callback eri_callback;
	/* current message */
	struct i2c_msg *msg;
	/* read/write position in the current message */
	size_t msg_pos;
	/* target address of the messages in the queue */
	uint16_t addr;
};

#define DEV_DATA(data, dev) struct i2c_rx_sci_data *data = \
	(struct i2c_rx_sci_data *) dev->data;

/* the values to set the SIMR3 register to generate a start/restart/stop condition */
#define SIMR3_START_CONDITION 0x51
#define SIMR3_RESTART_CONDITION 0x52
#define SIMR3_STOP_CONDITION 0x54
/* while the I2C bus is idle (i.e. before generating a start condition), both SSDAn and SSCLn
 * should be in high-impedance state, i.e. SIMR3.IICSDAS and SIMR3.IICSCLS are set to 0b11
 */
#define SIMR3_IDLE 0xf0

/* lookup table for bitrate divider based on equation in table 40.12 of the
 * RX66N Manual. clock_divider = 64 * 2^(2 * SMR.CKS - 1)
 */
static const uint32_t clock_dividers[] = {32, 128, 512, 2048};

/**
 * @brief calculate the bitrate of the I2C interface
 *
 * @param f	operating frequency of the clock in Hz
 * @param cks	SCR.CKS setting
 * @param brr	BRR register value
 * @param M	MDDR register value
 *
 * @return device bitrate in bps
 */
static inline uint32_t calc_bitrate(uint32_t f, uint8_t cks, uint32_t brr, uint32_t M)
{
	return f / (clock_dividers[cks] * 256 * (brr + 1) / M);
}

/**
 * @brief configures I2C device (setting the bus-speed)
 *
 * @param dev		device to configure
 * @param dev_config	configuration flags (cf. ic2.h)
 *
 * @returns 0 on success or negative error code
 */
static int i2c_rx_sci_configure(const struct device *dev, uint32_t dev_config)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);
	uint32_t freq_hz = 0;
	int32_t bitrate = 0;
	uint8_t scr_backup;
	uint32_t best_error = 0xffffffff;
	uint32_t error;
	uint32_t br;
	uint8_t best_cks = 0;
	uint8_t best_brr = 0;
	uint8_t best_M = 0;

	if ((dev_config & I2C_MODE_MASTER) == 0) {
		/* current implementation only supports master mode */
		return -ENOTSUP;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		bitrate = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		bitrate = KHZ(400);
		break;
	case I2C_SPEED_FAST_PLUS:
	case I2C_SPEED_HIGH:
	case I2C_SPEED_ULTRA:
		LOG_ERR("I2C device %s only supports standard and fast speed, not %u", dev->name,
			I2C_SPEED_GET(dev_config));
		return -ENOTSUP;
	default:
		LOG_ERR("Invalid I2C speed setting %u", I2C_SPEED_GET(dev_config));
		return -EINVAL;
	}

	clock_control_get_rate(cfg->clock, (clock_control_subsys_t)&cfg->subsys, &freq_hz);

	for (uint8_t cks = 0; cks < 4; cks++) {
		for (uint16_t brr = 0; brr < 256; brr++) {
			br = calc_bitrate(freq_hz, cks, brr, 128);
			error = abs(br - bitrate);
			if (br > bitrate && error >= best_error) {
				/* all results for this cks,brr will be greater */
				continue;
			}
			br = calc_bitrate(freq_hz, cks, brr, 255);
			error = abs(br - bitrate);
			if (br < bitrate && error >= best_error) {
				/* all results for this cks,brr will be smaller */
				continue;
			}

			for (uint16_t M = 128; M < 257; M++) {
				br = calc_bitrate(freq_hz, cks, brr, M);
				error = abs(br - bitrate);
				if (error < best_error) {
					best_error = error;
					best_cks = cks;
					best_brr = brr;
					best_M = M;
				}
				if (br > bitrate) {
					/* increasing M will only increase the result */
					break;
				}
			}
		}
	}

	k_sem_take(&data->lock, K_FOREVER);

	/* while changing SMR and SEMR, SCR has to be 0. Backup the value to restore later */
	scr_backup = *cfg->scr;
	*cfg->scr = 0;

	/* unless the best result is for M = 256, use bitrate modulation */
	WRITE_BIT(*cfg->semr, 2, best_M < 256);
	if (best_M < 256) {
		*cfg->mddr = best_M;
	}
	/* set the divider SMR.CKS (b1,b0)*/
	*cfg->smr = (*cfg->smr & 0xfc) | best_cks;
	/* set the bit rate register */
	*cfg->brr = best_brr;

	*cfg->scr = scr_backup;

	k_sem_give(&data->lock);

	return 0;
}

/**
 * @brief transfer data on the I2C interface (API function)
 *
 * @param dev		device data structure
 * @param msgs		array of I2C messages
 * @param num_msgs	number of messages in msgs
 * @param addr		slave address to communicate with
 *
 * @return 0 on success or negative error code
 *
 * @note the function is blocking i.e. it returns after the transfer is complete
 */
static int i2c_rx_sci_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
	uint16_t addr)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);
	int res = 0;

	if (addr == 0) {
		/* Slave address can not be 0 */
		return -EINVAL;
	}

	if (num_msgs == 0) {
		/* no messages */
		return 0;
	}

	/* lock the I2C interface to this operation */
	k_sem_take(&data->lock, K_FOREVER);

	/* activate Start/Stop condition interrupt STI (SCR.TEIE = 1) */
	WRITE_BIT(*cfg->scr, 2, 1);
	/* activate transmit interrupt TXI (SCR.TIE = 1) */
	WRITE_BIT(*cfg->scr, 7, 1);
	/* deactivate Receive interrupt (SCR.RIE = 0) - even if this is a read operation, the
	 * master initiates it by sending the address of the device it requests data from
	 */
	WRITE_BIT(*cfg->scr, 6, 0);

	/* set SCR.TE (b5) and SCI.RE (b4) to enable transmit/receive */
	*cfg->scr = *cfg->scr | (BIT(4)|BIT(5));

	data->addr = addr;

	for (uint16_t i = 0; i < num_msgs; i++) {
		LOG_INF("%s: %s %u bytes", dev->name,
			(msgs[i].flags & I2C_MSG_READ)?"Reading":"Writing",
			msgs[i].len);
		data->msg = &msgs[i];
		data->msg_pos = 0;

		if (data->state == I2C_RX_SCI_IDLE) {
			LOG_INF("%s: generating start condition", dev->name);
			*cfg->simr3 = SIMR3_RESTART_CONDITION;
			data->state = I2C_RX_SCI_GENERATING_START_COND;
		} else if (msgs[i].flags & I2C_MSG_RESTART) {
			LOG_INF("%s: generating restart condition", dev->name);
			*cfg->simr3 = SIMR3_RESTART_CONDITION;
			data->state = I2C_RX_SCI_GENERATING_RESTART_COND;
		}

		/* wait for the (IRQ-controlled) transfer to be complete */
		res = k_sem_take(&data->sync, K_FOREVER);

		if (res < 0) {
			/* the semaphore was probably reset by i2c_recover_bus() */
			LOG_WRN("%s: k_sem_take error, cancelling", dev->name);
			data->state = I2C_RX_SCI_IDLE;
			break;
		}

		if (data->state == I2C_RX_SCI_NACK) {
			/* transmission has been cancelled as there was no ACK from the target. */
			data->state = I2C_RX_SCI_IDLE;
			res = -EIO;
			break;
		}
	}

	/* deactivate Start/Stop condition interrupt STI (SCR.TEIE = 1) */
	WRITE_BIT(*cfg->scr, 2, 0);
	/* deactivate transmit interrupt TXI (SCR.TIE = 1) */
	WRITE_BIT(*cfg->scr, 7, 0);
	/* deactivate Receive interrupt (SCR.RIE = 0) */
	WRITE_BIT(*cfg->scr, 6, 0);
	/* unset SCR.TE and SCI.RE to disable transmit/receive */
	*cfg->scr = *cfg->scr & ~(BIT(4)|BIT(5));

	data->msg = NULL;

	k_sem_give(&data->lock);
	return res;
}

/**
 * @brief attempt to recover I2C bus (API structure)
 *
 * @param dev	SCI I2C device data structure
 *
 * @return always 0
 */
static int i2c_rx_sci_recover_bus(const struct device *dev)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);

	/* Recovery from Bus Hang-up according to RX66N user manual section 40.7.7:
	 * - simultaneously unset SCR.TE and SCR.RE
	 * - set SIMR3 to idle (0xf0)
	 * - if SSR.RDRF is set (i.e. data received), dummy-read RDR
	 * - simultaneously set SCR.TE and SCR.RE
	 */
	*cfg->scr = *cfg->scr & ~(BIT(4) | BIT(5));
	*cfg->simr3 = SIMR3_IDLE;
	if (*cfg->ssr & BIT(6)) {
		uint8_t dummy = *cfg->rdr;

		ARG_UNUSED(dummy);
	}
	*cfg->scr = *cfg->scr | (BIT(4) | BIT(5));

	data->state = I2C_RX_SCI_IDLE;

	k_sem_reset(&data->sync);

	if (k_sem_count_get(&data->lock) == 0) {
		/* unlock the device for any waiting threads */
		k_sem_give(&data->lock);
	}

	return 0;
}

/**
 * @brief API structure for the I2C driver
 *
 * Renesas RX SCI in simple I2C mode only supports single master more, so slave_register
 * and slave_unregister are not implemented.
 */
const struct i2c_driver_api i2c_rx_sci_api = {
	.configure = i2c_rx_sci_configure,
	.transfer = i2c_rx_sci_transfer,
	.slave_register = NULL,
	.slave_unregister = NULL,
	.recover_bus = i2c_rx_sci_recover_bus,
};

/**
 * @brief initialize a SCI device in simple I2C mode
 *
 * @param dev	SCI I2C device data structure
 *
 * @returns 0 on success or negative error
 */
static int i2c_rx_sci_init(const struct device *dev)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);
	int res = 0;

	clock_control_on(cfg->clock, (clock_control_subsys_t)&cfg->subsys);

	/* lock semaphore is initially available */
	k_sem_init(&data->lock, 1, 1);
	/* synchronization semaphore is not initially available */
	k_sem_init(&data->sync, 0, 1);

	/* set SCR = 0 before (potentially) changing SMR */
	*cfg->scr = 0;

	/* set SSDAn and SSCLn to high impedance */
	*cfg->simr3 = SIMR3_IDLE;

	/* set SCI device to "Asynchronous mode or simple I2C mode" */
	WRITE_BIT(*cfg->smr, 7, 0);

	/* SCMR.SINV = 0 and SCMR.SDIR = 1 in accordance to RX66N user manual */
	WRITE_BIT(*cfg->scmr, 2, 0);
	WRITE_BIT(*cfg->scmr, 3, 1);

	/* initialize standard bitrate */
	res = i2c_rx_sci_configure(dev, I2C_MODE_MASTER | I2C_SPEED_SET(I2C_SPEED_STANDARD));

	/* select I2C mode with SIMR1.IICM = 1 (and no output delay) */
	*cfg->simr1 = 1;
	/* I2C interrupt settings:
	 * - use reception and transmission interrupts (SIMR2.IICINTM = 1)
	 * - Synchronization with the clock signal (SIMR2.IICCSC = 1)
	 * - NACK transmission and reception of ACK/NACK (SIMR2.IICACKT = 1)
	 */
	*cfg->simr2 = 0x23;

	data->state = I2C_RX_SCI_IDLE;

	return res;
}

/**
 * @brief ISR to handle the RXI (received data) interrupt
 *
 * @param arg	pointer to SCI I2C device data structure
 */
static void i2c_rx_sci_rxi_isr(const void *arg)
{
	const struct device *dev = (const struct device *)arg;

	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);

	LOG_DBG("%s: RXI", dev->name);

	if (data->state == I2C_RX_SCI_TRANSFERRING && data->msg_pos < data->msg->len) {
		data->msg->buf[data->msg_pos++] = *cfg->tdr;
	} else {
		if (data->state != I2C_RX_SCI_TRANSFERRING) {
			LOG_ERR("%s: RXI triggered in state %u, not I2C_RX_SCI_TRANSFERRING",
				dev->name, data->state);
		} else {
			LOG_ERR("%s: RXI triggered, but receive buffer full.", dev->name);
		}
	}
}

/**
 * @brief ISR to handle the TXI (transmittion ready) interrupt
 *
 * @param arg	pointer to SCI I2C device data structure
 */
static void i2c_rx_sci_txi_isr(const void *arg)
{
	const struct device *dev = (const struct device *)arg;

	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);

	LOG_DBG("%s: TXI", dev->name);

	if (data->state == I2C_RX_SCI_WAITING_FOR_ACK) {
		if ((*cfg->sisr & BIT(0)) == 1) {
			LOG_ERR("%s: no ACK from 0x%02x (SISR: 0x%02x)",
				dev->name, data->addr, *cfg->sisr);
			/* unset SISR.IICACKR bit */
			WRITE_BIT(*cfg->sisr, 0, 0);
			/* no ACK received after sending address. Generate stop condition and
			 * cancel transmission
			 */
			data->state = I2C_RX_SCI_NACK;
			*cfg->simr3 = SIMR3_STOP_CONDITION;
		} else {
			/* ACK received. Can start sending */
			LOG_INF("ACK received");
			data->state = I2C_RX_SCI_TRANSFERRING;
			if ((data->msg->flags & I2C_MSG_READ) == I2C_MSG_READ) {
				/* unset SIMR2.IICACKT */
				WRITE_BIT(*cfg->simr2, 5, 0);
				/* set SCR.RIE */
				WRITE_BIT(*cfg->scr, 6, 1);
			}
		}
	}

	if (data->state == I2C_RX_SCI_TRANSFERRING) {
		if (data->msg_pos < data->msg->len) {
			if ((data->msg->flags & I2C_MSG_READ) == I2C_MSG_READ) {
				if (data->msg_pos == data->msg->len - 1) {
					/* last byte to receive, set SIMR2.IICACKT */
					WRITE_BIT(*cfg->simr2, 5, 1);
				}
				/* send dummy byte */
				*cfg->tdr = 0xff;
			} else {
				LOG_DBG("Writing 0x%02x", data->msg->buf[data->msg_pos]);
				/* send message byte */
				*cfg->tdr = data->msg->buf[data->msg_pos++];
			}
		} else {
			/* clear SCR.RIE */
			WRITE_BIT(*cfg->scr, 6, 0);
			if (data->msg->flags & I2C_MSG_STOP) {
				LOG_DBG("%s: generating stop condition", dev->name);
				*cfg->simr3 = SIMR3_STOP_CONDITION;
				data->state = I2C_RX_SCI_GENERATING_STOP_COND;
				/* if a stop condition is to be generated, data->sync will be
				 * released in i2c_rx_sci_sti_isr()
				 */
			} else {
				/* otherwise, release it here */
				k_sem_give(&data->sync);
			}
		}
	}
}

/**
 * @brief isr for the STI interrupt
 *
 * @param arg	pointer to SCI I2C device data structure
 *
 * @note the STI interrupt replaces the TEI interrupt in simple I2C mode. It is generated when
 *       a start/stop condition has been generated
 */
static void i2c_rx_sci_sti_isr(const void *arg)
{
	const struct device *dev = (const struct device *)arg;

	DEV_DATA(data, dev);
	DEV_CFG(cfg, dev);

	switch (data->state) {
	case I2C_RX_SCI_GENERATING_START_COND:
	case I2C_RX_SCI_GENERATING_RESTART_COND:
		LOG_INF("(re)start condition generated, writing address (0x%02x) and R/W bit"
			" (0x%02x)", data->addr,
			(data->addr << 1) | ((data->msg->flags & I2C_MSG_READ) ? 1 : 0));
		/* (re)start condition generated, set SIMR3.IICSTIF, SIMR3.IICSDAS and
		 * SIMR3.IICSCLS to 0 and send target address and R/W bit
		 */
		*cfg->simr3 &= 0x07;
		/* the first byte to output is always the target address and the R/W bit */
		*cfg->tdr = (data->addr << 1) | ((data->msg->flags & I2C_MSG_READ) ? 1 : 0);
		data->state = I2C_RX_SCI_WAITING_FOR_ACK;
		break;
	case I2C_RX_SCI_GENERATING_STOP_COND:
		/* stop condition generated, set SIMR3.IICSTIF to 0, SIMR3.IICSDAS and
		 * SIMR3.IICSCLS to 0b11
		 */
		LOG_INF("stop condition generated, releasing sync");
		*cfg->simr3 = SIMR3_IDLE;
		data->state = I2C_RX_SCI_IDLE;
		k_sem_give(&data->sync);
		break;
	case I2C_RX_SCI_NACK:
		/* no ACK received, transmission cancelled and stop condition generated.
		 * treat as I2C_RX_SCI_GENERATING_STOP_COND, but don't reset data->state
		 */
		*cfg->simr3 = SIMR3_IDLE;
		k_sem_give(&data->sync);
		break;
	default:
		LOG_ERR("%s triggered STI, but no SIT expected (state %u)", dev->name, data->state);
		/* generate stop condition to prevent blocking state */
		*cfg->simr3 = SIMR3_STOP_CONDITION;
		data->state = I2C_RX_SCI_GENERATING_STOP_COND;
		break;
	}
}

/**
 * @brief ISR to handle the ERI (error) interrupt
 *
 * @param arg	pointer to SCI I2C device data structure
 */
static void i2c_rx_sci_error_isr(const void *arg)
{
	const struct device *dev = (const struct device *)arg;

	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);

	LOG_ERR("I2C error !");

	/* mark an error and generate a stop condition */
	data->state = I2C_RX_SCI_NACK;
	*cfg->simr3 = SIMR3_STOP_CONDITION;
}

#define RX_I2C_SCI_DEFINE(id) \
	static const struct i2c_rx_sci_cfg i2c_rx_sci_cfg_##id = { \
		.smr = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SMR), \
		.brr = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, BRR), \
		.scr = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SCR), \
		.tdr = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, TDR), \
		.ssr = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SSR), \
		.rdr = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, RDR), \
		.scmr = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SCMR), \
		.semr = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SEMR), \
		.snfr = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SNFR), \
		.simr1 = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SIMR1), \
		.simr2 = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SIMR2), \
		.simr3 = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SIMR3), \
		.sisr = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SISR), \
		.spmr = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, SPMR), \
		.mddr = (volatile uint8_t *)DT_INST_REG_ADDR_BY_NAME(id, MDDR), \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(id, clock)), \
		.subsys = DT_INST_CLOCK_RX65N_SUB_SYSTEM(id, clock_subsystems), \
	}; \
	static struct i2c_rx_sci_data i2c_rx_sci_data_##id = { \
		.sti_callback = { \
			.callback = i2c_rx_sci_sti_isr, \
			.param = DEVICE_DT_INST_GET(id), \
			.pin_mask = BIT(DT_INST_PROP(id, tei_number)),\
		},\
		.eri_callback = { \
			.callback = i2c_rx_sci_error_isr, \
			.param = DEVICE_DT_INST_GET(id), \
			.pin_mask = BIT(DT_INST_PROP(id, eri_number)),\
		},\
	}; \
	static int i2c_rx_sci_init_##id(const struct device *dev) \
	{ \
		DEV_DATA(data, dev); \
		int ret = 0; \
		DT_INST_FOREACH_PROP_ELEM(id, pinmuxs, RX_INIT_PIN); \
		if (ret == 0) { \
			IRQ_CONNECT(DT_INST_IRQ_BY_IDX(id, 0, irq), \
				DT_INST_IRQ_BY_IDX(id, 0, priority), \
				i2c_rx_sci_rxi_isr, DEVICE_DT_INST_GET(id), \
				DT_INST_IRQ_BY_IDX(id, 0, flags));\
			irq_enable(DT_INST_IRQ_BY_IDX(id, 0, irq));\
			IRQ_CONNECT(DT_INST_IRQ_BY_IDX(id, 1, irq), \
				DT_INST_IRQ_BY_IDX(id, 1, priority), \
				i2c_rx_sci_txi_isr, DEVICE_DT_INST_GET(id), \
				DT_INST_IRQ_BY_IDX(id, 1, flags));\
			irq_enable(DT_INST_IRQ_BY_IDX(id, 1, irq));\
			grp_intc_rx65n_manage_callback( \
				DEVICE_DT_GET(DT_INST_PHANDLE(id, tei_ctrl)), \
				&data->sti_callback, true); \
			grp_intc_rx65n_manage_callback( \
				DEVICE_DT_GET(DT_INST_PHANDLE(id, eri_ctrl)), \
				&data->eri_callback, true); \
		} \
		ret =  i2c_rx_sci_init(dev); \
		return ret; \
	} \
	DEVICE_DT_INST_DEFINE(id, \
		i2c_rx_sci_init_##id, \
		NULL, \
		&i2c_rx_sci_data_##id, \
		&i2c_rx_sci_cfg_##id, \
		POST_KERNEL, \
		CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
		&i2c_rx_sci_api, \
	);

DT_INST_FOREACH_STATUS_OKAY(RX_I2C_SCI_DEFINE)
