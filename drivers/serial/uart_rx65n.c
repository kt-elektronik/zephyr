/*
 * Copyright (c) 2021-2022 KT-Elektronik, Klaucke und Partner GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * Driver for the Renesas RX65N SCI module in asynchronous mode.
 * This MCU has 13 independent serial communications interface (SCI) channels.
 * The SCI consists of
 * - the SCIg module (SCI0 to SCI9),
 * - the SCIi module (SCI10 and SCI11), and
 * - the SCIh module (SCI12).
 * The SCIi module (SCI10 and SCI11) extends the base functionality by a FIFO.
 *
 */

#define DT_DRV_COMPAT renesas_rx_sci_uart

#include <kernel.h>
#include <arch/cpu.h>
#include <drivers/uart.h>
#include <drivers/uart/uart_rx.h>
#include <soc.h>
#include <sys/util.h>
#include <drivers/pinmux.h>
#include <drivers/gpio.h>
#include <drivers/clock_control.h>
#include <drivers/interrupt_controller/rxv2_irq.h>
#include <drivers/clock_control/clock_control_rx65n.h>

/* Renesas FIT module for iodefine.h data structures */
#include <platform.h>

#if CONFIG_SOC_SERIES_RX66N
	/* Renesas FIT module for sci_init_bit_rate */
	#include <r_sci_rx66n_private.h>
	/* "struct st_sci7" in the rx66n iodefine.h is the same as "struct st_sci10" in
	 * the rx65n iodefine.h as on rx66n SCI7-SCI11 are SCIi while on rx65n only SCI10-SCI11
	 * are SCIi.
	 * TODO: This is a quick fix, but on the long run, this should be solved using a
	 * more generalized approach (e.g. by using a full Zephyr-native driver without iodefine.h
	 * structures).
	 */
	#define st_sci10 st_sci7
#elif CONFIG_SOC_SERIES_RX65N
	/* Renesas FIT module for sci_init_bit_rate */
	#include <r_sci_rx65n_private.h>
#else
#error Unknown SOC, not (yet) supported.
#endif

/* suppress compiler warning if no UART is configured */
#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) > 0
#define DEV_CFG(dev) \
	((const struct uart_rx65n_device_config *const)(dev)->config)
#define DEV_DATA(dev) ((struct uart_rx65n_data *)(dev)->data)
#define DEV_BASE(dev) (DEV_CFG(dev)->base)

/* RX65N has three different types of SCI: SCIg, SCIi and SCIh*/
enum sci_type_t {
	UART_RX65N_UARTTYPE_SCIG,
	UART_RX65N_UARTTYPE_SCII,
	UART_RX65N_UARTTYPE_SCIH
};

struct uart_rx65n_device_config {
	/* base address of the device registers*/
	uint32_t base;
	/* subclock device for bit rate calculation*/
	const struct device *clock;
	/* subclock subsystem*/
	struct clock_control_rx65n_subsys clock_subsys;
	/* type of SCI: SCIg, SCIi or SCIh*/
	enum sci_type_t type;
	/* hardware fifo settings (only relevant for SCIi devices) */
	/* use hardware FIFO mode */
	bool hw_fifo;
	/* transmit FIFO threshold */
	uint8_t fifo_tx_irq_thresh;
	/* reveice FIFO RTS threshold */
	uint8_t fifo_tx_rts_thresh;
	/* receive FIFO interrupt threshold */
	uint8_t fifo_rx_thresh;
	/* gpio device of the RXD pin for break detection */
	const struct device *rxd_gpio;
	/* gpio pin numbfer of the RXD pin for break detection */
	uint8_t rxd_pin;
	/* extended serial mode enabled */
	bool extended;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	/* rx interrupt number (generated when data has been received) */
	uint8_t rx_irq;
	/* tx interrupt number (generated when data has been sent) */
	uint8_t tx_irq;
	/* group interrupt controller for the error interrupt (ERI) */
	const struct device *eri_ctrl;
	/*  group interrupt controller for the transmit end interrupt (TEI) */
	const struct device *tei_ctrl;
#endif
};

struct uart_rx65n_data {
	/* bitrate of the interface */
	uint32_t baudrate;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	/* external callback function for ALL interrupts (zephyr API) */
	uart_irq_callback_user_data_t callback;
	/* argument of the callback function */
	void *cb_data;
	/* callback structure for the error interrupt (ERI) */
	struct grp_intc_rx65n_callback eri_callback;
	/* callback structure for the transmission end interrupt (TEI) */
	struct grp_intc_rx65n_callback tei_callback;
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_rx65n_irq_handler(const void *param);
static void uart_rx65n_irq_err_enable(const struct device *dev);
#endif

/**
 * @brief Output a character in polled mode.
 *
 * Writes data to tx register if transmitter is not full.
 *
 * @param dev UART device struct
 * @param c Character to send
 */
static void uart_rx65n_poll_out(const struct device *dev, unsigned char c)
{
	/* struct st_sci0 can be used for all sci devices as the others only
	 * extends this struct
	 */
	volatile struct st_sci0 *sci = (struct st_sci0 *)DEV_BASE(dev);

	if (sci->SCR.BIT.TE == 0) {
		/* transmission disabled - can't send and the following loop
		 * would turn out to be infinite. Unfortunately, there is no
		 * way to return an error.
		 */
		return;
	}

	while (sci->SSR.BIT.TEND == 0) {
		;
	}
	sci->TDR = c;
}

/**
 * @brief Poll the device for input.
 *
 * @param dev UART device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */
static int uart_rx65n_poll_in(const struct device *dev, unsigned char *c)
{
	volatile struct st_sci0 *sci = (struct st_sci0 *)DEV_BASE(dev);

	if (sci->SSR.BIT.RDRF) {
		*c = sci->RDR;
		return 0;
	} else {
		return -1;
	}
}

/**
 * @brief get the device errors (without resetting)
 *
 * @param dev UART device struct
 *
 * @return bitmask based on enum uart_rx_stop_reason (uart.h)
 */
static int uart_rx65n_err_get(const struct device *dev)
{
	int error = 0;
	const struct uart_rx65n_device_config *dev_cfg = DEV_CFG(dev);

	if (dev_cfg->hw_fifo) {
		volatile struct st_sci10 *sci = (struct st_sci10 *)DEV_BASE(dev);

		if (sci->SSR.BIT.FER) {
			error |=  UART_ERROR_FRAMING;
			if (sci->SPTR.BIT.RXDMON == 0) {
				error |= UART_BREAK;
			}
		}
		if (sci->SSR.BIT.PER) {
			error |=  UART_ERROR_PARITY;
		}
		if (sci->SSR.BIT.ORER) {
			error |= UART_ERROR_OVERRUN;
		}
	} else {
		volatile struct st_sci0 *sci = (struct st_sci0 *)DEV_BASE(dev);

		if (sci->SSR.BIT.FER) {
			error |=  UART_ERROR_FRAMING;

			int rxd_state = gpio_pin_get(dev_cfg->rxd_gpio, dev_cfg->rxd_pin);

			if (rxd_state == 0) {
				error |= UART_BREAK;
			}
		}
		if (sci->SSR.BIT.PER) {
			error |=  UART_ERROR_PARITY;
		}
		if (sci->SSR.BIT.ORER) {
			error |= UART_ERROR_OVERRUN;
		}
	}

	return error;
}

/**
 * @brief reset the device errors
 *
 * @param dev UART device struct
 */
static void uart_rx65n_err_reset(const struct device *dev)
{
	const struct uart_rx65n_device_config *dev_cfg = DEV_CFG(dev);

	if (dev_cfg->hw_fifo) {
		volatile struct st_sci0 *sci = (struct st_sci0 *)DEV_BASE(dev);

		if (sci->SSR.BIT.FER) {
			sci->SSR.BIT.FER = 0;
		}
		if (sci->SSR.BIT.PER) {
			sci->SSR.BIT.PER = 0;
		}
		if (sci->SSR.BIT.ORER) {
			sci->SSR.BIT.ORER = 0;
		}
	} else {
		volatile struct st_sci0 *sci = (struct st_sci0 *)DEV_BASE(dev);

		if (sci->SSR.BIT.FER) {
			sci->SSR.BIT.FER = 0;
		}
		if (sci->SSR.BIT.PER) {
			sci->SSR.BIT.PER = 0;
		}
		if (sci->SSR.BIT.ORER) {
			sci->SSR.BIT.ORER = 0;
		}
	}
}

/**
 * @brief poll the device for errors
 *
 * @param dev UART device struct
 *
 * @return bitmask based on enum uart_rx_stop_reason (uart.h)
 */
static int uart_rx65n_err_check(const struct device *dev)
{
	int error = uart_rx65n_err_get(dev);

	uart_rx65n_err_reset(dev);

	return error;
}

/**
 * @brief poll the device for its current configuration
 *
 * @param dev UART device struct
 * @param cfg UART configuration struct that will contain the current
 *            configuration of the device after the call
 *
 * @return always 0
 */
static int uart_rx65n_config_get(const struct device *dev,
				 struct uart_config *cfg)
{
	struct uart_rx65n_data *dev_data = DEV_DATA(dev);
	volatile struct st_sci0 *sci = (struct st_sci0 *)DEV_BASE(dev);

	if (sci->SMR.BIT.CHR == 1 && sci->SCMR.BIT.CHR1 == 1) {
		cfg->data_bits = UART_CFG_DATA_BITS_7;
	} else if (sci->SMR.BIT.CHR == 0 && sci->SCMR.BIT.CHR1 == 1) {
		cfg->data_bits = UART_CFG_DATA_BITS_8;
	} else if (sci->SMR.BIT.CHR == 0 && sci->SCMR.BIT.CHR1 == 0) {
		cfg->data_bits = UART_CFG_DATA_BITS_9;
	}

	if (sci->SMR.BIT.PE == 0) {
		cfg->parity = UART_CFG_PARITY_NONE;
	} else {
		if (sci->SMR.BIT.PM == 0) {
			cfg->parity = UART_CFG_PARITY_EVEN;
		} else {
			cfg->parity = UART_CFG_PARITY_ODD;
		}
	}

	if (sci->SMR.BIT.STOP == 0) {
		cfg->stop_bits = UART_CFG_STOP_BITS_1;
	} else {
		cfg->stop_bits = UART_CFG_STOP_BITS_2;
	}

	if (sci->SPMR.BIT.CTSE == 0) {
		cfg->flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
	} else {
		cfg->flow_ctrl = UART_CFG_FLOW_CTRL_RTS_CTS;
	}

	/* returning the baudrate the interface was configured to use, not the actual one */
	cfg->baudrate = dev_data->baudrate;

	return 0;
}

/**
 * @brief configure the uart settings of a SCI device
 *
 * @param dev UART device struct
 * @param cfg UART configuration
 *
 * @return 0 on success or negative error code
 */
static int uart_rx65n_configure(const struct device *dev,
			     const struct uart_config *cfg)
{
	uint32_t pclk_freq_hz;
	volatile struct st_sci0 *sci = (struct st_sci0 *)DEV_BASE(dev);
	const struct uart_rx65n_device_config *dev_cfg = DEV_CFG(dev);
	struct uart_rx65n_data *dev_data = DEV_DATA(dev);
	/* backup SCR and set to 0 to allow changing configuration */
	uint8_t scr_backup = sci->SCR.BYTE;

	sci->SCR.BYTE = 0;

	switch (cfg->data_bits) {
	case UART_CFG_DATA_BITS_7:
		sci->SMR.BIT.CHR = 1;
		sci->SCMR.BIT.CHR1 = 1;
		break;
	case UART_CFG_DATA_BITS_8:
		sci->SMR.BIT.CHR = 0;
		sci->SCMR.BIT.CHR1 = 1;
		break;
	case UART_CFG_DATA_BITS_9:
		sci->SMR.BIT.CHR = 0;
		sci->SCMR.BIT.CHR1 = 0;
		break;
	default:
		return -ENOTSUP;
	}

	switch (cfg->parity) {
	case UART_CFG_PARITY_NONE:
		sci->SMR.BIT.PE = 0;
		break;
	case UART_CFG_PARITY_ODD:
		sci->SMR.BIT.PM = 1;
		sci->SMR.BIT.PE = 1;
		break;
	case UART_CFG_PARITY_EVEN:
		sci->SMR.BIT.PM = 0;
		sci->SMR.BIT.PE = 1;
		break;
	default:
		return -ENOTSUP;
	}

	switch (cfg->stop_bits) {
	case UART_CFG_STOP_BITS_1:
		sci->SMR.BIT.STOP = 0;
		break;
	case UART_CFG_STOP_BITS_2:
		sci->SMR.BIT.STOP = 1;
		break;
	default:
		return -ENOTSUP;
	}

	switch (cfg->flow_ctrl) {
	case UART_CFG_FLOW_CTRL_NONE:
		sci->SPMR.BIT.CTSE = 0;
		break;
	case UART_CFG_FLOW_CTRL_RTS_CTS:
		sci->SPMR.BIT.CTSE = 1;
	break;
	default:
		return -ENOTSUP;
	}

	dev_data->baudrate = cfg->baudrate;

	clock_control_get_rate(dev_cfg->clock,
		(clock_control_subsys_t)&dev_cfg->clock_subsys, &pclk_freq_hz);

	struct st_sci_ch_rom sci_rom = { .regs = (struct st_sci10 *)DEV_BASE(dev) };
	struct st_sci_ch_ctrl hdl = {	.rom = &sci_rom,
					.mode = SCI_MODE_ASYNC,
					.fifo_ctrl = dev_cfg->hw_fifo, };

	if (sci_init_bit_rate(&hdl, pclk_freq_hz, dev_data->baudrate) == 1000) {
		/* the function was unable to calculate appropriate settings
		 * for the desired bitrate (or failed in another way) and
		 * returned 100% error (return value 1000) - the interface is
		 * not configured
		 */
		dev_data->baudrate = 0;
		return -EINVAL;
	}

	/* restore SCR */
	sci->SCR.BYTE = scr_backup;

	return 0;
}

/**
 * @brief configure the rx and tx pins for a SCI device
 *
 * @param rx_pinmux_drv		pinmux driver for the rx pin
 * @param tx_pinmux_drv		pinmux driver for the tx pin
 * @param rx_gpio_drv		gpio driver for the rx pin
 * @param tx_gpio_drv		gpio driver for the tx pin
 * @param rx_pin		pin number of the rx pin
 * @param tx_pin		pin number of the tx pin
 * @param rx_func		pinmux function for the rx pin
 * @param tx_func		pinmux function for the tx pin
 * @param rx_flags		gpio flags for the rx pin
 * @param tx_flags		gpio flags for the tx pin
 *
 * @return 0 on success or negative error code
 */
static int uart_rx65n_init_pins(const struct device *rx_pinmux_drv,
				const struct device *tx_pinmux_drv,
				const struct device *rx_gpio_drv,
				const struct device *tx_gpio_drv,
				uint8_t rx_pin, uint8_t tx_pin,
				uint32_t rx_func, uint32_t tx_func,
				uint32_t rx_flags, uint32_t tx_flags)
{
	int ret;

	if (rx_pinmux_drv != NULL && device_is_ready(rx_pinmux_drv)) {
		ret = pinmux_pin_set(rx_pinmux_drv, rx_pin, rx_func);
	} else {
		ret = -EINVAL;
	}

	if (ret == 0 && tx_pinmux_drv != NULL && device_is_ready(tx_pinmux_drv)) {
		ret = pinmux_pin_set(tx_pinmux_drv, tx_pin, tx_func);
	} else {
		ret = -EINVAL;
	}

	if (ret == 0 && rx_gpio_drv != NULL && device_is_ready(rx_gpio_drv)) {
		ret = gpio_pin_configure(rx_gpio_drv, rx_pin,
			rx_flags | GPIO_INPUT | GPIO_PERIPHERAL);
	} else {
		ret = -EINVAL;
	}

	if (ret == 0 && tx_gpio_drv != NULL && device_is_ready(tx_gpio_drv)) {
		ret = gpio_pin_configure(tx_gpio_drv, tx_pin,
			tx_flags | GPIO_OUTPUT | GPIO_PERIPHERAL);
	} else {
		ret = -EINVAL;
	}
	return ret;
}

/**
 * @brief initialize a UART device
 *
 * @param dev UART device structure
 *
 * @return 0 on success or negative error code
 */
static int uart_rx65n_sci0_init(const struct device *dev)
{
	volatile struct st_sci0 *sci = (volatile struct st_sci0 *)DEV_BASE(dev);
	const struct uart_rx65n_device_config *cfg = DEV_CFG(dev);
	struct uart_rx65n_data *data = DEV_DATA(dev);
	int ret;

	clock_control_on(cfg->clock, (clock_control_subsys_t)&cfg->clock_subsys);

	/* Clear the control register: On-chip baud rate generator */
	sci->SCR.BYTE = 0;
	/* Asynchronous mode */
	sci->SIMR1.BIT.IICM = 0;

	if (cfg->hw_fifo) {
		/* only SCIi devices have a hardware fifo */
		struct st_sci10 *sci10 = (struct st_sci10 *)sci;

		sci10->FCR.BIT.FM = 1;
		sci10->FCR.BIT.TTRG = cfg->fifo_tx_irq_thresh;
		sci10->FCR.BIT.RTRG = cfg->fifo_rx_thresh;
		sci10->FCR.BIT.RSTRG = cfg->fifo_tx_rts_thresh;
		/* set SSRFIFO.DR to generate an ERI instead of RXI */
		sci10->FCR.BIT.DRES = 1;
	}

	if (cfg->type == UART_RX65N_UARTTYPE_SCIH && cfg->extended) {
		struct st_sci12 *sci12 = (struct st_sci12 *)DEV_BASE(dev);

		sci12->ESMER.BIT.ESME = 1;
	}

	struct uart_config uart_cfg = {
		.data_bits = UART_CFG_DATA_BITS_8,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
		.baudrate = data->baudrate,
	};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	irq_enable(cfg->rx_irq);
	irq_enable(cfg->tx_irq);
	uart_rx65n_irq_err_enable(dev);
#endif

	ret = uart_rx65n_configure(dev, &uart_cfg);

	if (ret != 0) {
		/* the configuration of the device has failed */
		return ret;
	}

	/* Enable transmission */
	sci->SCR.BIT.TE = 1;
	/* Enable reception */
	sci->SCR.BIT.RE = 1;

	return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

/**
 * @brief Interrupt driven FIFO fill function (Zephyr API)
 *
 * note: except for SCIi devices, the "fifo" is the one byte TDR and only one
 * byte at a time can be transmitted
 *
 * @param dev		UART device structure
 * @param tx_data	pointer to the data to be transmitted
 * @param lin		number of bytes to transmit
 *
 * @return		number of bytes actually transmitted (<= len)
 */
static int uart_rx65n_fifo_fill(const struct device *dev, const uint8_t *tx_data,
			 int len)
{
	const struct uart_rx65n_device_config *dev_cfg = DEV_CFG(dev);

	if (dev_cfg->hw_fifo) {
		volatile struct st_sci10 *sci = (volatile struct st_sci10 *)DEV_BASE(dev);
		uint8_t bytes_written = 0;

		while (sci->SCR.BIT.TE && bytes_written < len && sci->FDR.BIT.T < 16) {
			sci->FTDR.BYTE.L = tx_data[bytes_written];
			bytes_written++;
		}
		return bytes_written;
	}

	volatile struct st_sci0 *sci = (volatile struct st_sci0 *)DEV_BASE(dev);

	if (sci->SCR.BIT.TE && sci->SSR.BIT.TDRE) {
		/* currently only can write 1 byte at a time */
		sci->TDR = *tx_data;
		return 1;
	}
	return 0;
}

/**
 * @brief Interrupt driven FIFO read function (Zephyr API)
 *
 * note: except for SCIi devices, the receive fifo is the RDR and can only
 * contain one byte of data at a time
 *
 * @param dev		UART device structure
 * @param rx_data	pointer to where to store the data
 * @param size		size of rx_data (maximum number of bytes to read)
 *
 * @return		actual number of bytes read (<= size)
 */
static int uart_rx65n_fifo_read(const struct device *dev, uint8_t *rx_data,
			 const int size)
{
	const struct uart_rx65n_device_config *dev_cfg = DEV_CFG(dev);

	if (dev_cfg->hw_fifo) {
		volatile struct st_sci10 *sci = (volatile struct st_sci10 *)DEV_BASE(dev);
		uint8_t bytes_read = 0;

		while (bytes_read < size && sci->FDR.BIT.R > 0) {
			rx_data[bytes_read] = sci->FRDR.BYTE.L;
			bytes_read++;
		}
		return bytes_read;
	}

	volatile struct st_sci0 *sci = (volatile struct st_sci0 *)DEV_BASE(dev);

	if (sci->SSR.BIT.RDRF) {
		/* without hardware FIFO, only one byte can be read from the RDR */
		*rx_data = sci->RDR;
		return 1;
	}
	return 0;
}

/**
 * @brief Interrupt driven transfer enabling function (Zephyr API)
 *
 * will trigger the UART callback function as soon as it is possible to transmit
 * data
 *
 * @param dev	UART device structure
 */
static void uart_rx65n_irq_tx_enable(const struct device *dev)
{
	volatile struct st_sci0 *sci = (volatile struct st_sci0 *)DEV_BASE(dev);
	const struct uart_rx65n_device_config *cfg = DEV_CFG(dev);
	struct uart_rx65n_data *data = DEV_DATA(dev);

	sci->SCR.BIT.TIE = 1;
	sci->SCR.BIT.TEIE = 1;
	grp_intc_rx65n_manage_callback(cfg->tei_ctrl, &data->tei_callback, true);

	/* after calling tx_enable, Zephyr waits for the next interrupt to write
	 * data to the UART. The RX65N TXI interrupt only triggers after sending
	 * data, so if the TDR is empty, trigger a callback to allow the
	 * application to start transmitting data.
	 */
	if (sci->SSR.BIT.TDRE) {
		/* the callback function is usually called from an interrupt,
		 * preventing other interrupts to be triggered during execution
		 * just to be sure, lock interrupts while the callback is
		 * handled.
		 */
		uint32_t key = irq_lock();

		uart_rx65n_irq_handler(dev);
		irq_unlock(key);
	}
}

/**
 * @brief Interrupt driven transfer disabling function (Zephyr API)
 *
 * @param dev	UART device structure
 */
static void uart_rx65n_irq_tx_disable(const struct device *dev)
{
	volatile struct st_sci0 *sci = (volatile struct st_sci0 *)DEV_BASE(dev);
	const struct uart_rx65n_device_config *cfg = DEV_CFG(dev);
	struct uart_rx65n_data *data = DEV_DATA(dev);

	sci->SCR.BIT.TIE = 0;
	sci->SCR.BIT.TEIE = 0;
	grp_intc_rx65n_manage_callback(cfg->tei_ctrl, &data->tei_callback, false);
}

/**
 * @brief Interrupt driven transfer ready function (Zephyr API)
 *
 * @param dev	UART device structure
 *
 * @return 1 if data can be transferred, 0 else
 */
static int uart_rx65n_irq_tx_ready(const struct device *dev)
{
	const struct uart_rx65n_device_config *dev_cfg = DEV_CFG(dev);

	if (dev_cfg->hw_fifo) {
		volatile struct st_sci10 *sci = (volatile struct st_sci10 *)DEV_BASE(dev);

		return sci->SCR.BIT.TE && sci->FDR.BIT.T < 16;
	}

	volatile struct st_sci0 *sci = (volatile struct st_sci0 *)DEV_BASE(dev);

	return sci->SCR.BIT.TE && sci->SSR.BIT.TDRE;
}

/**
 * @brief Interrupt driven receiver enabling function (Zephyr API)
 *
 * @param dev	UART device structure
 */
static void uart_rx65n_irq_rx_enable(const struct device *dev)
{
	volatile struct st_sci0 *sci = (volatile struct st_sci0 *)DEV_BASE(dev);

	sci->SCR.BIT.RE = 1;
	sci->SCR.BIT.RIE = 1;
}

/**
 * @brief Interrupt driven receiver disabling function (Zephyr API)
 *
 * @param dev	UART device structure
 */
static void uart_rx65n_irq_rx_disable(const struct device *dev)
{
	volatile struct st_sci0 *sci = (volatile struct st_sci0 *)DEV_BASE(dev);

	sci->SCR.BIT.RE = 0;
	sci->SCR.BIT.RIE = 0;
}

/**
 * @brief Interrupt driven transfer complete function (Zephyr API)
 *
 * @param dev	UART device structure
 *
 * @return	1 if the UART has finished sending all data, 0 else
 */
static int uart_rx65n_irq_tx_complete(const struct device *dev)
{
	const struct uart_rx65n_device_config *dev_cfg = DEV_CFG(dev);

	if (dev_cfg->hw_fifo) {
		volatile struct st_sci10 *sci = (volatile struct st_sci10 *)DEV_BASE(dev);

		return sci->SSRFIFO.BIT.TEND;
	}

	volatile struct st_sci0 *sci = (volatile struct st_sci0 *)DEV_BASE(dev);

	return sci->SSR.BIT.TEND;
}

/**
 * @brief Interrupt driven receiver ready function (Zephyr API)
 *
 * @param dev	UART device structure
 *
 * @return	1 if received data is available, 0 else
 */
static int uart_rx65n_irq_rx_ready(const struct device *dev)
{
	const struct uart_rx65n_device_config *dev_cfg = DEV_CFG(dev);

	if (dev_cfg->hw_fifo) {
		volatile struct st_sci10 *sci = (volatile struct st_sci10 *)DEV_BASE(dev);

		return sci->SCR.BIT.RE && sci->FDR.BIT.R > 0;
	}

	volatile struct st_sci0 *sci = (volatile struct st_sci0 *)DEV_BASE(dev);

	return sci->SCR.BIT.RE && sci->SSR.BIT.RDRF;
}

/**
 * @brief Interrupt driven error enabling function (Zephyr API)
 *
 * @param dev	UART device structure
 */
static void uart_rx65n_irq_err_enable(const struct device *dev)
{
	const struct uart_rx65n_device_config *cfg = DEV_CFG(dev);
	struct uart_rx65n_data *data = DEV_DATA(dev);

	grp_intc_rx65n_manage_callback(cfg->eri_ctrl, &data->eri_callback, true);
}

/**
 * @brief Interrupt driven error disabling function (Zephyr API)
 *
 * @param dev	UART device structure
 */
static void uart_rx65n_irq_err_disable(const struct device *dev)
{
	const struct uart_rx65n_device_config *cfg = DEV_CFG(dev);
	struct uart_rx65n_data *data = DEV_DATA(dev);

	grp_intc_rx65n_manage_callback(cfg->eri_ctrl, &data->eri_callback, false);
}

/**
 * @brief Interrupt driven pending status function (Zephyr API)
 *
 * @param dev	UART device structure
 *
 * @return	1 if received data is available, 0 else
 */
static int uart_rx65n_irq_is_pending(const struct device *dev)
{
	return uart_rx65n_irq_rx_ready(dev) ||
		uart_rx65n_irq_tx_ready(dev) ||
		uart_rx65n_irq_tx_complete(dev) ||
		(uart_rx65n_err_get(dev) != 0);
}

/**
 * @brief Interrupt driven interrupt update function (Zephyr API)
 *
 * @param dev	UART device structure
 *
 * @return	always 1 (as required by the API)
 */
static int uart_rx65n_irq_update(const struct device *dev)
{
	return 1;
}

/**
 * @brief Set the irq callback function (Zephyr API)
 *
 * @param dev	UART device structure
 */
static void uart_rx65n_irq_callback_set(const struct device *dev,
				 uart_irq_callback_user_data_t cb,
				 void *user_data)
{
	DEV_DATA(dev)->callback = cb;
	DEV_DATA(dev)->cb_data = user_data;
}

static int uart_rx65n_drv_cmd(const struct device *dev, uint32_t cmd, uint32_t p)
{
	const struct uart_rx65n_device_config *dev_cfg = DEV_CFG(dev);
	volatile struct st_sci12 *sci12 = (volatile struct st_sci12 *)DEV_BASE(dev);

	switch (cmd) {
	case UART_RX_INVERT_RXD_TXD_POLARITY:
		/* while Renesas RX is definitely not the only MCU that allows the polarity of
		 * UART pins to be inverted, the Zephyr UART driver API does not have a way to
		 * use this function. If this changes in the future, this should be replaced.
		 */
		if (dev_cfg->type == UART_RX65N_UARTTYPE_SCIH && sci12->ESMER.BIT.ESME == 1) {
			sci12->PCR.BIT.TXDXPS = !sci12->PCR.BIT.TXDXPS;
			sci12->PCR.BIT.RXDXPS = !sci12->PCR.BIT.RXDXPS;
			return 0;
		}
	}

	return -ENOTSUP;
}

/**
 * @brief Handler function for all interrupts
 *
 * @param param	pointer to UART device
 */
static void uart_rx65n_irq_handler(const void *param)
{
	const struct device *dev = (const struct device *) param;
	struct uart_rx65n_data *data = DEV_DATA(dev);
	const struct uart_rx65n_device_config *dev_cfg = DEV_CFG(dev);

	if (data->callback) {
		data->callback(dev, data->cb_data);
	}

	/* reset error flags */
	if (dev_cfg->hw_fifo) {
		volatile struct st_sci10 *sci = (volatile struct st_sci10 *)DEV_BASE(dev);

		/* reset error flags */
		sci->SSRFIFO.BIT.PER = 0;
		sci->SSRFIFO.BIT.FER = 0;
		sci->SSRFIFO.BIT.ORER = 0;
		sci->SSRFIFO.BIT.TDFE = 0;

		/* In fifo mode, the Receive Data Full Flag and Transmit Data Empty Flag have
		 * to be reset explicitly.
		 */
		sci->SSRFIFO.BIT.RDF = 0;
		sci->SSRFIFO.BIT.DR = 0;
	} else {
		volatile struct st_sci0 *sci = (volatile struct st_sci0 *)DEV_BASE(dev);

		/* reset error flags */
		sci->SSR.BIT.PER = 0;
		sci->SSR.BIT.FER = 0;
		sci->SSR.BIT.ORER = 0;
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_rx65n_driver_api = {
	.poll_in = uart_rx65n_poll_in,
	.poll_out = uart_rx65n_poll_out,
	.err_check = uart_rx65n_err_check,
	.configure = uart_rx65n_configure,
	.config_get = uart_rx65n_config_get,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_rx65n_fifo_fill,
	.fifo_read = uart_rx65n_fifo_read,
	.irq_tx_enable = uart_rx65n_irq_tx_enable,
	.irq_tx_disable = uart_rx65n_irq_tx_disable,
	.irq_tx_ready = uart_rx65n_irq_tx_ready,
	.irq_rx_enable = uart_rx65n_irq_rx_enable,
	.irq_rx_disable = uart_rx65n_irq_rx_disable,
	.irq_tx_complete = uart_rx65n_irq_tx_complete,
	.irq_rx_ready = uart_rx65n_irq_rx_ready,
	.irq_err_enable = uart_rx65n_irq_err_enable,
	.irq_err_disable = uart_rx65n_irq_err_disable,
	.irq_is_pending = uart_rx65n_irq_is_pending,
	.irq_update = uart_rx65n_irq_update,
	.irq_callback_set = uart_rx65n_irq_callback_set,
#endif
#ifdef CONFIG_UART_DRV_CMD
	.drv_cmd = uart_rx65n_drv_cmd,
#endif
};

#endif /* DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) > 0 */

#define UART_RX65N_INIT_FUNCTION(id) \
static int uart_rx65n_init##id(const struct device *dev) \
{ \
	struct uart_rx65n_data *data = DEV_DATA(dev); \
	uart_rx65n_init_pins(DEVICE_DT_GET(DT_INST_PHANDLE_BY_NAME(id, pinmuxs, rxd)), \
		DEVICE_DT_GET(DT_INST_PHANDLE_BY_NAME(id, pinmuxs, txd)), \
		DEVICE_DT_GET(DT_INST_PHANDLE_BY_NAME(id, gpios, rxd)), \
		DEVICE_DT_GET(DT_INST_PHANDLE_BY_NAME(id, gpios, txd)), \
		DT_INST_PHA_BY_NAME(id, pinmuxs, rxd, pin), \
		DT_INST_PHA_BY_NAME(id, pinmuxs, txd, pin), \
		DT_INST_PHA_BY_NAME(id, pinmuxs, rxd, function), \
		DT_INST_PHA_BY_NAME(id, pinmuxs, txd, function), \
		DT_INST_PHA_BY_NAME(id, gpios, rxd, flags), \
		DT_INST_PHA_BY_NAME(id, gpios, txd, flags)); \
	IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN, \
		(IRQ_CONNECT(DT_INST_IRQ_BY_IDX(id, 0, irq), \
			DT_INST_IRQ_BY_IDX(id, 0, priority), \
			uart_rx65n_irq_handler, \
			DEVICE_DT_INST_GET(id), \
			DT_INST_IRQ_BY_IDX(id, 0, flags)); \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(id, 1, irq), \
			DT_INST_IRQ_BY_IDX(id, 1, priority), \
			uart_rx65n_irq_handler, \
			DEVICE_DT_INST_GET(id), \
			DT_INST_IRQ_BY_IDX(id, 1, flags)); \
		data->eri_callback.callback = uart_rx65n_irq_handler; \
		data->eri_callback.param = DEVICE_DT_INST_GET(id); \
		data->eri_callback.pin_mask = BIT(DT_INST_PROP(id, eri_number)); \
		data->tei_callback.callback = uart_rx65n_irq_handler; \
		data->tei_callback.param = DEVICE_DT_INST_GET(id); \
		data->tei_callback.pin_mask = BIT(DT_INST_PROP(id, tei_number));)) \
	return uart_rx65n_sci0_init(dev); \
};

#define UART_RX65N_CONFIG_INIT(id) \
UART_RX65N_INIT_FUNCTION(id) \
static const struct uart_rx65n_device_config uart_rx65n_dev_cfg_##id = { \
	.base = DT_INST_REG_ADDR(id), \
	.clock = DEVICE_DT_GET(DT_INST_PHANDLE(id, clock)),\
	.clock_subsys = DT_INST_CLOCK_RX65N_SUB_SYSTEM(id, clock_subsystems), \
	IF_ENABLED(DT_NODE_HAS_COMPAT(DT_DRV_INST(id), renesas_rx_scig), \
		(.type = UART_RX65N_UARTTYPE_SCIG, \
		 .hw_fifo = false, \
		 .extended = false,)) \
	IF_ENABLED(DT_NODE_HAS_COMPAT(DT_DRV_INST(id), renesas_rx_scii), \
		(.type = UART_RX65N_UARTTYPE_SCII, \
		 .hw_fifo = DT_INST_PROP(id, hw_fifo), \
		 .extended = false, \
		 .fifo_tx_irq_thresh = DT_INST_PROP_OR(id, fifo_tx_irq_thresh, 1), \
		 .fifo_tx_rts_thresh = DT_INST_PROP_OR(id, fifo_tx_rts_thresh, 1), \
		 .fifo_rx_thresh = DT_INST_PROP_OR(id, fifo_rx_thresh, 1),)) \
	IF_ENABLED(DT_NODE_HAS_COMPAT(DT_DRV_INST(id), renesas_rx_scih), \
		(.type = UART_RX65N_UARTTYPE_SCIH, \
		 .hw_fifo = false, \
		 .extended = DT_INST_PROP(id, extended_serial_mode),)) \
	IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN, \
		(.rx_irq = DT_INST_IRQ_BY_IDX(id, 0, irq), \
		.tx_irq = DT_INST_IRQ_BY_IDX(id, 1, irq), \
		.eri_ctrl = DEVICE_DT_GET(DT_INST_PHANDLE(id, eri_ctrl)), \
		.tei_ctrl = DEVICE_DT_GET(DT_INST_PHANDLE(id, tei_ctrl)),)) \
	.rxd_gpio = DEVICE_DT_GET(DT_INST_PHANDLE_BY_NAME(id, gpios, rxd)),\
	.rxd_pin = DT_INST_PHA_BY_NAME(id, pinmuxs, rxd, pin)};

#define RX_SCI_DEFINE(id) \
	static struct uart_rx65n_data uart_rx65n_data_##id = { \
		.baudrate = DT_INST_PROP_OR(id, current_speed, 0), \
	}; \
	UART_RX65N_CONFIG_INIT(id); \
	DEVICE_DT_INST_DEFINE(id, \
				uart_rx65n_init##id, \
				NULL, \
				&uart_rx65n_data_##id,  \
				&uart_rx65n_dev_cfg_##id, \
				PRE_KERNEL_1,  \
				CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
				(void *)&uart_rx65n_driver_api); \

DT_INST_FOREACH_STATUS_OKAY(RX_SCI_DEFINE)
