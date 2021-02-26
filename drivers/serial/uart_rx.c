/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH 
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//#define DT_DRV_COMPAT renesas_rx_sci


// adapted from  uart_sifive.c // uart_esp32.c
#include <kernel.h>
#include <arch/cpu.h>
#include <drivers/uart.h>
#include <rx_sci.h>
#include <soc.h>



#define DEV_CFG(dev) ((const struct uart_rx_device_config *const)(dev)->config)
#define DEV_DATA(dev) ((struct uart_rx_data *)(dev)->data)
#define DEV_BASE(dev) \
	((volatile struct st_sci12  *)(DEV_CFG(dev))->base)

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
typedef void (*irq_cfg_func_t)(void);
#endif

struct uart_rx_device_config {
	uintptr_t base;
	uint32_t sys_clk_freq;
	uint32_t baud_rate;
	uint8_t type;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	irq_cfg_func_t cfg_func;
#endif
};

struct uart_rx_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
};


/**
 * @brief Output a character in polled mode.
 *
 * Writes data to tx register if transmitter is not full.
 *
 * @param dev UART device struct
 * @param c Character to send
 */
static void uart_rx_poll_out(const struct device *dev,
					 unsigned char c)
{
	volatile struct st_sci_com *sci = (struct st_sci_com *) DEV_BASE(dev);

	while ((sci->SSR & BIT(2)) == 0);
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
static int uart_rx_poll_in(const struct device *dev, unsigned char *c)
{
	return 0;
}

static int uart_rx_err_check(const struct device *dev)
{
	return 0;
}

static int uart_rx_config_get(const struct device *dev,
				 struct uart_config *cfg)
{
	return 0;
}

static int uart_rx_configure(const struct device *dev,
				const struct uart_config *cfg)
{
	return 0;
}

static int uart_rx_sci0_init(const struct device *dev)
{
	/* do sci0 specific init */
	ARG_UNUSED(dev);
	return 0;
}

static int uart_rx_sci10_init(const struct device *dev)
{
	/* do sci10 specific init */
	ARG_UNUSED(dev);
	return 0;
}

static int uart_rx_sci12_init(const struct device *dev)
{
	volatile struct st_sci12 *sci = DEV_BASE(dev);

	REGISTER_WRITE_ENABLE(1);
	ENABLE_WRITING_MPC;

	/* Cancel SCI stop state */
	*(uint16_t*)0x00080014 &= ~BIT(4);

	/* Clear the control register */
	sci->SCR = 0x00U;
	/* Set clock enable */
	sci->SCR = _00_SCI_INTERNAL_SCK_UNUSED;
	/* Clear the SIMR1.IICM, SPMR.CKPH, and CKPOL bit */
	sci->SIMR1 &= ~BIT(0);
	sci->SPMR &= ~(BIT(6) | BIT(7));

	/* Set registers for first test*/
	// 115200 Baud
	sci->SPMR = _00_SCI_RTS | _00_SCI_CLOCK_NOT_INVERTED | _00_SCI_CLOCK_NOT_DELAYED;
	sci->SMR = _00_SCI_CLOCK_PCLK | _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_STOP_1 | 
			_00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8  | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	sci->SCMR = _00_SCI_SERIAL_MODE | _00_SCI_DATA_INVERT_NONE | _00_SCI_DATA_LSB_FIRST |
			_10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	sci->SEMR = _00_SCI_BIT_MODULATION_DISABLE | _10_SCI_8_BASE_CLOCK | 
			_00_SCI_NOISE_FILTER_DISABLE | _40_SCI_BAUDRATE_DOUBLE | _00_SCI_LOW_LEVEL_START_BIT;

	/* Set bit rate */
	sci->BRR = 0x40U;

 	return 0;
}

static const struct uart_driver_api uart_rx_driver_api = {
	.poll_in          = uart_rx_poll_in,
	.poll_out         = uart_rx_poll_out,
	.err_check        = uart_rx_err_check,
	.configure        = uart_rx_configure,
	.config_get       = uart_rx_config_get,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill        = uart_rx_fifo_fill,
	.fifo_read        = uart_rx_fifo_read,
	.irq_tx_enable    = uart_rx_irq_tx_enable,
	.irq_tx_disable   = uart_rx_irq_tx_disable,
	.irq_tx_ready     = uart_rx_irq_tx_ready,
	.irq_tx_complete  = uart_rx_irq_tx_complete,
	.irq_rx_enable    = uart_rx_irq_rx_enable,
	.irq_rx_disable   = uart_rx_irq_rx_disable,
	.irq_rx_ready     = uart_rx_irq_rx_ready,
	.irq_err_enable   = uart_rx_irq_err_enable,
	.irq_err_disable  = uart_rx_irq_err_disable,
	.irq_is_pending   = uart_rx_irq_is_pending,
	.irq_update       = uart_rx_irq_update,
	.irq_callback_set = uart_rx_irq_callback_set,
#endif
};



#define RX_CONFIG_INIT(id) \
static const struct uart_rx_device_config uart_rx_dev_cfg_##id = { \
	.base         = DT_INST_REG_ADDR(id), \
	.baud_rate    = DT_INST_PROP(id, current_speed), \
	IF_ENABLED( \
		CONFIG_UART_INTERRUPT_DRIVEN, \
		(.cfg_func     = uart_rx_irq_cfg_func_##id,)) \
}; 

// #ifdef CONFIG_UART_INTERRUPT_DRIVEN \
// static void uart_rx_irq_cfg_func_##id(void); \
// #endif


#define RX_SCIG_DEFINE(id) \
static const struct uart_rx_device_config uart_rx_dev_cfg_g##id = {}; \
static struct st_sci0 uart_rx_data_g##id; \
DEVICE_DT_DEFINE(id, \
			uart_rx_sci0_init, \
			device_pm_control_nop, \
			&uart_rx_data_g##id,  \
			&uart_rx_dev_cfg_g##id, \
			PRE_KERNEL_2,  \
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			(void *)&uart_rx_driver_api);


#define RX_SCII_DEFINE(id) \
static const struct uart_rx_device_config uart_rx_dev_cfg_i##id = {}; \
static struct st_sci10 uart_rx_data_i##id; \
DEVICE_DT_DEFINE(id, \
			uart_rx_sci10_init, \
			device_pm_control_nop, \
			&uart_rx_data_i##id,  \
			&uart_rx_dev_cfg_i##id, \
			PRE_KERNEL_2,  \
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			(void *)&uart_rx_driver_api);


#define RX_SCIH_DEFINE(id) \
RX_CONFIG_INIT(id) \
static struct st_sci12 uart_rx_data_##id; \
DEVICE_DT_DEFINE(id, \
			uart_rx_sci12_init, \
			device_pm_control_nop, \
			&uart_rx_data_##id,  \
			&uart_rx_dev_cfg_##id, \
			PRE_KERNEL_2,  \
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			(void *)&uart_rx_driver_api);


#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT renesas_rx_scig
DT_INST_FOREACH_STATUS_OKAY(RX_SCIG_DEFINE)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT renesas_rx_scii
DT_INST_FOREACH_STATUS_OKAY(RX_SCII_DEFINE)

// #undef DT_DRV_COMPAT
// #define DT_DRV_COMPAT renesas_rx_scih
// DT_INST_FOREACH_STATUS_OKAY(RX_SCIH_DEFINE)


