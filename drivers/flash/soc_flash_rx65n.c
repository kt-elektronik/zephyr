/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 * SPDX-License-Identifier: Apache-2.0
 *
 * Flash controller for the RX65N SOC flash. All RX65N processors have up
 * to 2 MB code flash, some have 32 KB of data flash in addition. In these cases,
 * two different flash devices are generated, as the access to code and data
 * flash is different. The flash capacity of an RX65N MCU can be deduced from the
 * product number:
 *
 *  - R5F565NE: 2 MB code flash flash, 32 KB data flash
 *  - R5F565NC: 1.5 MB code flash, 32 KB data flash
 *  - R5F565N9: 1 MB code flash, no data flash
 *  - R5F565N7: 768 KB code flash, no data flash
 *  - R5F565N4: 512 KB code flash, no data flash
 *
 * if both code and data flash exist, they both use the same flash sequencer
 * (i.e. Flash Control Unit (FCU) and Flash Application Command Interface (FACI)),
 * i.e. only one of them can be programmed/erased at the same time. Also, while
 * in program/erase (P/E) mode, the flash can not be read. The code flash is
 * still readable while the data flash is in P/E mode.
 */

#define DT_DRV_COMPAT renesas_rx65n_flash_controller

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <kernel.h>
#include <device.h>
#include <drivers/flash.h>
#include <soc.h>

#define FCU_NODE DT_NODELABEL(fcu)

#define LOG_LEVEL CONFIG_FLASH_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(flash_rx65n);

#define FENTRYR_KEY			(0xaa00)

#define FACI_CMD_PROGRAM		(0xe8)
#define FACI_CMD_BLOCK_ERASE		(0x20)
#define FACI_CMD_MULTI_BLOCK_ERASE	(0x21)
#define FACI_CMD_SUSPEND		(0xb0)
#define FACI_CMD_RESUME			(0xd0)
#define FACI_CMD_STATUS_CLEAR		(0x50)
#define FACI_CMD_FORCE_STOP		(0xb3)
#define FACI_CMD_BLANK_CHECK		(0x71)
#define FACI_CMD_CONFIGURATION		(0x40)
#define FACI_CMD_FINAL			(0xd0)

/* data buffer timeout is 1.4 usec (when FCLK >= 20 MHz) according to RX65N User's
 * manual. The wait loop required several processor cycles per loop, but
 * estimating one 120 MHz cycle per loop will definitely result in a "safe"
 * timeout counter, where 1.4 usec * 120 MHz = 14 * 12.
 */
#define FACI_DATA_BUFFER_TIMEOUT_CNT		(168)

/* Data Flash Blank check timeouts are given for three different data lengths in
 * the RX65N User's Manual (Table 60.57) as
 *
 *   30 us for  4 bytes
 *  100 us for 64 bytes
 * 2200 us for  2 Kbytes
 *
 * This suggests a linear relationship with an intercept of approx. 30 usec and a
 * slope of approx. 1.1 usec/byte. Again, considering the 120 MHz cycle and generously
 * assuming a wait loop to running one processor cycle per iteration, the wait time (in
 * counts) can be calculated as 30 usec * 120 MHz = 3600 and 1.1 usec * 120 MHz = 132
 */
#define FACI_BLANK_CHECK_TIMEOUT_INTERCEPT_CNT	(3600)
#define FACI_BLANK_CHECK_TIMEOUT_SLOPE_CNT	(132)

/**
 * @brief common semaphore for using the FCU
 *
 * @note since both code flash and data flash use the same semaphore, only one
 *	of them can be programmed at any time
 */
static struct k_sem fcu_sem;

/**
 * @brief data structure for a flash controller
 */
struct flash_rx65n_data {
	/** @brief block any access while programming */
	struct k_sem access_sem;
};

/**
 * @brief configuration of the common flash control unit
 */
struct flash_rx65n_fcu_cfg {
	/* address of the Flash Application Command Interface */
	volatile uint8_t *faci;
	/* Flash P/E Protect Register */
	volatile uint8_t *fwepror;
	/* Flash P/R Mode Entry Register */
	volatile uint16_t *fentryr;
	/* Flash Status Register */
	volatile uint32_t *fstatr;
	/* Flash Access Status Register */
	volatile uint8_t *fastat;
	/* FACI Command Start Address Register */
	volatile uint32_t *fsaddr;
	/* FACI Command End Address Register */
	volatile uint32_t *feaddr;
	/* Flash Sequencer Processing Switching Register */
	volatile uint16_t *fcpsr;
	/* Flash Access Error Interrupt Enable Register */
	volatile uint8_t *faeint;
	/* Flash Ready Interrupt Enable Register */
	volatile uint8_t *frdyie;
	/* Data Flash Blank Check Control Register */
	volatile uint8_t *fbccnt;
	/* Data Flash Blank Check Status Register */
	volatile uint8_t *fbcstat;
	/* Data Flash Programming Start Address Register */
	volatile uint32_t *fpsaddr;
};

static struct flash_rx65n_fcu_cfg fcu_cfg = {
	.faci = (uint8_t *)DT_REG_ADDR_BY_NAME(FCU_NODE, FACI),
	.fwepror = (uint8_t *)DT_REG_ADDR_BY_NAME(FCU_NODE, FWEPROR),
	.fentryr = (uint16_t *)DT_REG_ADDR_BY_NAME(FCU_NODE, FENTRYR),
	.fstatr = (uint32_t *)DT_REG_ADDR_BY_NAME(FCU_NODE, FSTATR),
	.fastat = (uint8_t *)DT_REG_ADDR_BY_NAME(FCU_NODE, FASTAT),
	.fsaddr = (uint32_t *)DT_REG_ADDR_BY_NAME(FCU_NODE, FSADDR),
	.feaddr = (uint32_t *)DT_REG_ADDR_BY_NAME(FCU_NODE, FEADDR),
	.fcpsr = (uint16_t *)DT_REG_ADDR_BY_NAME(FCU_NODE, FCPSR),
	.faeint = (uint8_t *)DT_REG_ADDR_BY_NAME(FCU_NODE, FAEINT),
	.frdyie = (uint8_t *)DT_REG_ADDR_BY_NAME(FCU_NODE, FRDYIE),
	.fbccnt = (uint8_t *)DT_REG_ADDR_BY_NAME(FCU_NODE, FBCCNT),
	.fbcstat = (uint8_t *)DT_REG_ADDR_BY_NAME(FCU_NODE, FBCSTAT),
	.fpsaddr = (uint32_t *)DT_REG_ADDR_BY_NAME(FCU_NODE, FPSADDR),
};

/**
 * @brief configuration of a flash device (i.e. code flash or data flash)
 */
struct flash_rx65n_cfg {
	/* start address of this particular flash in bytes */
	uint32_t read_offset;
	/* size in bytes */
	size_t size;
	/* FENTRYR bit for this flash */
	uint8_t entry_bit;
	/* flash parameters (block size and erase value) */
	const struct flash_parameters *parameters;
	/* page layout */
	const struct flash_pages_layout *layout;
	/* size of the page layout array */
	uint8_t layout_size;
	/* timeout for programming one block in ms */
	uint32_t program_timeout_ms;
	/* timeout of erasing one page in ms */
	uint32_t erase_timeout_ms;
	/* if true, a blank check is necessary to identify erased blocks */
	bool erased_undefined;
};

#define DEV_CFG(cfg, dev) const struct flash_rx65n_cfg *cfg = \
					(struct flash_rx65n_cfg *)dev->config;
#define DEV_DATA(data, dev) struct flash_rx65n_data *data = \
					(struct flash_rx65n_data *)dev->data;

/* wait poll signal for background operations, used by error and ready irq */
static struct k_poll_signal flash_wait_signal;

/* wait events for background operations */
struct k_poll_event flash_wait_events[1] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
		&flash_wait_signal, 0),
};

/**
 * @brief force the FCU to stop the current operation and reset error flags
 */
static void fcu_rx65n_force_stop(void)
{
	*fcu_cfg.faci = FACI_CMD_FORCE_STOP;

	/* wait for ready bit*/
	while ((*fcu_cfg.fstatr & BIT(15)) == 0) {
		;
	}
}

/**
 * @brief ISR for the flash read interrupt
 *
 * @param arg	unused
 */
static void fcu_rx65n_flash_ready_isr(void *arg)
{
	ARG_UNUSED(arg);
	LOG_INF("Flash ready interrupt.");

	if (!flash_wait_signal.signaled) {
		/* raise the signal with result 0 (ready) */
		k_poll_signal_raise(&flash_wait_signal, 0);
	}
}

/**
 * @brief ISR for the flash error interrupt
 *
 * @param arg	unused
 */
static void fcu_rx65n_flash_error_isr(void *arg)
{
	ARG_UNUSED(arg);
	LOG_ERR("Flash error interrupt.");

	/* raise the signal with result -1 (error) */
	k_poll_signal_raise(&flash_wait_signal, -1);
	fcu_rx65n_force_stop();
}

static void reset_flash_events(void)
{
	flash_wait_events[0].signal->signaled = 0;
	flash_wait_events[0].state = K_POLL_STATE_NOT_READY;
}

/**
 * @brief wait for a flash operation to finish in the background (ceases control)
 *
 * @param timeout	timeout for the operation (will force stop after this)
 *
 * @return		0 on success or negative error code
 */
static int fcu_rx65n_wait_ready_bgo(k_timeout_t timeout)
{
	LOG_INF("Waiting for flash ready.");
	k_poll(flash_wait_events, ARRAY_SIZE(flash_wait_events), timeout);
	if (flash_wait_events[0].signal->signaled) {
		reset_flash_events();
		return flash_wait_events[0].signal->result;
	}
	/* timeout */
	fcu_rx65n_force_stop();
	return -ETIME;
}

/**
 * @brief wait for a flash operation to finish
 *
 * @param cnt	timeout in loop-counts (will force stop after this)
 *
 * @return	0 on success or negative error code
 */
static int fcu_rx65n_wait_ready(uint32_t cnt)
{
	/* wait for ready bit */
	while (!(*fcu_cfg.fstatr & BIT(15)) && cnt-- > 0) {
		;
	}

	if (!(*fcu_cfg.fstatr & BIT(15))) {
		/* timeout */
		fcu_rx65n_force_stop();
		return -ETIME;
	}
	return 0;
}

/**
 * @brief enable programming/erase mode of the FCU (also needed for blank check)
 *
 * @param entry_bit	bit of the FENTRYR to set
 */
static void fcu_rx65n_enable_pe(uint8_t entry_bit)
{
	k_sem_take(&fcu_sem, K_FOREVER);

	/* enable all error and ready interrupts */
	*fcu_cfg.faeint = 0x98;
	*fcu_cfg.frdyie = 1;

	/* enable programming/erasing */
	*fcu_cfg.fwepror = 1;
	while (*fcu_cfg.fwepror != 1) {
		;
	}

	/* use the key and the entry bit for the specific flash */
	*fcu_cfg.fentryr = FENTRYR_KEY | BIT(entry_bit);
	while ((*fcu_cfg.fentryr & BIT(entry_bit)) == 0) {
		;
	}
}

/**
 * @brief disable programming/erase mode
 */
static void fcu_rx65n_disable_pe(void)
{
	/* disable interrupts */
	*fcu_cfg.faeint = 0;
	*fcu_cfg.frdyie = 0;

	/* disable programming/erasing */
	while (*fcu_cfg.fentryr != 0) {
		*fcu_cfg.fentryr = FENTRYR_KEY;
	}
	*fcu_cfg.fwepror = 0;

	k_sem_give(&fcu_sem);
}

/**
 * @brief perform a blank check on a region of flash
 *
 * @param dev		flash device driver structure
 * @param offset	start-offset of the region - has to be write-block aligned
 * @param len		length of the region to check
 *
 * @returns		number of unprogrammed bytes before the first programmed
 *			region after offset or negative error code
 *
 * @note the function has to be called in "programming/erase" mode
 */
static int fcu_rx65n_blank_check(const struct device *dev, off_t offset, size_t len)
{
	DEV_CFG(cfg, dev);
	int ret;

	if (offset % cfg->parameters->write_block_size) {
		return -EINVAL;
	}

	if ((offset + len) % cfg->parameters->write_block_size) {
		return -EINVAL;
	}

	*fcu_cfg.fbccnt = 0;
	*fcu_cfg.fsaddr = offset + cfg->read_offset;
	while (*fcu_cfg.fsaddr != offset + cfg->read_offset) {
		;
	}
	*fcu_cfg.feaddr = offset + cfg->read_offset + len;
	while (*fcu_cfg.feaddr != offset + cfg->read_offset + len) {
		;
	}

	*fcu_cfg.faci = FACI_CMD_BLANK_CHECK;
	*fcu_cfg.faci = FACI_CMD_FINAL;

	ret = fcu_rx65n_wait_ready(FACI_BLANK_CHECK_TIMEOUT_INTERCEPT_CNT +
				FACI_BLANK_CHECK_TIMEOUT_SLOPE_CNT * len);

	if (ret < 0) {
		LOG_ERR("Error %d during blank check", ret);
		return ret;
	}

	if (*fcu_cfg.fastat & BIT(4)) {
		/* command lock state */
		LOG_ERR("FCU in lock state after blank check.");
		fcu_rx65n_force_stop();
		return -EIO;
	}

	if (*fcu_cfg.fbcstat == 0) {
		/* the whole region is blank */
		return len;
	}

	/* at least part of the region has already be programmed. The address
	 * (relative to the start of the flash) of the first programmed block is
	 * in the FPSADDR
	 */
	return *fcu_cfg.fpsaddr - offset;
}

/**
 * @brief read data from the flash (API function)
 *
 * @param dev		flash device driver
 * @param offset	offset from the start of the flash to start reading from
 * @param buffer	pointer to memore where to write the data
 * @param len		number of bytes to copy
 *
 * @return		0 on success or negative error code
 */
static int flash_rx65n_read(const struct device *dev, off_t offset, void *buffer,
				size_t len)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);
	int ret = 0;

	if (offset + len >= cfg->size) {
		return -EINVAL;
	}

	if (buffer == NULL) {
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	k_sem_take(&data->access_sem, K_FOREVER);

	memcpy(buffer, (uint8_t *)(cfg->read_offset + offset), len);

	if (cfg->erased_undefined) {
		uint8_t *buf = (uint8_t *)buffer;

		/* erased regions of the flash have undefined values. To mark
		 * these for applications, set the corresponding read buffer
		 * to the cfg->parameters->erase_value. A blank check can be
		 * performed for each write block.
		 */

		fcu_rx65n_enable_pe(cfg->entry_bit);

		/* reads do not have to be aligned with write blocks, but blank
		 * checks do. If the read is not aligned, the check has to
		 * include the block before and/or after the read
		 */
		size_t len_before = offset % cfg->parameters->write_block_size;
		size_t check_len = len + len_before;

		if (check_len % cfg->parameters->write_block_size) {
			check_len += cfg->parameters->write_block_size -
				(check_len % cfg->parameters->write_block_size);
		}

		size_t check_pos = 0;

		while (len > 0) {
			ret = fcu_rx65n_blank_check(dev, offset + check_pos - len_before,
						check_len);

			if (ret < 0) {
				break;
			}
			if (ret - len_before < len) {
				memset(&buf[check_pos],	cfg->parameters->erase_value,
						ret - len_before);
				if (ret - len_before + cfg->parameters->write_block_size > len) {
					ret = 0;
					break;
				}
				len -= ret - len_before + cfg->parameters->write_block_size;
				check_len -= ret + cfg->parameters->write_block_size;
			} else {
				memset(&buf[check_pos],	cfg->parameters->erase_value,
						len);
				len = 0;
			}
			check_pos += ret - len_before + cfg->parameters->write_block_size;

			ret = 0;
			len_before = 0;
			/* based on the blank check we know that at least one
			 * write_block is programmed after blank_bytes bytes of
			 * blank flash, so increase the check_pos accordingly
			 */
		}

		fcu_rx65n_disable_pe();
	}

	k_sem_give(&data->access_sem);

	return ret;
}

/**
 * @brief write a single block of data to flash using the FCU
 *
 * @param data	pointer to the data to write
 * @param len	number of bytes to write
 *
 * @return	0 on success or negative error code
 *
 * @note
 */
static int fcu_rx65n_write_block(const void *data, uint8_t len)
{
	uint8_t timeout_cnt;
	uint16_t *pos = (uint16_t *)data;

	*fcu_cfg.faci = FACI_CMD_PROGRAM;
	/* writing 2 bytes at a time, so len/2 operations */
	*fcu_cfg.faci = len / 2;

	while (len > 0) {
		/* due to fcu write buffer size, push data in 2 byte chunks */
		*(uint16_t *)fcu_cfg.faci = *pos++;

		len -= 2;

		/* wait for data buffer full flag to be cleared
		 * (roughly) estimate the 1.4us (according to RX65N flash user's
		 * manual) by counting loops
		 */
		timeout_cnt = FACI_DATA_BUFFER_TIMEOUT_CNT;
		while (*fcu_cfg.fstatr & BIT(10)
			&& timeout_cnt-- > 0) {
			;
		}
		if (*fcu_cfg.fstatr & BIT(10)) {
			/* timeout */
			fcu_rx65n_force_stop();
			return -ETIME;
		}
	}
	*fcu_cfg.faci = FACI_CMD_FINAL;

	return 0;
}

/**
 * @brief write data to the flash (API function)
 *
 * @param dev		flash decvice driver
 * @param offset	position in the flash to write to (has to be aligned to
 *			the beginning of a block)
 * @param buffer	pointer to the data to write
 * @param len		number of bytes to write (has to be multiple of block
 *			length)
 *
 * @return		0 if successful or negative error code
 */
static int flash_rx65n_write(const struct device *dev, off_t offset,
			       const void *buffer, size_t len)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);
	int ret = 0;
	uint8_t *buf = (uint8_t *)buffer;

	if ((offset + len >= cfg->size)
		|| ((len % cfg->parameters->write_block_size) != 0)) {
		return -EINVAL;
	}

	if (len == 0) {
		return 0;
	}

	k_sem_take(&data->access_sem, K_FOREVER);

	LOG_DBG("Writing %lu bytes to %s at offset %d.", len, dev->name, (int)offset);

	while (!(*fcu_cfg.fstatr & BIT(15))) {
		;
	}

	fcu_rx65n_enable_pe(cfg->entry_bit);

	if (fcu_rx65n_blank_check(dev, offset, len) != len) {
		LOG_ERR("Attempting to write on previously written flash");
		fcu_rx65n_disable_pe();
		k_sem_give(&data->access_sem);
		return -EINVAL;
	}

	LOG_DBG("Blank check finished successfully.");

	reset_flash_events();

	for (size_t pos = 0; pos < len; pos += cfg->parameters->write_block_size) {
		*fcu_cfg.fsaddr = offset + cfg->read_offset;
		LOG_DBG("Writing %lu byte block to offset %x:",
			cfg->parameters->write_block_size, (uint32_t)(offset + cfg->read_offset));
		LOG_HEXDUMP_DBG(&buf[pos], cfg->parameters->write_block_size, "");
		fcu_rx65n_write_block(&buf[pos], cfg->parameters->write_block_size);
		offset += cfg->parameters->write_block_size;
		ret = fcu_rx65n_wait_ready_bgo(K_MSEC(cfg->program_timeout_ms));
		if (ret != 0) {
			break;
		}
	}

	fcu_rx65n_disable_pe();

	k_sem_give(&data->access_sem);

	return ret;
}

/**
 * @brief erase data in the flash (API function)
 *
 * @param dev		flash device driver
 * @param offset	position in flash to start erasing (has to align to page
 *			start)
 * @param size		number of bytes to erase (has to be multiple of page size)
 *
 * @return		0 on success or negative error code
 */
static int flash_rx65n_erase(const struct device *dev, off_t offset,
			       size_t size)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);
	struct flash_pages_info first_page_info, last_page_info, page_info;
	int ret = 0;

	if (size == 0) {
		return 0;
	}

	/* get first and last page to erase and ensure they are aligned with
	 * offset and size
	 */
	if (flash_get_page_info_by_offs(dev, offset, &first_page_info) != 0
		|| offset != first_page_info.start_offset) {
		return -EINVAL;
	}

	if (flash_get_page_info_by_offs(dev, offset + size - 1, &last_page_info) != 0
		|| offset + size != last_page_info.start_offset + last_page_info.size) {
		return -EINVAL;
	}

	k_sem_take(&data->access_sem, K_FOREVER);

	while (!(*fcu_cfg.fstatr & BIT(15))) {
		;
	}

	fcu_rx65n_enable_pe(cfg->entry_bit);

	*fcu_cfg.fcpsr = 1;

	for (uint32_t page_index = first_page_info.index;
			page_index <= last_page_info.index; page_index++) {
		flash_get_page_info_by_idx(dev, page_index, &page_info);
		if (fcu_rx65n_blank_check(dev, page_info.start_offset, page_info.size)
				== page_info.size) {
			/* page is already blank */
			continue;
		}
		reset_flash_events();
		*fcu_cfg.fsaddr = page_info.start_offset + cfg->read_offset;
		*fcu_cfg.faci = FACI_CMD_BLOCK_ERASE;
		*fcu_cfg.faci = FACI_CMD_FINAL;
		ret = fcu_rx65n_wait_ready_bgo(K_MSEC(cfg->erase_timeout_ms));
		if (ret != 0) {
			break;
		}
	}

	*fcu_cfg.fcpsr = 0;

	fcu_rx65n_disable_pe();

	k_sem_give(&data->access_sem);

	return ret;
}

/**
 * @brief get the parameters (block size and erase value) of the flash (API function)
 *
 * @param dev	flash device driver
 *
 * @return	pointer to a flash_parameters structure
 */
static const struct flash_parameters *
flash_rx65n_get_parameters(const struct device *dev)
{
	DEV_CFG(cfg, dev);

	return cfg->parameters;
}

/**
 * @brief get the page layout of the flash (API function)
 *
 * @param dev		flash device driver
 * @param layout	pointer that will point to layout array after call
 * @param layout_size	number of entries in the layout array
 */
static void rx65n_page_layout(const struct device *dev,
				       const struct flash_pages_layout **layout,
				       size_t *layout_size)
{
	DEV_CFG(cfg, dev);
	*layout = cfg->layout;
	*layout_size = cfg->layout_size;
}

/**
 * @brief initialize a flash device
 *
 * @param dev	flash device driver
 *
 * @return	always 0 (success)
 */
static int flash_rx65n_init(const struct device *dev)
{
	DEV_DATA(data, dev);

	k_sem_init(&data->access_sem, 1, 1);

	return 0;
}

/**
 * @brief initialize the FCU device
 *
 * @param dev	FCU device driver
 *
 * @return always 0 (success)
 *
 * @note the FCU driver has no API and is not supposed to be used anywhere but
 *       initializes interrupts and a semaphore required by all drivers for the
 *       internal flash
 */
static int flash_rx65n_fcu_init(const struct device *dev)
{
	k_sem_init(&fcu_sem, 1, 1);
	IRQ_CONNECT(DT_IRQ_BY_NAME(FCU_NODE, fiferr, irq),
		DT_IRQ_BY_NAME(FCU_NODE, fiferr, priority),
		fcu_rx65n_flash_error_isr, NULL,
		DT_IRQ_BY_NAME(FCU_NODE, fiferr, flags));
	irq_enable(DT_IRQ_BY_NAME(FCU_NODE, fiferr, irq));

	IRQ_CONNECT(DT_IRQ_BY_NAME(FCU_NODE, frdyi, irq),
		DT_IRQ_BY_NAME(FCU_NODE, frdyi, priority),
		fcu_rx65n_flash_ready_isr, NULL,
		DT_IRQ_BY_NAME(FCU_NODE, frdyi, flags));
	irq_enable(DT_IRQ_BY_NAME(FCU_NODE, frdyi, irq));

	return 0;
}

static const struct flash_driver_api flash_rx65n_api = {
	.read = flash_rx65n_read,
	.write = flash_rx65n_write,
	.erase = flash_rx65n_erase,
	.get_parameters = flash_rx65n_get_parameters,
	.page_layout = rx65n_page_layout,
};

#define FLASH_RX65N_INIT(id) \
	static const struct flash_parameters parameters_##id = { \
		.write_block_size = DT_PROP(id, write_block_size), \
		.erase_value = 0xff, \
	}; \
	static const struct flash_pages_layout layout_##id[DT_PROP_LEN(id, page_counts)] = { \
		{ \
			.pages_count = DT_PROP_BY_IDX(id, page_counts, 0), \
			.pages_size = DT_PROP_BY_IDX(id, page_sizes, 0), \
		}, \
		IF_ENABLED(DT_PROP_HAS_IDX(id, page_counts, 1), \
		({ \
			.pages_count = DT_PROP_BY_IDX(id, page_counts, 1), \
			.pages_size = DT_PROP_BY_IDX(id, page_sizes, 1), \
		})) \
	}; \
	static struct flash_rx65n_cfg cfg_##id = { \
		.read_offset = DT_REG_ADDR_BY_NAME(id, OFFSET), \
		.size = DT_REG_SIZE_BY_NAME(id, OFFSET), \
		.entry_bit = DT_PROP(id, entry_bit), \
		.parameters = &parameters_##id, \
		.layout = layout_##id, \
		.layout_size = DT_PROP_LEN(id, page_counts),\
		.program_timeout_ms = DT_PROP(id, program_timeout_ms), \
		.erase_timeout_ms = DT_PROP(id, erase_timeout_ms), \
		.erased_undefined = DT_PROP_OR(id, erased_undefined, false), \
	}; \
	static struct flash_rx65n_data data_##id; \
	DEVICE_DT_DEFINE(id, flash_rx65n_init, NULL, &data_##id,\
		&cfg_##id, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
		&flash_rx65n_api);

DT_FOREACH_CHILD_STATUS_OKAY(FCU_NODE, FLASH_RX65N_INIT);

/* define the FCU device just to run the init. */
DEVICE_DT_DEFINE(FCU_NODE, flash_rx65n_fcu_init, NULL, NULL, NULL, POST_KERNEL,
	CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);
