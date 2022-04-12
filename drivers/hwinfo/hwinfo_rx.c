/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Renesas RX MCU hwinfo driver
 */


#include <drivers/hwinfo.h>
#include <string.h>
#include <sys/util.h>

#define CPU_NODE_ID DT_NODELABEL(cpu0)

#define UIDR_BASE_ADDR DT_REG_ADDR(CPU_NODE_ID)
#define N_UIDR DT_REG_SIZE(CPU_NODE_ID)

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	uint32_t *uidr = (uint32_t *)UIDR_BASE_ADDR;
	size_t bytes_copied = 0;
	uint8_t idx = 0;

	while (bytes_copied < length && idx < N_UIDR) {
		uint8_t bytes_to_copy = MIN(sizeof(uint32_t), (length - bytes_copied));
		/* UIDR are supposed to be read as 32 bit values */
		uint32_t reg_value = uidr[idx++];

		memcpy(&buffer[bytes_copied], &reg_value, bytes_to_copy);

		bytes_copied += bytes_to_copy;
	}

	return bytes_copied;
}
