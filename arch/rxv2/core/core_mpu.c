/*
 * Copyright (c) 2017 Synopsys.
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <init.h>
#include <kernel.h>
#include <soc.h>
#include <arch/rxv2/rxv2_core_mpu.h>
#include <kernel_structs.h>

/*
 * @brief Configure MPU for the thread
 *
 * This function configures per thread memory map reprogramming the MPU.
 *
 * @param thread thread info data structure.
 */
void configure_mpu_thread(struct k_thread *thread){
	rxv2_core_mpu_disable();
	rxv2_core_mpu_configure_thread(thread);
	rxv2_core_mpu_enable();
}

#if defined(CONFIG_USERSPACE)

int rxv2_mem_domain_max_partitions_get(void){
	return rxv2_core_mpu_get_max_domain_partition_regions();
}

/*
 * Validate the given buffer is user accessible or not
 */
int rxv2_buffer_validate(void *addr, size_t size, int write){
	return rxv2_core_mpu_buffer_validate(addr, size, write);
}

#endif