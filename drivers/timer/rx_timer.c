/*
 * Copyright (c) 2017 Intel Corporation
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <device.h>
#include <irq.h>
#include <sys_clock.h>
#include <spinlock.h>
#include <drivers/timer/system_timer.h>

#define RXV2_TIMER_IRQ 2 // check if: TODO: redefine RXV2_TIMER_IRQ, as it currently serves as an enabler

static uint32_t accumulated_cycle_count = 0;
static int32_t _sys_idle_elapsed_ticks = 1;

static void rxv2_timer_irq_handler(void *unused){
	ARG_UNUSED(unused);

	accumulated_cycle_count += k_ticks_to_cyc_floor32(1);

	int key = irq_lock();

	z_clock_announce(_sys_idle_elapsed_ticks);

	irq_unlock(key);
}

int z_clock_driver_init(const struct device *device){
	ARG_UNUSED(device);

	IRQ_CONNECT(RXV2_TIMER_IRQ, 0, rxv2_timer_irq_handler, NULL, 0); // check if: ARCH_IRQ_CONNECT can be used, too
	irq_enable(RXV2_TIMER_IRQ);
	return 0;
}

uint32_t z_clock_elapsed(void){
	return 0;
}

uint32_t z_timer_cycle_get_32(void){
	return accumulated_cycle_count;
}
