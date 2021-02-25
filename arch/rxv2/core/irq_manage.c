/*
 * Copyright (c) 2019 Oticon A/S
 * Copyright (c) 2020 KT-Elektronik, Klaucke und Partner GmbH
 *
 * Note: For dynamic interrupts use CONFIG_DYNAMIC_INTERRUPTS and int arch_irq_connect_dynamic()
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <irq.h>
#include <drivers/interrupt_controller/rxv2_irq.h>

#ifdef CONFIG_IRQ_OFFLOAD
#include <irq_offload.h>

void arch_irq_offload(irq_offload_routine_t routine, void *parameter){
	//posix_irq_offload(routine, parameter);
}

#endif // CONFIG_IRQ_OFFLOAD

void arch_irq_enable(unsigned int irq){
	unsigned int key;

	key = irq_lock();

	rxv2_irq_enable(irq);

	irq_unlock(key);
}

void arch_irq_disable(unsigned int irq){
	unsigned int key;

	key = irq_lock();

	rxv2_irq_disable(irq);

	irq_unlock(key);
}

int arch_irq_is_enabled(unsigned int irq){

	return rxv2_irq_is_enabled(irq);
}
