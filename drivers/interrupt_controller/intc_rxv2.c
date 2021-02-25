/*
 * Copyright (c) 2018 Marvell
 * Copyright (c) 2018 Lexmark International, Inc.
 * Copyright (c) 2019 Stephanos Ioannidis <root@stephanos.io>
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/__assert.h>
#include <sw_isr_table.h>
#include <dt-bindings/interrupt-controller/rxv2_irq_dt.h>
#include <drivers/interrupt_controller/rxv2_irq.h>

void rxv2_irq_enable(unsigned int irq){

	int int_grp = irq / 32;
	int int_off = irq % 32;

	sys_write32((1 << int_off), (IRQ_SET_ENABLE_REG + int_grp * 4));
}

void rxv2_irq_disable(unsigned int irq){

	int int_grp = irq / 32;
	int int_off = irq % 32;

	sys_write32((1 << int_off), (IRQ_SET_ENABLE_REG + int_grp * 4));
}

bool rxv2_irq_is_enabled(unsigned int irq){

	int int_grp = irq / 32;
	int int_off = irq % 32;
	unsigned int enabler = sys_read32(IRQ_SET_ENABLE_REG + int_grp * 4);

	return (enabler & (1 << int_off)) != 0;
}

void rxv2_irq_set_priority(unsigned int irq, unsigned int prio, uint32_t flags){
	/* Set interrupt type */	
	int int_grp = (irq / 16) * 4;
	int int_off = (irq % 16) * 2;
	uint32_t val = sys_read32(IRQ_CONFIG_REGs + int_grp);

	/* Set priority */
	sys_write8(prio & 0xff, IRQ_CONFIG_REGs + irq); // check if: TODO: verify CPU-specific addresses

	val &= ~(IRQ_CONFIG_REG_MASK << int_off);
	if (flags & IRQ_TYPE_EDGE) {
		val |= (IRQ_CONFIG_REG_TYPE << int_off); // check if: TODO: verify logic
	}

	sys_write32(val, IRQ_CONFIG_REGs + int_grp);
}

unsigned int rxv2_irq_get_active(void){
	return (sys_read32(IRQ_ACKN_REG) & 0x3ff); // check if: TODO: verify CPU-specific addresses
}

void rxv2_irq_eo_irq(unsigned int irq){
	/*
	 * Ensure the write to peripheral registers are *complete* before the write
	 * to GIC_EOIR.
	 *
	 * Note: The completion gurantee depends on various factors of system design
	 * and the barrier is the best core can do by which execution of further
	 * instructions waits till the barrier is alive.
	 */
	__DSB();

	/* set to inactive */
	sys_write32(irq, EO_IRQ_REG);
}

static void rxv2_irq_distributor_interface_init(void){
	unsigned int rxv2_irqs = sys_read32(IRQ_CTRL_TYPE_REG) & 0x1f; // check if: TODO: verify CPU-specific addresses
	unsigned int i = 0;

	rxv2_irqs = (rxv2_irqs + 1) * 32;

	if (rxv2_irqs > 1020) {
		rxv2_irqs = 1020; // check if: TODO: verify CPU-specific addresses
	}

	// Disable the forwarding of pending interrupts from the Distributor to the CPU interfaces
	sys_write32(0, DISTR_CTRL_REG);

	// Set all global interrupts to this CPU only.
	for (i = IRQ_SPI_INT_BASE; i < rxv2_irqs; i += 4) {
		sys_write32(0x01010101, IRQ_CPU_TARGET_REG + i); // check if: TODO: verify CPU-specific addresses
	}

	// Set all global interrupts to be level triggered, active low.
	for (i = IRQ_SPI_INT_BASE; i < rxv2_irqs; i += 16) {
		sys_write32(0, IRQ_CONFIG_REG + i / 4);
	}

	//  Set priority on all global interrupts.
	for (i = IRQ_SPI_INT_BASE; i < rxv2_irqs; i += 4) {
		sys_write32(0, IRQ_PRIO_REG + i); // check if: TODO: verify CPU-specific addresses
	}

	// Set all interrupts to group 0
	for (i = IRQ_SPI_INT_BASE; i < rxv2_irqs; i += 32) {
		sys_write32(0, IRQ_GROUP_REG + i / 8); // check if: TODO: verify CPU-specific addresses
	}

	// Disable all interrupts.
	// Leave the PPI and SGIs alone as these enables are banked registers.

	for (i = IRQ_SPI_INT_BASE; i < rxv2_irqs; i += 32) {
		sys_write32(0xffffffff, IRQ_CLEAR_ENABLE_REG + i / 8); // check if: TODO: verify CPU-specific addresses
	}

	// Enable the forwarding of pending interrupts from the Distributor to the CPU interfaces
	sys_write32(1, DISTR_CTRL_REG);
}

static void rxv2_cpu_init(void){
	int i = 0;
	uint32_t val = 0;

	/*
	 * Deal with the banked PPI and SGI interrupts - disable all
	 * PPI interrupts, ensure all SGI interrupts are enabled.
	 */
	sys_write32(0xffff0000, IRQ_CLEAR_ENABLE_REG); // check if: TODO: verify CPU-specific addresses
	sys_write32(0x0000ffff, IRQ_SET_ENABLE_REG); // check if: TODO: verify CPU-specific addresses

	/*
	 * Set priority on PPI and SGI interrupts
	 */
	for (i = 0; i < 32; i += 4) {
		sys_write32(0xa0a0a0a0, IRQ_PRIO_REG + i); // check if: TODO: verify CPU-specific addresses
	}

	sys_write32(0xf0, IRQ_PRIO_MASK_REG);

	/*
	 * Enable interrupts and signal them using the IRQ signal.
	 */
	val = sys_read32(IRQ_CLEAR_ENABLE_REG);
	val |= IRQ_CTLR_ENABLE_MASK;
	sys_write32(val, IRQ_CLEAR_ENABLE_REG);
}

/**
 *
 * @brief Initialize the GIC device driver
 *
 *
 * @return N/A
 */
#define RXV2_PARENT_IRQ 0
#define RXV2_PARENT_IRQ_PRI 0
#define RXV2_PARENT_IRQ_FLAGS 0


int rxv2_irq_init(const struct device *unused){

	ARG_UNUSED(unused);

	// Init of Distributor interface registers
	rxv2_irq_distributor_interface_init();

	// Init CPU interface registers
	rxv2_cpu_init();

	return 0;
}

SYS_INIT(rxv2_irq_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);