/*
 * Copyright (c) 2019 Stephanos Ioannidis <root@stephanos.io>
 * Copyright (c) 2020 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Driver for RXV2 Interrupt Controller
 *
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_RXV2_IRQ_H_
#define ZEPHYR_INCLUDE_DRIVERS_RXV2_IRQ_H_

#ifndef _ASMLANGUAGE

#include <zephyr/types.h>
#include <device.h>

#define	IRQ_DISTANCE_BASE			0x0 // check if: TODO: verify CPU-specific addresses
#define IRQ_CPU_BASE				0x0 // check if: TODO: verify CPU-specific addresses
#define	IRQ_CONFIG_REGs				(IRQ_DISTANCE_BASE + 0xc00) // check if: TODO: verify CPU-specific addresses

/* IRQ_CONFIG_REG */
#define IRQ_CONFIG_REG_MASK			BIT_MASK(2)
#define IRQ_CONFIG_REG_TYPE			BIT(1)

/*
 * GIC Distributor Interface
 */

/*
 * 0x000  Distributor Control Register
 */
#define	DISTR_CTRL_REG		(IRQ_DISTANCE_BASE +   0x0) // check if: TODO: verify CPU-specific addresses


/*
 * 0x000C  Interrupt Acknowledge Register
 */
#define IRQ_ACKN_REG				(IRQ_CPU_BASE +    0xc) // check if: TODO: verify CPU-specific addresses

/*
 * 0x0010  End of Interrupt Register
 */
#define EO_IRQ_REG					(IRQ_CPU_BASE +   0x10) // check if: TODO: verify CPU-specific addresses

/*
 * 0x004  Interrupt Controller Type Register
 */
#define	IRQ_CTRL_TYPE_REG		(IRQ_DISTANCE_BASE +   0x4) // check if: TODO: verify CPU-specific addresses

/*
 * 0x800  Interrupt Processor Targets Registers
 */
#define	IRQ_CPU_TARGET_REG		(IRQ_DISTANCE_BASE + 0x800) // check if: TODO: verify CPU-specific addresses

/*
 * 0xC00  Interrupt Configuration Registers
 */
#define	IRQ_CONFIG_REG		(IRQ_DISTANCE_BASE + 0xc00) // check if: TODO: verify CPU-specific addresses

/*
 * 0x080  Interrupt Group Registers
 */
#define	IRQ_GROUP_REG		(IRQ_DISTANCE_BASE +  0x80) // check if: TODO: verify CPU-specific addresses

/*
 * 0x180  Interrupt Clear-Enable Registers
 */
#define	IRQ_CLEAR_ENABLE_REG		(IRQ_DISTANCE_BASE + 0x180) // check if: TODO: verify CPU-specific addresses

/*
 * 0x400  Interrupt Priority Registers
 */
#define	IRQ_PRIO_REG	(IRQ_DISTANCE_BASE + 0x400) // check if: TODO: verify CPU-specific addresses

/*
 * 0x100  Interrupt Set-Enable Reigsters
 */
#define	IRQ_SET_ENABLE_REG		(IRQ_DISTANCE_BASE + 0x100) // check if: TODO: verify CPU-specific addresses

/*
 * 0x0004  IRQ_PRIO_MASK_REG
 */
#define IRQ_PRIO_MASK_REG		(IRQ_DISTANCE_BASE +    0x4) // check if: TODO: verify CPU-specific addresses

/*
 * 0x0000  CPU Interface Control Register
 */
#define CPU_INTERFACE_CTRL_REG		(IRQ_DISTANCE_BASE +    0x0) // check if: TODO: verify CPU-specific addresses

#define IRQ_SPI_INT_BASE		32

/* IRQ CTLR */
#define IRQ_CTLR_ENABLE_GROUP_0	BIT(0)
#define IRQ_CTLR_ENABLE_GROUP_1	BIT(1)

#define IRQ_CTLR_ENABLE_MASK	(IRQ_CTLR_ENABLE_GROUP_0 | IRQ_CTLR_ENABLE_GROUP_1)

/**
 * @brief Enable interrupt
 *
 * @param irq interrupt ID
 */
void rxv2_irq_enable(unsigned int irq);

/**
 * @brief Disable interrupt
 *
 * @param irq interrupt ID
 */
void rxv2_irq_disable(unsigned int irq);

/**
 * @brief Check if an interrupt is enabled
 *
 * @param irq interrupt ID
 * @return Returns true if interrupt is enabled, false otherwise
 */
bool rxv2_irq_is_enabled(unsigned int irq);

/**
 * @brief Set interrupt priority
 *
 * @param irq interrupt ID
 * @param prio interrupt priority
 * @param flags interrupt flags
 */
void rxv2_irq_set_priority(unsigned int irq, unsigned int prio, unsigned int flags);

/**
 * @brief Get active interrupt ID
 *
 * @return Returns the ID of an active interrupt
 */
unsigned int rxv2_irq_get_active(void);

/**
 * @brief Signal end-of-interrupt
 *
 * @param irq interrupt ID
 */
void rxv2_irq_eo_irq(unsigned int irq);

#endif /* !_ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_DRIVERS_RXV2_IRQ_H_ */
