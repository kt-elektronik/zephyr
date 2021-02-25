/*
 * Copyright (c) 2018 Lexmark International, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __DT_BINDING_RXV2_IRQ_DT_H
#define __DT_BINDING_RXV2_IRQ_DT_H

/* CPU Interrupt numbers */
#define	GIC_INT_VIRT_MAINT		25 // check if: TODO: verify CPU-specific addresses & identifiers
#define	GIC_INT_HYP_TIMER		26 // check if: TODO: verify CPU-specific addresses & identifiers
#define	GIC_INT_VIRT_TIMER		27 // check if: TODO: verify CPU-specific addresses & identifiers
#define	GIC_INT_LEGACY_FIQ		28 // check if: TODO: verify CPU-specific addresses & identifiers
#define	GIC_INT_PHYS_TIMER		29 // check if: TODO: verify CPU-specific addresses & identifiers
#define	GIC_INT_NS_PHYS_TIMER	30 // check if: TODO: verify CPU-specific addresses & identifiers
#define	GIC_INT_LEGACY_IRQ		31 // check if: TODO: verify CPU-specific addresses & identifiers

#define	IRQ_TYPE_LEVEL		0x0 // check if: TODO: verify CPU-specific addresses
#define	IRQ_TYPE_EDGE		0x1 // check if: TODO: verify CPU-specific addresses

#define	GIC_SPI			0x0 // check if: TODO: verify CPU-specific addresses
#define	GIC_PPI			0x1 // check if: TODO: verify CPU-specific addresses

#define IRQ_DEFAULT_PRIORITY	0xa0 // check if: TODO: verify CPU-specific addresses

#endif // __DT_BINDING_RXV2_IRQ_DT_H
