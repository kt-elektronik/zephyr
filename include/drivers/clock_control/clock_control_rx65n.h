/*
 * Clock configuration for RX65N series.
 * Main clock, HOCO, LOCO, sub clock and PLL are implemented.
 *
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CLOCK_CONTROL_RX65N_H_
#define ZEPHYR_DRIVERS_CLOCK_CONTROL_RX65N_H_

#include <devicetree.h>


/**
 * @brief get a clock sub-system structure for an RX65N sub-clock from the device tree
 *
 * @param node_id	node identifier of the device
 * @param pha		phandle-array property describing the sub-clock
 * @return		a subsystem struct const that can be assigned to a
 *			struct clock_control_rx65n_subsys variable
 */
#define DT_CLOCK_RX65N_SUB_SYSTEM(node_id, pha) \
	{ \
		.mstpcr = (uint32_t *)DT_REG_ADDR(DT_PHANDLE(node_id, pha)), \
		.stop_bit = DT_PHA(node_id, pha, stop_bit), \
	}

#define DT_INST_CLOCK_RX65N_SUB_SYSTEM(inst, pha) DT_CLOCK_RX65N_SUB_SYSTEM(DT_DRV_INST(inst), pha)

struct clock_control_rx65n_subsys {
	volatile uint32_t *mstpcr;
	uint8_t stop_bit;
};

#endif
