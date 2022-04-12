/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_RX65N_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_RX65N_H_

/**
 * @brief helper-defines for the dma slots of the rx65n DMAC
 */
#define DMA_SLOT_RSPI0_SPRI0  0
#define DMA_SLOT_RSPI0_SPTI0  1
#define DMA_SLOT_RSPI1_SPRI1  2
#define DMA_SLOT_RSPI1_SPTI1  3
#define DMA_SLOT_QSPI_SPRI    4
#define DMA_SLOT_QSPI_SPTI    5

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_RX65N_H_ */
