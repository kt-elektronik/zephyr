/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief common SOC header for the Renesas RX SOC family
 */

#ifndef _SOC_RX_H_
#define _SOC_RX_H_

#include <sys/util.h>

#ifndef _ASMLANGUAGE

#include <device.h>
#include <devicetree.h>

int rx_pin_init(const struct device *pinmux,
	const struct device *gpio, uint8_t pin, uint32_t pinmux_func,
	uint32_t gpio_flags);

/**
 * @brief initialize one pin for a RX device using dts configuration
 *
 * @param id	node id of the node - the node must have phandle-array properties gpios and pinmuxs
 * @param prop	property id of the phandle array (unused)
 * @param idx	element index of the pinmuxs and gpios phandle arrays to use
 */
#define RX_INIT_PIN(id, prop, idx) \
	if (ret == 0) { \
		ret = rx_pin_init( \
			DEVICE_DT_GET(DT_PHANDLE_BY_IDX(id, pinmuxs, idx)), \
			DEVICE_DT_GET(DT_PHANDLE_BY_IDX(id, gpios, idx)), \
			DT_PHA_BY_IDX(id, pinmuxs, idx, pin), \
			DT_PHA_BY_IDX(id, pinmuxs, idx, function), \
			DT_PHA_BY_IDX(id, gpios, idx, flags));\
	}

#endif /* !_ASMLANGUAGE */

#endif /* _SOC_RX_H_ */
