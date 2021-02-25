/*
 * Copyright (c) 2019 Carlo Caione <ccaione@baylibre.com>
 * Copyright (c) 2020 KT-Elektronik, Klaucke und Partner GmbH
 * SPDX-License-Identifier: Apache-2.0
 *
 */

// adapted from qmu_cortex_a53

#ifndef _SOC_H_
#define _SOC_H_

#include <sys/util.h>

#ifndef _ASMLANGUAGE

#include <device.h>
/* Add include for DTS generated information */
#include <devicetree.h>

#ifdef CONFIG_GPIO_RX
#include <drivers/gpio.h> // check if: gpio.h, gpio.c might not be required
#endif // CONFIG_GPIO_RX

#ifdef CONFIG_PINMUX_RX
#include <drivers/pinmux.h>
#endif // CONFIG_PINMUX_RX

//#define RX_PINMUX_0_BASE_ADDR     (DT_REG_ADDR(DT_INST(0, rx_port0)))

#define RX_PINMUX_PINS 8

#endif /* !_ASMLANGUAGE */

#endif /* _SOC_H_ */
