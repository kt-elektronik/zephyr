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

#define RX_GPIO_PORT_PINS 8


#define SYSTEM_PRCR (*(uint16_t*)0x000803FE)
#define MPC_PWPR    (*(uint8_t*)0x0008C11F)
#define REGISTER_WRITE_ENABLE(val) (SYSTEM_PRCR = 0xA500u | BIT(3) | BIT(1) | BIT (0))

#define ENABLE_WRITING_MPC MPC_PWPR &= ~BIT(7); MPC_PWPR |= BIT(6);
#define DISABLE_WRITING_MPC MPC_PWPR &= ~BIT(6); MPC_PWPR |= BIT(7);
 

#endif /* !_ASMLANGUAGE */

#endif /* _SOC_H_ */
