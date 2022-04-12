/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_grp_intc

#include <soc.h>
#include <errno.h>
#include <device.h>
#include <sys/slist.h>
#include <drivers/interrupt_controller/rxv2_irq.h>

/* suppress compiler warning if not group interrupt controler is configured */
#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) > 0

struct grp_intc_rx65n_data {
	/* list of callbacks */
	sys_slist_t cb;
};

#define DEV_CFG(cfg, dev) struct grp_intc_rx65n_cfg *cfg = \
		(struct grp_intc_rx65n_cfg *)(dev->config)
#define DEV_DATA(data, dev) struct grp_intc_rx65n_data *data = \
		(struct grp_intc_rx65n_data *)(dev->data)

/**
 * @brief configuration data for a group interrupt device
 */
struct grp_intc_rx65n_cfg {
	/* address of the Group Interrupt Request Register (GRPxxx) */
	volatile uint32_t *grp;
	/* address of the Group Interrupt Request Enable Register (GENxxx)  */
	volatile uint32_t *gen;
	/* vector number of the interrupt */
	const uint8_t vect;
	/* priority of the interrupt */
	const uint8_t prio;
};

/**
 * @brief api for the group interrupt controller
 */
struct grp_intc_rx65n_api {
	/**
	 * @brief add or remove callbacks
	 *
	 * @param dev		pointer to the device structure
	 * @param callback	pointer to a callback structure
	 * @param set		should the entry be added (true) or removed (false)
	 *
	 * @return		0 on success or a negative error code
	 */
	int (*manage_callback)(const struct device *dev,
			struct grp_intc_rx65n_callback *callback, bool set);
};

int grp_intc_rx65n_manage_callback(const struct device *dev,
			struct grp_intc_rx65n_callback *callback, bool set)
{
	DEV_CFG(cfg, dev);
	DEV_DATA(data, dev);

	if (!sys_slist_is_empty(&data->cb)) {
		if (!sys_slist_find_and_remove(&data->cb, &callback->node)) {
			if (!set) {
				return -EINVAL;
			}
		}
	}

	if (set) {
		sys_slist_prepend(&data->cb, &callback->node);
	}

	/* set interrupt enable bits according to registered callbacks */
	struct grp_intc_rx65n_callback *cb, *tmp;
	uint32_t gen = 0;

	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&data->cb, cb, tmp, node) {
		gen |= cb->pin_mask;
	}

	uint32_t key = irq_lock();
	*(cfg->gen) = gen;
	irq_unlock(key);

	/* the interrupt only has to be active if at least one callback is
	 * registered for the group
	 */
	if (sys_slist_is_empty(&data->cb)) {
		irq_disable(cfg->vect);
	} else {
		irq_enable(cfg->vect);
	}

	return 0;
}

/**
 * @brief callback function for the group interrupt
 *
 * @param param		pointer to the device structure
 */
static void grp_intc_rx65n_callback(const void *param)
{
	const struct device *dev = (struct device *) param;

	DEV_DATA(data, dev);
	DEV_CFG(cfg, dev);

	struct grp_intc_rx65n_callback *cb, *tmp;

	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&data->cb, cb, tmp, node) {
		if (cb->pin_mask & *cfg->grp) {
			cb->callback(cb->param);
		}
	}
}

static int grp_intc_rx65n_init(const struct device *dev)
{
	return 0;
}

struct grp_intc_rx65n_api grp_intc_rx65n_api = {
	.manage_callback = grp_intc_rx65n_manage_callback,
};

#endif

#define GRP_INTC_DEVICE_INIT(id) \
	static struct grp_intc_rx65n_data grp_intc_rx65n_##id##_data = { \
	}; \
	static struct grp_intc_rx65n_cfg grp_intc_rx65n_##id##_cfg = { \
		.grp = (uint32_t *)DT_INST_REG_ADDR_BY_NAME(id, GRP), \
		.gen = (uint32_t *)DT_INST_REG_ADDR_BY_NAME(id, GEN), \
		.vect = DT_INST_PROP(id, vector), \
		.prio = DT_INST_PROP(id, prio) \
	}; \
	static int grp_intc_rx65n_##id##_init(const struct device *dev) \
	{ \
		IRQ_CONNECT(DT_INST_PROP(id, vector), DT_INST_PROP(id, prio), \
			grp_intc_rx65n_callback, DEVICE_DT_INST_GET(id), 0); \
		return grp_intc_rx65n_init(dev); \
	} \
	DEVICE_DT_INST_DEFINE(id, grp_intc_rx65n_##id##_init, NULL, \
			&grp_intc_rx65n_##id##_data, &grp_intc_rx65n_##id##_cfg, \
			PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			&grp_intc_rx65n_api);

DT_INST_FOREACH_STATUS_OKAY(GRP_INTC_DEVICE_INIT);
