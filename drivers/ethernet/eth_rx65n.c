/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Renesas RX65N Ethernet driver.
 *
 * This driver supplies the api to the Renesas RX65N ethernet interface
 * via the Renesas Ethernet FIT module.
 */

#define DT_DRV_COMPAT renesas_rx65n_ethernet

#define LOG_MODULE_NAME eth_rx65n
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_ETHERNET_LOG_LEVEL);

#include <soc.h>
#include <sys/util.h>
#include <net/net_if.h>
#include <net/ethernet.h>
#include <net/ethernet_mgmt.h>
#include <errno.h>
#include <drivers/pinmux.h>
#include <drivers/gpio.h>
#include <drivers/interrupt_controller/rxv2_irq.h>
#include "eth.h"

/* Renesas ethernet FIT module */
#include <r_ether_rx_if.h>

/* Ethernet Controller Status Register (ECSR) bits */
#define ETH_RX65N_ECXR_LCHNG BIT(2) /* Link Signal Change Flag */

/* ETHERC/EDMAC Status Register (EESR) bits */
#define ETH_RX65N_EESR_FR BIT(18)  /* frame received */
#define ETH_RX65N_EESR_TC BIT(21)  /*  indicating a completed transfer */

/* interval between link status checks if no link status signal is used */
#define ETH_RX65N_LINK_CHECK_INTERVAL K_MSEC(100)

#define DEV_CFG(cfg, dev) const struct eth_rx65n_dev_cfg *cfg = \
				(const struct eth_rx65n_dev_cfg *)(dev->config)
#define DEV_DATA(data, dev) struct eth_rx65n_dev_data *data = \
				(struct eth_rx65n_dev_data *)(dev->data)

/* RX65N processors has only one channel (0), but Renesas FIT modules are
 * written for other processors as well, which can have more than one channel,
 * so channel 0 has to be selected when calling FIT module functions
 */
#define ETH_RX65N_CHANNEL 0

#define ETH0_NODE_ID DT_NODELABEL(eth0)

struct eth_rx65n_dev_data {
	/* semaphore protecting the tx buffer of the r_eth_rx driver */
	struct k_sem tx_buf_sem;
	/* network interface to use */
	struct net_if *iface;
#if (ETHER_CFG_USE_LINKSTA == 1)
	/* work queue entry for link state change interrupts */
	struct k_work link_work;
#else
	/* without the link status interrupt, link status has to be actively
	 * checked regularly
	 */
	struct k_work_delayable link_work;
#endif
	/* work queue entry for frame reception interrupts */
	struct k_work rx_work;
	/* mac address */
	uint8_t mac_addr[6];
	/* use promiscuous mode */
	bool promiscuous;
	/* has the ethernet device been started (and not stopped) ? */
	bool started;
};

struct eth_rx65n_dev_cfg {
	uint8_t n_pinmuxs;
	uint8_t n_gpios;
	const struct device *intc;
};

static struct eth_rx65n_dev_data data = {
	.mac_addr = DT_PROP_OR(ETH0_NODE_ID, local_mac_address, {0}),
	.promiscuous = true,
};
static struct eth_rx65n_dev_cfg cfg = {
	.n_pinmuxs = DT_PROP_LEN(ETH0_NODE_ID, pinmuxs),
	.n_gpios = DT_PROP_LEN(ETH0_NODE_ID, gpios),
	.intc = DEVICE_DT_GET(DT_PHANDLE(ETH0_NODE_ID, interrupt_parent)),
};

/**
 * @brief work queue function to handle link state change interrupts
 *
 * @param item    (unused) the work item
 */
static void eth_rx65n_link_work(struct k_work *item)
{
	ARG_UNUSED(item);

	R_ETHER_LinkProcess(ETH_RX65N_CHANNEL);

#if (ETHER_CFG_USE_LINKSTA == 0)
	struct eth_rx65n_dev_data *data =
		CONTAINER_OF(item, struct eth_rx65n_dev_data, link_work);

	k_work_reschedule(&data->link_work, ETH_RX65N_LINK_CHECK_INTERVAL);
#endif
}

/**
 * @brief work queue function to handle frame received interrupts
 *
 * @param item    work queue item
 */
static void eth_rx65n_rx_work(struct k_work *item)
{
	struct eth_rx65n_dev_data *data =
		CONTAINER_OF(item, struct eth_rx65n_dev_data, rx_work);

	uint8_t *rx_buffer;
	struct net_pkt *pkt;

	int32_t bytes_received = R_ETHER_Read_ZC2(ETH_RX65N_CHANNEL, (void **)&rx_buffer);

	if (bytes_received < 1) {
		/* receive buffer is empty - nothing to do */
		R_ETHER_Read_ZC2_BufRelease(ETH_RX65N_CHANNEL);
		return;
	}

	LOG_DBG("Received %u bytes of data", bytes_received);

	if (data->iface == NULL) {
		/* no network interface to hand of data to, so just
		 * drop the received frame to clear the receive buffer
		 */
		R_ETHER_Read_ZC2_BufRelease(ETH_RX65N_CHANNEL);
		return;
	}

	pkt = net_pkt_rx_alloc_with_buffer(data->iface, bytes_received,
					   AF_UNSPEC, 0, K_NO_WAIT);
	if (!pkt) {
		/* failed to allocate a packet buffer in the stack,
		 * drop frame
		 */
		R_ETHER_Read_ZC2_BufRelease(ETH_RX65N_CHANNEL);
		return;
	}

	if (net_pkt_write(pkt, rx_buffer, bytes_received)) {
		LOG_ERR("Unable to write frame into the pkt");
		R_ETHER_Read_ZC2_BufRelease(ETH_RX65N_CHANNEL);
		return;
	}

	/* the content of the frame has been copied to pkt, the receive
	 * buffer can be released
	 */
	R_ETHER_Read_ZC2_BufRelease(ETH_RX65N_CHANNEL);

	if (net_recv_data(data->iface, pkt) < 0) {
		net_pkt_unref(pkt);
		return;
	}
}

/**
 * @brief interrupt handled function for ethernet interrupt events
 *
 * @param arg	pointer to a ether_cb_arg_t structure with the content
 *		of the ECSR and EESR registers
 */
static void eth_rx65n_int_handler(void *arg)
{
	/* The Renesas FIT module r_ether_rx does not allow custom callback
	 * arguments, so the global driver data structures are used. This works
	 * since RX65N only supports one ethernet channel.
	 */
	ether_cb_arg_t *cb_arg = (ether_cb_arg_t *) arg;

	if (cb_arg->status_eesr & ETH_RX65N_EESR_FR) {
		/* a frame has been received */
		LOG_DBG("interrupt: frame received");
		k_work_submit(&data.rx_work);
	}
	if (cb_arg->status_eesr & ETH_RX65N_EESR_TC) {
		/* TX buffer has been transmitted and is now free to send again */
		LOG_DBG("interrupt: frame transmitted");
		k_sem_give(&data.tx_buf_sem);
	}
#if (ETHER_CFG_USE_LINKSTA == 1)
	if (cb_arg->status_ecsr & ETH_RX65N_ECXR_LCHNG) {
		LOG_DBG("interrupt: link established");
		k_work_submit(&data.link_work);
	}
#endif
}

/**
 * @brief callback function for ethernet events
 *
 * @param pparam	pointer to a ether_cb_arg_t struct containing
 *			information about the event
 */
static void eth_rx65n_callback(void *pparam)
{
	ether_cb_arg_t *pdecode;
	uint32_t channel;

	pdecode = (ether_cb_arg_t *)pparam;
	channel = pdecode->channel;  /* Get Ethernet channel number */

	__ASSERT(channel == ETH_RX65N_CHANNEL, "wrong channel");

	switch (pdecode->event_id) {
	case ETHER_CB_EVENT_ID_LINK_ON:
		if (data.iface != NULL) {
			LOG_INF("Ethernet Link established.");
			net_eth_carrier_on(data.iface);
		}
		break;
	case ETHER_CB_EVENT_ID_LINK_OFF:
		if (data.iface != NULL) {
			LOG_INF("Ethernet Link lost.");
			net_eth_carrier_off(data.iface);
			/* reset ethernet driver to make sure the connection
			 * will be re-established for example if a cable is
			 * reconnected.
			 */
			R_ETHER_Close_ZC2(ETH_RX65N_CHANNEL);
			R_ETHER_Open_ZC2(ETH_RX65N_CHANNEL, data.mac_addr, 0);
		}
		break;
	default:
		/* the only other defined value is ETHER_CB_EVENT_WAKEON_LAN,
		 * which is currently not implemented
		 */
		LOG_INF("Unknown event id: %u", pdecode->event_id);
		break;
	}

}

/**
 * @brief open a new ethernet connection
 *
 * @param dev	ethernet device driver structure
 *
 * @returns	0 on success, negative value (Renesas FIT module error codes)
 */
int eth_rx65n_start(const struct device *dev)
{
	DEV_DATA(data, dev);
	int ret;

	LOG_INF("Starting Ethernet.");

	if (data->started) {
		LOG_INF("%s: ethernet already started", __func__);
		return 0;
	}

	ret = R_ETHER_Open_ZC2(ETH_RX65N_CHANNEL, data->mac_addr, 0);
	if (ret != ETHER_SUCCESS) {
		LOG_ERR("R_ETHER_Open_ZC2 returns error %d", ret);
		return ret;
	}
	#if (ETHER_CFG_USE_LINKSTA == 0)
		k_work_schedule(&data->link_work, ETH_RX65N_LINK_CHECK_INTERVAL);
	#endif

	data->started = true;

	return 0;
}

/**
 * @brief stop the ethernet device (API function)
 *
 * @param dev	ethernet device driver structure
 *
 * @returns	0 on success, negative value (Renesas FIT module error codes)
 */
static int eth_rx65n_stop(const struct device *dev)
{
	DEV_DATA(data, dev);
	int ret;

	LOG_INF("Stopping Ethernet");

	if (!data->started) {
		LOG_INF("%s: ethernet already stopped.", __func__);
		return 0;
	}

	ret = R_ETHER_Close_ZC2(ETH_RX65N_CHANNEL);
	if (ret != ETHER_SUCCESS) {
		LOG_ERR("R_ETHER_Close_ZC2 returns error %d", ret);
		return ret;
	}
#if (ETHER_CFG_USE_LINKSTA == 0)
	k_work_cancel_delayable(&data->link_work);
#endif

	data->started = false;
	return 0;
}

/**
 * @brief initializes the ethernet device driver
 *
 * @param dev	ethernet device driver structure
 *
 * @returns	0 on success or negative error code
 */
static int eth_rx65n_init(const struct device *dev)
{
	DEV_DATA(data, dev);
	DEV_CFG(cfg, dev);
	int ret = 0;
	ether_param_t param;
	ether_cb_t cb_func;
	bool valid_mac = false;

	/* cfg is only used in __ASSERTs, which depend on project configuration */
	ARG_UNUSED(cfg);

#if (ETHER_CFG_USE_LINKSTA == 1)
	k_work_init(&data->link_work, eth_rx65n_link_work);
#else
	k_work_init_delayable(&data->link_work, eth_rx65n_link_work);
#endif
	k_work_init(&data->rx_work, eth_rx65n_rx_work);

	k_sem_init(&data->tx_buf_sem, 1, 1);

	__ASSERT(cfg->n_pinmuxs == cfg->n_gpios,
		 "defined %u pinmusx and %u gpios for ethernet device",
		 cfg->n_pinmuxs, cfg->n_gpios);

	DT_FOREACH_PROP_ELEM(ETH0_NODE_ID, pinmuxs, RX_INIT_PIN);

	if (ret == 0) {
		R_ETHER_Initial();

		cb_func.pcb_func = &eth_rx65n_callback;
		param.ether_callback = cb_func;
		ret = R_ETHER_Control(CONTROL_SET_CALLBACK, param);
	}

	if (ret == 0) {
		cb_func.pcb_int_hnd = &eth_rx65n_int_handler;
		param.ether_callback = cb_func;
		ret = R_ETHER_Control(CONTROL_SET_INT_HANDLER, param);
	}

	if (ret == 0) {
		param.channel = ETHER_CHANNEL_0;
		ret = R_ETHER_Control(CONTROL_POWER_ON, param);
	}

	#if DT_NODE_HAS_PROP(ETH0_NODE_ID, stored_mac_address)
		uint8_t *stored_mac = (uint8_t *)DT_PROP(ETH0_NODE_ID, stored_mac_address);
		uint16_t mac_sum = 0;

		valid_mac = true;

		for (uint8_t i = 0; i < 6; i++) {
			mac_sum += stored_mac[i];
			if (stored_mac[i] != stored_mac[i + 6]) {
				valid_mac = false;
			}
		}
		if (mac_sum == 0 || mac_sum != *(uint16_t *)&stored_mac[12]) {
			/* either all bytes where 0x00 or the sum does not fit the stored sum */
			valid_mac = false;
		}

		if (valid_mac) {
			memcpy(data->mac_addr, stored_mac, 6);
			LOG_INF("Using stored MAC address:");
		}
	#endif

	#if DT_PROP(ETH0_NODE_ID, zephyr_random_mac_address) \
			|| !NODE_HAS_VALID_MAC_ADDR(ETH0_NODE_ID)
		if (!valid_mac) {
			/* generate a random MAC using the Renesas Vendor OUI */
			gen_random_mac(data->mac_addr, 0x74, 0x90, 0x50);
			LOG_INF("Using random MAC address:");
		}
	#else
		if (!valid_mac) {
			LOG_INF("Using MAC address from device tree:");
		}
	#endif

	LOG_HEXDUMP_INF(data->mac_addr, 6, "MAC:");

	if (ret == 0) {
		ret = eth_rx65n_start(dev);
	}

	return ret;
}

static void eth_rx65n_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);

	DEV_DATA(data, dev);

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr),
			     NET_LINK_ETHERNET);

	ethernet_init(iface);
	net_if_flag_set(iface, NET_IF_NO_AUTO_START);

	if (data->iface == NULL) {
		data->iface = iface;
	}
}

static enum ethernet_hw_caps
eth_rx65n_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* capabilities of the Renesas RX65N ethernet controller according to
	 * User Manual Chapter 34:
	 * - promiscuous mode
	 * - full-duplex and half-duplex
	 * - 10 Mbps and 100 Mbps
	 */
	return ETHERNET_LINK_10BASE_T | ETHERNET_LINK_100BASE_T |
	       ETHERNET_DUPLEX_SET | ETHERNET_PROMISC_MODE;
}

/**
 * @brief set ethernet configuration (API function)
 *
 * @param type		type of configuration to set
 * @param config	configuration settings
 *
 * @returns		0 on success or negative error code
 */
static int eth_rx65n_set_config(const struct device *dev,
				enum ethernet_config_type type,
				const struct ethernet_config *config)
{
	DEV_DATA(data, dev);
	int ret;
	ether_param_t param;
	ether_promiscuous_t promiscuous;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:

		param.channel = ETHER_CHANNEL_0;
		if (config->promisc_mode) {
			promiscuous.bit = ETHER_PROMISCUOUS_ON;
		} else  {
			promiscuous.bit = ETHER_PROMISCUOUS_OFF;
		}
		param.p_ether_promiscuous = &promiscuous;
		ret = R_ETHER_Control(CONTROL_SET_PROMISCUOUS_MODE, param);
		if (ret != ETHER_SUCCESS) {
			return ret;
		}
		data->promiscuous = config->promisc_mode;
		break;
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		if (data->iface != NULL && net_if_flag_is_set(data->iface, NET_IF_UP)) {
			LOG_ERR("Attempting to change MAC while interface is up");
			return -EINVAL;
		}
		if (memcmp(data->mac_addr, config->mac_address.addr,
					sizeof(data->mac_addr))) {

			memcpy(data->mac_addr, config->mac_address.addr,
					sizeof(data->mac_addr));
			net_if_set_link_addr(data->iface, data->mac_addr,
						sizeof(data->mac_addr),
						NET_LINK_ETHERNET);
		}
		break;
	default:
		/* no other configuration type is supported at the moment */
		return -ENOTSUP;
	}

	return 0;
}

/**
 * @brief read ethernet configuration (API function)
 *
 * @param type		type of configuration to read
 * @param config	configuration settings
 *
 * @returns		0 on success or negative error code
 */
static int eth_rx65n_get_config(const struct device *dev,
				enum ethernet_config_type type,
				struct ethernet_config *config)
{
	DEV_DATA(data, dev);

	switch (type) {
	case ETHERNET_CONFIG_TYPE_LINK:
		/* since changing these settings is not supported yet, return
		 * default values
		 */
		config->l.link_10bt = true;
		config->l.link_100bt = true;
		config->l.link_1000bt = false;
		break;
	case ETHERNET_CONFIG_TYPE_DUPLEX:
		/* since changing these settings is not supported yet, return
		 * default values
		 */
		config->full_duplex = true;
		break;
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
		config->promisc_mode = data->promiscuous;
		break;
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(config->mac_address.addr, data->mac_addr,
				sizeof(data->mac_addr));
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int eth_rx65n_send(const struct device *dev, struct net_pkt *pkt)
{
	DEV_DATA(data, dev);
	uint16_t data_len, pbuf_size, send_len;
	uint8_t *tx_buffer;
	ether_return_t ret;
	int net_ret;

	/* reserve the send buffer or wait for it to be available */
	k_sem_take(&data->tx_buf_sem, K_FOREVER);

	data_len = net_pkt_get_len(pkt);

	ret = R_ETHER_Write_ZC2_GetBuf(0, (void **) &tx_buffer, &pbuf_size);
	if (ret != ETHER_SUCCESS) {
		LOG_ERR("R_ETHER_Write_ZC2_GetBuf error: %d", (int32_t)ret);
		return -1;
	}

	net_ret = net_pkt_read(pkt, tx_buffer, data_len);
	if (net_ret != 0) {
		LOG_ERR("net_pkt_read error: %d", net_ret);
		return -EIO;
	}

	if (data_len < 60) {
		/* R_ETHER_Write_ZC2_setBuf expects to send at least 60 bytes.
		 * If the packet is smaller, pad with 0s
		 */
		memset(&tx_buffer[data_len], 0, 60 - data_len);
		send_len = 60;
		LOG_DBG("Sending %u bytes (padded to 60 bytes)", data_len);
	} else {
		send_len = data_len;
		LOG_DBG("Sending %u bytes", data_len);
	}

	ret = R_ETHER_Write_ZC2_SetBuf(0, send_len);
	if (ret != ETHER_SUCCESS) {
		LOG_ERR("R_ETHER_Write_ZC2_SetBuf error: %d", (int32_t)ret);
		return -1;
	}

	return 0;
}

static const struct ethernet_api eth_rx65n_api = {
	.iface_api.init = eth_rx65n_iface_init,
	.get_capabilities = eth_rx65n_get_capabilities,
	.set_config = eth_rx65n_set_config,
	.get_config = eth_rx65n_get_config,
	.send = eth_rx65n_send,
	.start = eth_rx65n_start,
	.stop = eth_rx65n_stop,
};

ETH_NET_DEVICE_DT_DEFINE(ETH0_NODE_ID, eth_rx65n_init, NULL, &data, &cfg,
				CONFIG_ETH_INIT_PRIORITY, &eth_rx65n_api,
				NET_ETH_MTU);
