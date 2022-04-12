/*
 * Copyright (c) 2022 KT-Elektronik, Klaucke und Partner GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_RTC_RX_H_
#define ZEPHYR_INCLUDE_DRIVERS_RTC_RX_H_

#include <time.h>

/**
 * @brief set the time of the RTC
 *
 * @param dev		RTC device structure
 * @param unix_time	the unix time (seconds since epoch) to set
 *
 * @return 0 on success or negative error code
 */
int rx_rtc_set_time(const struct device *dev, time_t unix_time);

#endif /* ZEPHYR_INCLUDE_DRIVERS_RTC_RX_H_ */
