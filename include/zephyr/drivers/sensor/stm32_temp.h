/*
 * Copyright (c) 2023 Tools for Humanity GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_STM32_TEMP_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_STM32_TEMP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/sensor.h>

enum sensor_attribute_stm32_temp {
	SENSOR_ATTR_VREF_MV = SENSOR_ATTR_PRIV_START,
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_STM32_TEMP_H_ */
