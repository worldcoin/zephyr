/*
 * Copyright (c) 2018 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT apa_apa102

#include <errno.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

struct apa102_config {
	struct spi_dt_spec bus;
};

static int apa102_update(const struct device *dev, void *buf, size_t size)
{
	const struct apa102_config *config = dev->config;
	static const uint8_t zeros[] = { 0, 0, 0, 0 };

	/*
	 * the only function of the “End frame” is to supply more clock pulses to the string until
	 * the data has permeated to the last LED. The number of clock pulses required is exactly
	 * half the total number of LEDs in the string
	 */
	BUILD_ASSERT(sizeof(struct led_rgb) == 4); /* see also BUILD_ASSERT in apa102_update_rgb */
	uint8_t ones[((size / sizeof(struct led_rgb)) / sizeof(uint8_t) / 2) + 1];
	memset(ones, 0xFF, sizeof ones);

	const struct spi_buf tx_bufs[] = {
		{
			/* Start frame: at least 32 zeros */
			.buf = (uint8_t *)zeros,
			.len = sizeof(zeros),
		},
		{
			/* LED data itself */
			.buf = buf,
			.len = size,
		},
		{
			/* End frame: at least 32 ones to clock the
			 * remaining bits to the LEDs at the end of
			 * the strip.
			 */
			.buf = (uint8_t *)ones,
			.len = sizeof(ones),
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs)
	};

	return spi_write_dt(&config->bus, &tx);
}

static int apa102_update_rgb(const struct device *dev, struct led_rgb *pixels,
			     size_t count)
{
	uint8_t *p = (uint8_t *)pixels;
	size_t i;
	/* SOF (3 bits) followed by the 0 to 31 global dimming level */
	const uint8_t prefix = 0xE0;

	/* Rewrite to the on-wire format */
	for (i = 0; i < count; i++) {
		uint8_t r = pixels[i].r;
		uint8_t g = pixels[i].g;
		uint8_t b = pixels[i].b;
		uint8_t brightness = 31;
#if CONFIG_LED_STRIP_RGB_SCRATCH
		brightness = pixels[i].scratch & 0x1F;
#endif

		*p++ = prefix | brightness;
		*p++ = b;
		*p++ = g;
		*p++ = r;
	}

	BUILD_ASSERT(sizeof(struct led_rgb) == 4);
	return apa102_update(dev, pixels, sizeof(struct led_rgb) * count);
}

static int apa102_update_channels(const struct device *dev, uint8_t *channels,
				  size_t num_channels)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channels);
	ARG_UNUSED(num_channels);

	/* Not implemented */
	return -EINVAL;
}

static int apa102_init(const struct device *dev)
{
	const struct apa102_config *config = dev->config;

	if (!spi_is_ready_dt(&config->bus)) {
		return -ENODEV;
	}

	return 0;
}

static const struct led_strip_driver_api apa102_api = {
	.update_rgb = apa102_update_rgb,
	.update_channels = apa102_update_channels,
};

#define APA102_DEVICE(idx)						 \
	static const struct apa102_config apa102_##idx##_config = {	 \
		.bus = SPI_DT_SPEC_INST_GET(				 \
			idx,						 \
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8), \
			0),						 \
	};								 \
									 \
	DEVICE_DT_INST_DEFINE(idx,					 \
			      apa102_init,				 \
			      NULL,					 \
			      NULL,					 \
			      &apa102_##idx##_config,			 \
			      POST_KERNEL,				 \
			      CONFIG_LED_STRIP_INIT_PRIORITY,		 \
			      &apa102_api);

DT_INST_FOREACH_STATUS_OKAY(APA102_DEVICE)
