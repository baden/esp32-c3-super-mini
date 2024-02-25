/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// #include <sample_usbd.h>

#include <errno.h>
#include <string.h>

#define LOG_LEVEL 4
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/dac.h>
// #include <zephyr/usb/usb_device.h>
// #include <zephyr/usb/usbd.h>
// #include <zephyr/drivers/uart.h>

// BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
// 	     "Console device is not ACM CDC UART device");

// #if defined(CONFIG_USB_DEVICE_STACK_NEXT)
// static struct usbd_contex *sample_usbd;

// static int enable_usb_device_next(void)
// {
// 	int err;

// 	sample_usbd = sample_usbd_init_device();
// 	if (sample_usbd == NULL) {
// 		return -ENODEV;
// 	}

// 	err = usbd_enable(sample_usbd);
// 	if (err) {
// 		return err;
// 	}

// 	return 0;
// }
// #endif /* IS_ENABLED(CONFIG_USB_DEVICE_STACK_NEXT) */


#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define DAC_NODE DT_PHANDLE(ZEPHYR_USER_NODE, dac)
#define DAC_CHANNEL_ID DT_PROP(ZEPHYR_USER_NODE, dac_channel_id)
#define DAC_RESOLUTION DT_PROP(ZEPHYR_USER_NODE, dac_resolution)

static const struct device *const dac_dev = DEVICE_DT_GET(DAC_NODE);

static const struct dac_channel_cfg dac_ch_cfg = {
	.channel_id  = DAC_CHANNEL_ID,
	.resolution  = DAC_RESOLUTION,
	.buffered = true
};



// Адресний світлодіод (в подальшому можливо стрічка)
#define STRIP_NODE		DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS	DT_PROP(DT_ALIAS(led_strip), chain_length)

#define DELAY_TIME K_MSEC(CONFIG_SAMPLE_LED_UPDATE_DELAY)

#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

static const struct led_rgb colors[] = {
	RGB(0x0f, 0x00, 0x00), /* red */
	RGB(0x00, 0x0f, 0x00), /* green */
	RGB(0x00, 0x00, 0x0f), /* blue */
};

struct led_rgb pixels[STRIP_NUM_PIXELS];

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

int main(void)
{
	// const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	// uint32_t dtr = 0;

// #if defined(CONFIG_USB_DEVICE_STACK_NEXT)
// 	if (enable_usb_device_next()) {
// 		return 0;
// 	}
// #else
// 	if (usb_enable(NULL)) {
// 		return 0;
// 	}
// #endif

// 	/* Poll if the DTR flag was set */
// 	while (!dtr) {
// 		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
// 		/* Give CPU resources to low priority threads. */
// 		k_sleep(K_MSEC(100));
// 	}

// 		printk("Hello World! %s\n", CONFIG_ARCH);
// 		k_sleep(K_SECONDS(1));


	size_t cursor = 0, color = 0;
	int rc;

	if (device_is_ready(strip)) {
		LOG_INF("Found LED strip device %s", strip->name);
	} else {
		LOG_ERR("LED strip device %s is not ready", strip->name);
		return 0;
	}

	if (!device_is_ready(dac_dev)) {
		printk("DAC device %s is not ready\n", dac_dev->name);
		return 0;
	}
	int ret = dac_channel_setup(dac_dev, &dac_ch_cfg);
	if (ret != 0) {
		printk("Setting up of DAC channel failed with code %d\n", ret);
		return 0;
	}
	unsigned int dac_value = 0;

	LOG_INF("Displaying pattern on strip");
	while (1) {
		memset(&pixels, 0x00, sizeof(pixels));
		memcpy(&pixels[cursor], &colors[color], sizeof(struct led_rgb));
		rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);

		if (rc) {
			LOG_ERR("couldn't update strip: %d", rc);
		}

		cursor++;
		if (cursor >= STRIP_NUM_PIXELS) {
			cursor = 0;
			color++;
			if (color == ARRAY_SIZE(colors)) {
				color = 0;
			}
		}

		/* Number of valid DAC values, e.g. 4096 for 12-bit DAC */
		const int dac_values = 1U << DAC_RESOLUTION;

		ret = dac_write_value(dac_dev, DAC_CHANNEL_ID, dac_value);
		if (ret != 0) {
			printk("dac_write_value() failed with code %d\n", ret);
			return 0;
		}
		dac_value = (dac_value + 1) % dac_values;

		k_sleep(DELAY_TIME);
	}
	return 0;
}

// #include <zephyr/kernel.h>

// int main(void)
// {
//     int count = 0;
//     for(;;) {
//         printk("Hello World %d! %s\n", count++, CONFIG_BOARD);
//         k_sleep(K_MSEC(1000));
//     }
//     return 0;
// }
