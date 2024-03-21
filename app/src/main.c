/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// #include <sample_usbd.h>

#include <errno.h>
#include <string.h>


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
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

#define DEV_CRFS   DEVICE_DT_GET(DT_CHOSEN(crsf))
static const struct device *const crsf_dev = DEV_CRFS;

// CRSF protocol state machine
enum crsf_state {
	CRSF_STATE_WAIT_SYNC,
	CRSF_STATE_WAIT_LENGTH,
	CRSF_STATE_PAYLOAD,
	// CRFS_STATE_END
};

enum crsf_state crsf_state = CRSF_STATE_WAIT_SYNC;

volatile int channel[16] = {0};
volatile bool crsf_frame_ready = false;

typedef struct __attribute__((packed)) crsf_frame {
	uint8_t type;
	unsigned ch0: 11;
	unsigned ch1: 11;
	unsigned ch2: 11;
	unsigned ch3: 11;
	unsigned ch4: 11;
	unsigned ch5: 11;
	unsigned ch6: 11;
	unsigned ch7: 11;
	unsigned ch8: 11;
	unsigned ch9: 11;
	unsigned ch10: 11;
	unsigned ch11: 11;
	unsigned ch12: 11;
	unsigned ch13: 11;
	unsigned ch14: 11;
	unsigned ch15: 11;
	uint8_t crc;
} crsf_frame_t;

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len){
	static const uint8_t crsf_crc8tab[256] = {
		0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
		0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
		0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
		0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
		0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
		0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
		0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
		0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
		0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
		0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
		0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
		0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
		0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
		0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
		0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
		0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) {
		crc = crsf_crc8tab[crc ^ *ptr++];
	}
	return crc;
}

void crsf_parse_payload(uint8_t *payload, uint8_t len) {

	if(len != 0x18) {
		return;
	}
	if (payload[0] != 0x16) {		// Type: RC Channels
		// LOG_ERR("CRSF Frame Type: 0x%02X", payload[0]);
		return;
	}

	crsf_frame_t *frame = (crsf_frame_t *)payload;

	channel[0] = frame->ch0;
	channel[1] = frame->ch1;
	channel[2] = frame->ch2;
	channel[3] = frame->ch3;
	channel[4] = frame->ch4;
	channel[5] = frame->ch5;
	channel[6] = frame->ch6;
	channel[7] = frame->ch7;
	channel[8] = frame->ch8;
	channel[9] = frame->ch9;
	channel[10] = frame->ch10;
	channel[11] = frame->ch11;
	channel[12] = frame->ch12;
	channel[13] = frame->ch13;
	channel[14] = frame->ch14;
	channel[15] = frame->ch15;

	crsf_frame_ready = true;

	// LOG_ERR("CH0:%5d CH1:%5d CH2:%5d CH3:%5d CH4:%5d CH5:%5d CH6: %5d CH7:%5d",
	// 	frame->ch0, frame->ch1, frame->ch2, frame->ch3, frame->ch4, frame->ch5, frame->ch6, frame->ch7
	// );
	// LOG_ERR("CRSF Frame Type: %d", frame->type);
	// LOG_ERR("CRSF Channel 0: %d", frame->ch0);
	// LOG_ERR("CRSF Channel 1: %d", frame->ch1);
	// LOG_ERR("CRSF Payload Length: %d", len);
	// uint8_t checksum = payload[len-1];
	// uint8_t crc = crsf_crc8(payload, len-1);

	// LOG_ERR("CRSF CRC: 0x%02X, Got: 0x%02X", checksum, crc);
	// LOG_HEXDUMP_ERR(payload, len, "CRSF Payload");
}

/*
 * Read characters from CRSF UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;
	// static char rx_buf[32];
	// static int rx_buf_pos;
	static int len;
	static unsigned int crsf_payload_pos;
	static uint8_t crsf_payload[32];

	if (!uart_irq_update(crsf_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(crsf_dev)) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(crsf_dev, &c, 1) == 1) {
		// Wait for the start of a frame
		switch (crsf_state) {
		case CRSF_STATE_WAIT_SYNC:
			if (c == 0xC8) {
				crsf_state = CRSF_STATE_WAIT_LENGTH;
				// LOG_ERR("S");
			}
			break;
		case CRSF_STATE_WAIT_LENGTH:
			len = c;	// Expecting 0x18 only
			if(len > 32) {
				crsf_state = CRSF_STATE_WAIT_SYNC;
				break;
			}
			crsf_payload_pos = 0;
			crsf_state = CRSF_STATE_PAYLOAD;
			break;
		case CRSF_STATE_PAYLOAD:
			crsf_payload[crsf_payload_pos++] = c;
			if (crsf_payload_pos == len) {
				crsf_state = CRSF_STATE_WAIT_SYNC;
				crsf_parse_payload(crsf_payload, len);
			}
			break;
		}


		
	}

}

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
	int ret;

	if (device_is_ready(strip)) {
		LOG_INF("Found LED strip device %s", strip->name);
	} else {
		LOG_ERR("LED strip device %s is not ready", strip->name);
		return 0;
	}


	if (!device_is_ready(crsf_dev)) {
		printk("CRSF UART device not found!");
		return 0;
	}
	/* configure interrupt and callback to receive data */
	ret = uart_irq_callback_user_data_set(crsf_dev, serial_cb, NULL);
	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}
	uart_irq_rx_enable(crsf_dev);


	/* indefinitely wait for input from the user */
	// while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {

	for(;;) {
		if(crsf_frame_ready) {
			crsf_frame_ready = false;
			LOG_DBG("CH0:%5d CH1:%5d CH2:%5d CH3:%5d CH4:%5d CH5:%5d CH6: %5d CH7:%5d",
				channel[0], channel[1], channel[2], channel[3], channel[4], channel[5], channel[6], channel[7]
			);
		
		}
		k_sleep(K_MSEC(100));
	}

	for(;;) {
		printk("Hello World! %s\n", CONFIG_ARCH);
		k_sleep(K_SECONDS(10));
	}

	if (!device_is_ready(dac_dev)) {
		printk("DAC device %s is not ready\n", dac_dev->name);
		return 0;
	}
	ret = dac_channel_setup(dac_dev, &dac_ch_cfg);
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
