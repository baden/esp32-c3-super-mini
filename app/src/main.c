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
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/dac.h>


// CRSF protocol
#define CENTER_X 991	// Чомусь мені здається, шо треба 992
#define CENTER_Y 991	// Чомусь мені здається, шо треба 992

#define MAX_X 820		// -820...820
#define MAX_Y 820		// -820...820
// Підвищення роздільної здатності задля кращої точності
#define MULTIPLIER 1

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

const struct gpio_dt_spec lf_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, lf_gpios);	// Left forward
const struct gpio_dt_spec lr_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, lr_gpios);	// Left reverse
const struct gpio_dt_spec rf_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, rf_gpios);	// Right forward
const struct gpio_dt_spec rr_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, rr_gpios);	// Right reverse

#define DAC0_NODE DT_PHANDLE(ZEPHYR_USER_NODE, dac0)
#define DAC0_CHANNEL_ID DT_PROP(ZEPHYR_USER_NODE, dac0_channel_id)
#define DAC0_RESOLUTION DT_PROP(ZEPHYR_USER_NODE, dac0_resolution)

#define DAC1_NODE DT_PHANDLE(ZEPHYR_USER_NODE, dac1)
#define DAC1_CHANNEL_ID DT_PROP(ZEPHYR_USER_NODE, dac1_channel_id)
#define DAC1_RESOLUTION DT_PROP(ZEPHYR_USER_NODE, dac1_resolution)

static const struct device *const dac0_dev = DEVICE_DT_GET(DAC0_NODE);
static const struct device *const dac1_dev = DEVICE_DT_GET(DAC1_NODE);

static const struct dac_channel_cfg dac0_ch_cfg = {
	.channel_id  = DAC0_CHANNEL_ID,
	.resolution  = DAC0_RESOLUTION,
	.buffered = true
};

static const struct dac_channel_cfg dac1_ch_cfg = {
	.channel_id  = DAC1_CHANNEL_ID,
	.resolution  = DAC1_RESOLUTION,
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

#if !defined(CONFIG_CRSF)
#define CONFIG_CRSF	1
#endif

#if defined(CONFIG_CRSF)
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
		// LOG_ERR("CRSF Frame Length: %d", len);
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
		// printk("1");
		return;
	}

	if (!uart_irq_rx_ready(crsf_dev)) {
		// printk("2");
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(crsf_dev, &c, 1) == 1) {
		// printk("3");
		// Wait for the start of a frame
		switch (crsf_state) {
		case CRSF_STATE_WAIT_SYNC:
			if (c == 0xC8) {
				crsf_state = CRSF_STATE_WAIT_LENGTH;
				// LOG_ERR("S");
				// printk("4");
			}
			break;
		case CRSF_STATE_WAIT_LENGTH:
			len = c;	// Expecting 0x18 only
			if(len > 32) {
				crsf_state = CRSF_STATE_WAIT_SYNC;
				// LOG_ERR("CRSF Frame Length: %d", len);
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
#endif

#if !defined(abs)
#define abs(x) ((x) < 0 ? -(x) : (x))
#endif

int io_init()
{
	int ret;
	// Шо за куйня? Нафіга тут таке?
	if(!gpio_is_ready_dt(&lf_gpio)) {
		LOG_ERR("Left forward GPIO device not found!");
		return -1;
	}
	ret = gpio_pin_configure_dt(&lf_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure left forward GPIO: %d", ret);
		return ret;
	}

	if(!gpio_is_ready_dt(&lr_gpio)) {
		LOG_ERR("Left reverse GPIO device not found!");
		return -1;
	}
	ret = gpio_pin_configure_dt(&lr_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure left reverse GPIO: %d", ret);
		return ret;
	}

	if(!gpio_is_ready_dt(&rf_gpio)) {
		LOG_ERR("Right forward GPIO device not found!");
		return -1;
	}
	ret = gpio_pin_configure_dt(&rf_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure right forward GPIO: %d", ret);
		return ret;
	}

	if(!gpio_is_ready_dt(&rr_gpio)) {
		LOG_ERR("Right reverse GPIO device not found!");
		return -1;
	}
	ret = gpio_pin_configure_dt(&rr_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure right reverse GPIO: %d", ret);
		return ret;
	}
	return 0;
}

int led_init()
{
	if(!device_is_ready(strip)) {
		LOG_ERR("LED strip device %s is not ready", strip->name);
		return -1;
	}
	return 0;
}

int crsf_init()
{
	int ret;
	#if defined(CONFIG_CRSF)
	if (!device_is_ready(crsf_dev)) {
		LOG_ERR("CRSF UART device not found!");
		return 0;
	} else {
		LOG_INF("CRSF UART device found!");
	}
	/* configure interrupt and callback to receive data */
	ret = uart_irq_callback_user_data_set(crsf_dev, serial_cb, NULL);
	if (ret < 0) {
		if (ret == -ENOTSUP) {
			LOG_ERR("Interrupt-driven UART API support not enabled");
		} else if (ret == -ENOSYS) {
			LOG_ERR("UART device does not support interrupt-driven API");
		} else {
			LOG_ERR("Error setting UART callback: %d", ret);
		}
		return 0;
	}
	uart_irq_rx_enable(crsf_dev);

	/* indefinitely wait for input from the user */
	// while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {

	// for(;;) {
	// 	if(crsf_frame_ready) {
	// 		crsf_frame_ready = false;
	// 		LOG_INF(
	// 			"0:%5d 1:%5d 2:%5d 3:%5d 4:%5d 5:%5d 6: %5d 7:%5d "
	// 			"8:%5d 9:%5d 10:%5d 11:%5d 12:%5d 13:%5d 14:%5d 15:%5d",
	// 			channel[0], channel[1], channel[2], channel[3], channel[4], channel[5], channel[6], channel[7],
	// 			channel[8], channel[9], channel[10], channel[11], channel[12], channel[13], channel[14], channel[15]
	// 		);
		
	// 	}
	// 	k_sleep(K_MSEC(100));
	// }
	#endif
	return 0;
}

int dac_init()
{
	int ret;
	if (!device_is_ready(dac0_dev)) {
		printk("DAC0 device %s is not ready\n", dac0_dev->name);
		return 0;
	}
	ret = dac_channel_setup(dac0_dev, &dac0_ch_cfg);
	if (ret != 0) {
		printk("Setting up of DAC0 channel failed with code %d\n", ret);
		return 0;
	}

	if (!device_is_ready(dac1_dev)) {
		printk("DAC1 device %s is not ready\n", dac1_dev->name);
		return 0;
	}
	ret = dac_channel_setup(dac1_dev, &dac1_ch_cfg);
	if (ret != 0) {
		printk("Setting up of DAC1 channel failed with code %d\n", ret);
		return 0;
	}
	return 0;
}

int led_update()
{
	static size_t cursor = 0, color = 0;
	int rc;

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
	return 0;
}

int dac_update(int left_wheel, int right_wheel)
{
	int ret;

	unsigned int dac0_value = abs(left_wheel) * 4094 / MAX_X / MULTIPLIER;

	if(dac0_value > 4094) {
		dac0_value = 4094;
	} else if(dac0_value < 0) {
		dac0_value = 0;
	}

	// Invert voltage
	dac0_value = 4094 - dac0_value;

	unsigned int dac1_value = abs(right_wheel) * 4094 / MAX_Y / MULTIPLIER;

	if(dac1_value > 4094) {
		dac1_value = 4094;
	} else if(dac1_value < 0) {
		dac1_value = 0;
	}

	// Invert voltage
	dac1_value = 4094 - dac1_value;

	// LOG_INF(
	// 	"L: %s:%d "
	// 	"R: %s:%d "
	// 	"DAC0: %d "
	// 	"DAC1: %d",
	// 	left_wheel_dir ? "^" : "v", left_wheel,
	// 	right_wheel_dir ? "^" : "v", right_wheel,
	// 	dac0_value,
	// 	dac1_value
	// );


	ret = dac_write_value(dac0_dev, DAC0_CHANNEL_ID, dac0_value);
	if (ret != 0) {
		LOG_ERR("dac0_write_value() failed with code %d\n", ret);
		return ret;
	}

	ret = dac_write_value(dac1_dev, DAC1_CHANNEL_ID, dac1_value);
	if (ret != 0) {
		printk("dac1_write_value() failed with code %d\n", ret);
		return ret;
	}

	return 0;
}

// Автопилот:
// 7й канал - вмикання (1792)/вимикання(191) автопілота 
// 6й канал - обмеження швидкості автопілота (192-повільно, 992-середньо, 1792-швидко)
// 10й канал - лівий двигун (191...1792)
// 11й канал - правий двигун (191...1792)


int crsf_update(int *left_wheel, int *right_wheel)
{
	// LOG_INF(
	// 	"0:%5d 1:%5d 2:%5d 3:%5d 4:%5d 5:%5d 6: %5d 7:%5d "
	// 	"8:%5d 9:%5d 10:%5d 11:%5d 12:%5d 13:%5d 14:%5d 15:%5d",
	// 	channel[0], channel[1], channel[2], channel[3], channel[4], channel[5], channel[6], channel[7],
	// 	channel[8], channel[9], channel[10], channel[11], channel[12], channel[13], channel[14], channel[15]
	// );

	if(channel[7] > 1000) {
		int speed = channel[6];
		int left_w = channel[10] - 191;
		int right_w = channel[11] - 191;
		// Швидкість
		int max_speed = 820;
		if(speed < 592) {
			max_speed = 820/3;
		} else if(speed < 1392) {
			max_speed = 820/2;
		}

		// Автопілот ввімкнено
		// Лівий двигун
		*left_wheel = left_w * max_speed / 1600;
		// Правий двигун
		*right_wheel = right_w * max_speed / 1600;
		return 0;
	}
	// 
	// int left_stick_x = channel[3];
	// int left_stick_y = channel[2];
	int right_stick_x = channel[0];
	int right_stick_y = channel[1];


	// Translate from 174...1811 to -820...820 (centered at 991)
	right_stick_x -= CENTER_X;
	right_stick_y -= CENTER_Y;

	#define DEADZONE 10
	bool in_deadzone = false;
	if((right_stick_x > -DEADZONE && right_stick_x < DEADZONE) && (right_stick_y > -DEADZONE && right_stick_y < DEADZONE)) {
		right_stick_x = 0;
		right_stick_y = 0;
		in_deadzone = true;
	}

	#if 0
	#define MULTIPLIER 16

	// Multiply by MULTIPLIER for better resolution
	right_stick_x *= MULTIPLIER;
	right_stick_y *= MULTIPLIER;

	// Rotate right_stick_x, right_stick_y 45 degrees counterclockwise
	int temp = right_stick_x;
	right_stick_x = (int)(0.70710678118 * right_stick_x - 0.70710678118 * right_stick_y);
	right_stick_y = (int)(0.70710678118 * right_stick_y + 0.70710678118 * temp);

	// int left_wheel = right_stick_y + right_stick_x;
	// int right_wheel = right_stick_y - right_stick_x;

	int left_wheel = abs(right_stick_y);
	int right_wheel = abs(right_stick_x);
	bool left_wheel_dir = right_stick_y > 0;
	bool right_wheel_dir = right_stick_x > 0;
	#endif

	// Проста поведінка - прцює тільки частина діапазону
	*left_wheel = right_stick_x + right_stick_y;
	*right_wheel = right_stick_y - right_stick_x;
	return 0;
}

static bool in_deadzone(int left_wheel, int right_wheel)
{
	return (left_wheel > -DEADZONE && left_wheel < DEADZONE) && (right_wheel > -DEADZONE && right_wheel < DEADZONE);
}

int set_dirs(int left_wheel, int right_wheel)
{
	bool left_wheel_dir = left_wheel > 0;
	bool right_wheel_dir = right_wheel > 0;

	if(in_deadzone(left_wheel, right_wheel)) {
		// Off all directions
		gpio_pin_set_dt(&lf_gpio, 0);
		gpio_pin_set_dt(&lr_gpio, 0);
		gpio_pin_set_dt(&rf_gpio, 0);
		gpio_pin_set_dt(&rr_gpio, 0);
	} else {
		if(left_wheel_dir) {
			gpio_pin_set_dt(&lr_gpio, 0);
			gpio_pin_set_dt(&lf_gpio, 1);
		} else {
			gpio_pin_set_dt(&lf_gpio, 0);
			gpio_pin_set_dt(&lr_gpio, 1);
		}
		if(right_wheel_dir) {
			gpio_pin_set_dt(&rr_gpio, 0);
			gpio_pin_set_dt(&rf_gpio, 1);
		} else {
			gpio_pin_set_dt(&rf_gpio, 0);
			gpio_pin_set_dt(&rr_gpio, 1);
		}
	}
	return 0;
}

int main(void)
{
	int left_wheel = 0;
	int right_wheel = 0;
	int new_left_wheel = 0;	// Updated values from CRSF or (0,0) if no new values
	int new_right_wheel = 0;

	if(io_init() < 0) return -1;
	if(led_init() < 0) return -1;
	if(crsf_init() < 0) return -1;
	if(dac_init() < 0) return -1;

	while (1) {
		led_update();

		if(crsf_frame_ready) {
			crsf_frame_ready = false;
			crsf_update(&new_left_wheel, &new_right_wheel);
		} else {
			// LOG_INF("CRSF Frame not ready");
			new_left_wheel = 0;
			new_right_wheel = 0;
		}

		// Low-pass filter
		left_wheel = (left_wheel + new_left_wheel) / 2;
		right_wheel = (right_wheel + new_right_wheel) / 2;

		// LOG_INF("L: %5d R: %5d ", left_wheel, right_wheel);

		set_dirs(left_wheel, right_wheel);

		dac_update(left_wheel, right_wheel);

		k_sleep(DELAY_TIME);
	}
	return 0;
}
