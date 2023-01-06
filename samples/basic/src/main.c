/** @file main.c
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2022 Irnas. All rights reserved.
 * Author: Marko Sagadin <marko@irnas.eu>
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(main);

const struct device *pca9554 = DEVICE_DT_GET(DT_NODELABEL(pca9554));

const struct gpio_dt_spec leds[] = {
	GPIO_DT_SPEC_GET(DT_PATH(pins, led4), gpios),
	GPIO_DT_SPEC_GET(DT_PATH(pins, led5), gpios),
	GPIO_DT_SPEC_GET(DT_PATH(pins, led6), gpios),
	GPIO_DT_SPEC_GET(DT_PATH(pins, led7), gpios),
};

void main(void)
{
	if (!device_is_ready(pca9554)) {
		LOG_ERR("pca9554 not ready!");
		while (1) {
		}
	}
	while (1) {
		LOG_INF("Toggling gpio pins directly with PCA9554 device");
		for (size_t i = 4; i < 8; i++) {
			gpio_pin_configure(pca9554, i, GPIO_OUTPUT_LOW);
			k_msleep(100);
		}

		for (size_t i = 0; i < 10; i++) {
			for (size_t j = 4; j < 8; j++) {
				gpio_pin_toggle(pca9554, j);
				k_msleep(100);
			}
		}

		LOG_INF("Toggling gpio pins over gpio_dt spec structs");
		for (size_t i = 0; i < 4; i++) {
			gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
			k_msleep(100);
		}

		for (size_t i = 0; i < 10; i++) {
			for (size_t j = 0; j < 4; j++) {
				gpio_pin_toggle_dt(&leds[j]);
				k_msleep(100);
			}
		}
	}
}
