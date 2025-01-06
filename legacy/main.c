/*
 * Copyright (c) 2023 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <lvgl.h>
#include <lvgl_mem.h>
#include <lv_demos.h>
#include <stdio.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	const struct device *display_dev;
	int ret;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	gpio_pin_set_dt(&led, 0);

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Device not ready, aborting test");
		return 0;
	}

#if defined(CONFIG_LV_USE_DEMO_MUSIC)
	lv_demo_music();
#elif defined(CONFIG_LV_USE_DEMO_BENCHMARK)
	lv_demo_benchmark();
#elif defined(CONFIG_LV_USE_DEMO_STRESS)
	lv_demo_stress();
#elif defined(CONFIG_LV_USE_DEMO_WIDGETS)
	lv_demo_widgets();
#else
#error Enable one of the demos CONFIG_LV_USE_DEMO_MUSIC, CONFIG_LV_USE_DEMO_BENCHMARK ,\
	CONFIG_LV_USE_DEMO_STRESS, or CONFIG_LV_USE_DEMO_WIDGETS
#endif

	lv_task_handler();
	display_blanking_off(display_dev);
#ifdef CONFIG_LV_Z_MEM_POOL_SYS_HEAP
	lvgl_print_heap_info(false);
#else
	printf("lvgl in malloc mode\n");
#endif
	gpio_pin_set_dt(&led, 1);
	while (1) {
		uint32_t sleep_ms = lv_task_handler();

		k_msleep(MIN(sleep_ms, INT32_MAX));
	}

	return 0;
}
