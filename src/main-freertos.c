/*
 * Copyright (c) 2024 Geoff Simmons <geoff@simmons.de>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * See LICENSE
 */

#include "pico/cyw43_arch.h"
#include "pico/multicore.h"

/* For HTTP_LOG_ERROR() */
#include "picow_http/log.h"

#include "tasks.h"

#include "FreeRTOS.h"
#include "task.h"

#define TASK_PRIO (tskIDLE_PRIORITY + 1UL)
#define PICO_STACK_WORDS (PICO_STACK_SIZE / sizeof(configSTACK_DEPTH_TYPE))

/*
 * For the single-core FreeRTOS version, the main function for core1 is
 * identical to that of the non-FreeRTOS version -- it initiates
 * asynchronous temperature and rssi updates, and exits.
 */
static void
core1_main(void)
{
	initiate_temp(NULL);
	initiate_rssi(&linkup);
}

int
main(void)
{
	BaseType_t ret;

	/* Global initialization */
	main_init();

	/* Launch the non-FreeRTOS thread on core1. */
	multicore_launch_core1(core1_main);

	/*
	 * Create a task that gets the WiFi connection and starts the http
	 * server.
	 */
	if ((ret = xTaskCreate(initiate_http, "http", PICO_STACK_WORDS, NULL,
			       TASK_PRIO, NULL)) != pdPASS) {
		HTTP_LOG_ERROR("Failed to create http task: %d", ret);
		exit(-1);
	}

	/* Run a FreeRTOS task scheduler. */
	vTaskStartScheduler();

	/* Unreachable */
	return 0;
}
