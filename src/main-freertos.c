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

/* https initialization requires a larger stack. */
#if PICOW_HTTPS
#define HTTP_STACK_SIZE (2 * PICO_STACK_WORDS)
#else
#define HTTP_STACK_SIZE (PICO_STACK_WORDS)
#endif

#ifndef FREE_RTOS_KERNEL_SMP

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
	if ((ret = xTaskCreate(initiate_http, "http", HTTP_STACK_SIZE, NULL,
			       TASK_PRIO, NULL)) != pdPASS) {
		HTTP_LOG_ERROR("Failed to create http task: %d", ret);
		exit(-1);
	}

	/* Run a FreeRTOS task scheduler. */
	vTaskStartScheduler();

	/* Unreachable */
	return 0;
}

#else // FreeRTOS SMP

#if !configUSE_CORE_AFFINITY
#error "configUSE_CORE_AFFINITY false for FreeRTOS SMP"
#endif

int
main(void)
{
	BaseType_t ret;
	TaskHandle_t temp_task, rssi_task, http_task;

	/* Global initialization */
	main_init();

	/*
	 * Create a task that gets the WiFi connection and starts the http
	 * server.
	 */
	if ((ret = xTaskCreate(initiate_temp, "temp", configMINIMAL_STACK_SIZE,
			       NULL, TASK_PRIO, &temp_task)) != pdPASS) {
		HTTP_LOG_ERROR("Failed to create temp task: %d", ret);
		exit(-1);
	}
	if ((ret = xTaskCreate(initiate_rssi, "rssi", configMINIMAL_STACK_SIZE,
			       &linkup, TASK_PRIO, &rssi_task)) != pdPASS) {
		HTTP_LOG_ERROR("Failed to create rssi task: %d", ret);
		exit(-1);
	}
	if ((ret = xTaskCreate(initiate_http, "http", HTTP_STACK_SIZE, NULL,
			       TASK_PRIO + 1, &http_task)) != pdPASS) {
		HTTP_LOG_ERROR("Failed to create http task: %d", ret);
		exit(-1);
	}

# if NO_SYS
	vTaskCoreAffinitySet(http_task, 1);
	vTaskCoreAffinitySet(temp_task, 2);
	vTaskCoreAffinitySet(rssi_task, 2);
# endif

	vTaskStartScheduler();

	/* Unreachable */
	return 0;
}

#endif
