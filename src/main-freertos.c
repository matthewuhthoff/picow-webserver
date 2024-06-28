/*
 * Copyright (c) 2024 Geoff Simmons <geoff@simmons.de>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * See LICENSE
 */

#if !FREE_RTOS_KERNEL_SMP
#error FreeRTOS version since V11.0.0 is required (with SMP support)
#endif

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

int
main(void)
{
	BaseType_t ret;
	TaskHandle_t temp_task, rssi_task, http_task;

	/* Global initialization */
	main_init();

	/* Create tasks for all of the initialization functions. */
	if ((ret = xTaskCreate(initiate_temp, "temp", configMINIMAL_STACK_SIZE,
			       NULL, TASK_PRIO, &temp_task)) != pdPASS) {
		HTTP_LOG_ERROR("Failed to create temp task: %d", ret);
		exit(-1);
	}
	/*
	 * Pass in the linkup semaphore, so that the at-time worker for
	 * rssi updates starts after the wifi connection reaches the
	 * linkup state.
	 */
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

	vTaskStartScheduler();

	/* Unreachable */
	return 0;
}
