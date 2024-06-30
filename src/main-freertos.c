/*
 * Copyright (c) 2024 Geoff Simmons <geoff@simmons.de>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * See LICENSE
 */

#if !FREE_RTOS_KERNEL_SMP
#error FreeRTOS version since V11.0.0 is required (with SMP support)
#endif

/* For PICOW_HTTP_ASSERT() */
#include "picow_http/assertion.h"

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
	ret = xTaskCreate(initiate_temp, "temp", configMINIMAL_STACK_SIZE,
			  NULL, TASK_PRIO, &temp_task);
	PICOW_HTTP_ASSERT(ret == pdPASS);

	/*
	 * Pass in the linkup semaphore, so that the at-time worker for
	 * rssi updates starts after the wifi connection reaches the
	 * linkup state.
	 */
	ret = xTaskCreate(initiate_rssi, "rssi", configMINIMAL_STACK_SIZE,
			  &linkup, TASK_PRIO, &rssi_task);
	PICOW_HTTP_ASSERT(ret == pdPASS);

	ret = xTaskCreate(initiate_http, "http", HTTP_STACK_SIZE, NULL,
			  TASK_PRIO, &http_task);
	PICOW_HTTP_ASSERT(ret == pdPASS);

	vTaskStartScheduler();

	/* Unreachable */
	return 0;
}
