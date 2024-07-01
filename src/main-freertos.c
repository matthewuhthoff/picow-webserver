/*
 * Copyright (c) 2024 Geoff Simmons <geoff@simmons.de>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * See LICENSE
 */

/*
 * The main function for FreeRTOS implementations of the sample app. Since
 * picow-http requires FreeRTOS with support of symmetric multiprocessing
 * (SMP), fail at compile time if the kernel doesn't support it.
 */

#if !FREE_RTOS_KERNEL_SMP
#error FreeRTOS version since V11.0.0 is required (with SMP support)
#endif

/* For PICOW_HTTP_ASSERT() */
#include "picow_http/assertion.h"

#include "tasks.h"

#include "FreeRTOS.h"
#include "task.h"

/*
 * The priority of the initialization tasks is probably not critical,
 * since they run once and exit. TASK_PRIO is one higher than the idle
 * task priority.
 *
 * Stack sizes for FreeRTOS are expressed in words, not bytes. The macro
 * converts PICO_STACK_SIZE to a value in words.
 */
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

	/*
	 * Create tasks for all of the initialization functions.
	 * In debug builds (NDEBUG is not defined), the assertions
	 * check that xTaskCreate() always succeeds.
	 *
	 * The task priorities are set as defined above. The init tasks
	 * for temperature and rssi measurements only need the smallest
	 * stack possible for FreeRTOS, which the init task for network
	 * and the http server needs a larger stack, as defined above in
	 * HTTP_STACK_SIZE.
	 */
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

	/*
	 * Start the FreeRTOS scheduler, which never exits. First the
	 * three intialization tasks run, then the SDK's implementation
	 * (pico_cyw43_arch_lwip_sys_freertos) starts tasks that implement
	 * lwIP callbacks, run the network driver, and run the at-time
	 * worker for rssi updates. Since picow-http requires FreeRTOS
	 * with SMP, the tasks may run on either of the PicoW's two cores.
	 */
	vTaskStartScheduler();

	/* Unreachable */
	return 0;
}
