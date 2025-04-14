/*
 * Copyright (c) 2024 Geoff Simmons <geoff@simmons.de>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * See LICENSE
 */

/*
 * The main function for FreeRTOS implementations of the sample app.
 */

#include "pico/multicore.h"

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

#if !FREE_RTOS_KERNEL_SMP || configNUMBER_OF_CORES == 1

/*
 * Temperature sensor readings are all IRQ-driven, reading from the
 * free-running ADC, and do not require a FreeRTOS task or context.
 * These can run on the non-FreeRTOS core in a single-core version.
 */
static void
core1_main(void)
{
	initiate_temp(&linkup);
}

#endif

int
main(void)
{
	BaseType_t ret;
	TaskHandle_t temp_task, rssi_task, picture_task, http_task;

	/* Global initialization */
	main_init();

#if !FREE_RTOS_KERNEL_SMP || configNUMBER_OF_CORES == 1
	/*
	 * In the single-core version, initiate temperature sensor
	 * readings on core1.
	 */
	multicore_launch_core1(core1_main);
#else
	/*
	 * Create tasks for all of the initialization functions.
	 * In debug builds (NDEBUG is not defined), the assertions
	 * check that xTaskCreate() always succeeds.
	 *
	 * The task priorities are set as defined above. The init tasks
	 * for temperature and rssi measurements only need the smallest
	 * stack possible for FreeRTOS (configMINIMAL_STACK_SIZE).
	 */

	/*
	 * In the SMP version, initiate temperature measurements in
	 * main().  The task scheduler decides what tasks run on which
	 * core.
	 */
	ret = xTaskCreate(initiate_temp, "temp", configMINIMAL_STACK_SIZE,
			  NULL, TASK_PRIO, &temp_task);
	PICOW_HTTP_ASSERT(ret == pdPASS);
#endif

	/*
	 * Pass in the linkup semaphore, so that the at-time worker for
	 * rssi updates starts after the wifi connection reaches the
	 * linkup state.
	 */
	ret = xTaskCreate(initiate_rssi, "rssi", configMINIMAL_STACK_SIZE,
			  &linkup, TASK_PRIO, &rssi_task);
	PICOW_HTTP_ASSERT(ret == pdPASS);


	// ret = xTaskCreate(initiate_picture, "picture", configMINIMAL_STACK_SIZE,
	// 		  NULL, TASK_PRIO, &picture_task);
	// PICOW_HTTP_ASSERT(ret == pdPASS);


	/*
	 * The init task for the network and http server needs a
	 * larger stack, as defined above in HTTP_STACK_SIZE.
	 */
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
