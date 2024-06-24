/*
 * Copyright (c) 2024 Geoff Simmons <geoff@simmons.de>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * See LICENSE
 */

#include "pico/cyw43_arch.h"
#include "pico/multicore.h"

#include "tasks.h"

#if PICO_CYW43_ARCH_POLL
#define POLL_SLEEP_MS (1)
#endif

/*
 * The main function for core1 initiates the asynchronous processes that
 * read the temperature and rssi. This function can then exit; everything
 * on the core is then IRQ- and timer-driven.
 */
void
core1_main(void)
{
	initiate_temp(NULL);
	initiate_rssi(&linkup);
}

int
main(void)
{
	/* Global initialization */
	main_init();

	/* Launch asynchronous temperature and rssi updates on core1. */
	multicore_launch_core1(core1_main);

	/* Get the WiFi connection and start the http server. */
	initiate_http(NULL);

	/*
	 * After the server starts, in poll mode we must periodically call
	 * cyw43_arch_poll(). Check if the timer has set the boolean to
	 * indicate that timeout for rssi updates has expired.
	 *
	 * Background mode is entirely interrupt-driven. So we use WFI to
	 * let the processor sleep until an interrupt is called.
	 */
	for (;;) {
		cyw43_arch_poll();
#if PICO_CYW43_ARCH_POLL
		cyw43_arch_wait_for_work_until(
			make_timeout_time_ms(POLL_SLEEP_MS));
#else
		__wfi();
#endif
	}

	/* Unreachable */
	return 0;
}
