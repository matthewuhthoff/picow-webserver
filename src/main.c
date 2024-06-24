/*
 * Copyright (c) 2022 Geoff Simmons <geoff@simmons.de>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * See LICENSE
 */

#include <stdio.h>
#include <stdint.h>

#include "pico/stdio_uart.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include "hardware/regs/adc.h"
#include "hardware/sync.h"

/* For LWIP_VERSION_STRING */
#include "lwip/init.h"

/* For MBEDTLS_VERSION_STRING */
#if PICOW_HTTPS
#include "mbedtls/version.h"
#endif

/*
 * Code using picow-http must include picow_http/http.h.
 * cmake configuration ensures that it is on the include path.
 *
 * See: https://slimhazard.gitlab.io/picow_http/
 */
#include "picow_http/http.h"
#include "handlers.h"

#if PICO_CYW43_ARCH_POLL
#define POLL_SLEEP_MS (1)
#endif

#define ADC_TEMP_CH (4)
/*
 * Maximum value for the ADC clock divider. Results in about 7.6 Hz, or
 * approximately one measurement every 131 ms.
 */
#define ADC_MAX_CLKDIV (65535.f + 255.f/256.f)

/*
 * Interval between rssi updates in ms (for a repeating_timer).
 */
#define RSSI_INTVL_MS (5 * 1000)

/* The following values are shared between the two cores. */

/* Raw ADC reading for the temperature sensor. */
static volatile uint16_t temp_adc_raw = UINT16_MAX;
/* The most recent rssi value for our access point */
static volatile int32_t rssi = INT32_MAX;
/* Struct for network information, passed to the /netinfo handler */
static struct netinfo netinfo;

/* Critical sections to protect access to shared data */
static critical_section_t temp_critsec, rssi_critsec;

/*
 * The linkup semaphore is signaled when the TCP/IP link is up, so that
 * core1 knows that the periodic rssi updates may begin.
 */
static semaphore_t linkup;

/* at-time worker for rssi updates */
static void rssi_work(async_context_t *context, async_at_time_worker_t *worker);
static async_at_time_worker_t rssi_worker = {
	.do_work = rssi_work,
	.user_data = NULL,
};

/*
 * ISR for ADC_IRQ_FIFO, called when the ADC writes a value to its
 * FIFO. Save the raw ADC value in temp_adc_raw, protected by a critical
 * section.
 */
static void
__time_critical_func(temp_isr)(void)
{
	uint16_t val = adc_fifo_get();
	critical_section_enter_blocking(&temp_critsec);
	temp_adc_raw = val;
	critical_section_exit(&temp_critsec);
}

/*
 * See the comment in handlers.h
 *
 * Since a fixed-point value is used, the ADC to temperature conversion
 * can be done with only integer arithmetic. The client side (Javascript)
 * scales the result.
 */
uint32_t
get_temp(void)
{
	uint16_t raw;

	critical_section_enter_blocking(&temp_critsec);
	raw = temp_adc_raw;
	critical_section_exit(&temp_critsec);

	if (raw == UINT16_MAX)
		return UINT32_MAX;
	if ((raw & ADC_FIFO_ERR_BITS) != 0)
		return UINT32_MAX;
	/*
	 * This formula is equivalent to the temperature sensor conversion
	 * formula given in the RP2040 datasheet and the SDK docs, except
	 * for degrees Kelvin, and for the fixed-point Q18.14 result.
	 */
	return 11638810 - 7670 * raw;
}

/*
 * See the comment in handler.h
 */
int32_t
get_rssi(void)
{
	int32_t val;

	critical_section_enter_blocking(&rssi_critsec);
	val = rssi;
	critical_section_exit(&rssi_critsec);
	return val;
}

/*
 * Worker funcion to update the AP rssi.
 */
static void
rssi_work(async_context_t *ctx, async_at_time_worker_t *wrk)
{
	int32_t val;

	if (cyw43_wifi_get_rssi(&cyw43_state, &val) != 0)
		val = INT32_MAX;

	critical_section_enter_blocking(&rssi_critsec);
	rssi = val;
	critical_section_exit(&rssi_critsec);

	PICOW_HTTP_ASSERT(async_context_add_at_time_worker_in_ms(
				  ctx, wrk, RSSI_INTVL_MS));
}

/*
 * main function for core1.
 *
 * Configure the ADC in free-running mode to read from the temperature
 * sensor, and set the ADC FIFO to length 1. Configure temp_isr() as
 * the interrupt handler to run when the ADC writes to the FIFO.
 *
 * In background mode, the repeating timer (from the SDK's pico_time) runs
 * periodically to get the current rssi value. Wait until the network link
 * is up before starting the timer.
 */
void
core1_main(void)
{
	/* Initiate asynchronous ADC temperature sensor reads */
	adc_init();
	adc_set_temp_sensor_enabled(true);
	adc_select_input(ADC_TEMP_CH);
	/* Write to FIFO length 1, and retain the ERR bit. */
	adc_fifo_setup(true, false, 1, true, false);
	adc_set_clkdiv(ADC_MAX_CLKDIV);

	irq_set_exclusive_handler(ADC_IRQ_FIFO, temp_isr);
	adc_irq_set_enabled(true);
	irq_set_enabled(ADC_IRQ_FIFO, true);
	adc_run(true);

	/* Wait for the other core to signal that the network link is up. */
	sem_acquire_blocking(&linkup);
	async_context_add_at_time_worker_in_ms(
		cyw43_arch_async_context(), &rssi_worker, 0);
}

int
main(void)
{
	struct server *srv;
	struct server_cfg cfg;
	int link_status = CYW43_LINK_DOWN;
	struct netif *netif;
	uint8_t mac[6];
	err_t err;

	/* For picotool info */
	bi_decl(bi_program_feature("hostname: " CYW43_HOST_NAME));
	bi_decl(bi_program_feature("AP SSID: " WIFI_SSID));
	bi_decl(bi_program_feature("picow-http version: " PICOW_HTTP_VERSION));
	bi_decl(bi_program_feature("lwIP version: " LWIP_VERSION_STRING));
#if PICO_CYW43_ARCH_POLL
	bi_decl(bi_program_feature("arch: poll"));
#elif PICO_CYW43_ARCH_THREADSAFE_BACKGROUND
	bi_decl(bi_program_feature("arch: threadsafe background"));
#endif
#if PICOW_HTTPS
	bi_decl(bi_program_feature("TLS: yes"));
	bi_decl(bi_program_feature("mbedtls version: " MBEDTLS_VERSION_STRING));
#else
	bi_decl(bi_program_feature("TLS: no"));
#endif

	/* Initialize the critical sections and semaphore */
	critical_section_init(&temp_critsec);
	critical_section_init(&rssi_critsec);
	sem_init(&linkup, 0, 1);

	/*
	 * Launch core1. The code preceding multicore_launch_core1()
	 * ensures that core1 is properly reset when a reset is initiated
	 * by openocd.
	 */
	sleep_ms(5);
	multicore_reset_core1();
	sleep_ms(5);
	multicore_launch_core1(core1_main);

	/*
	 * HTTP log output uses the configuration for pico_stdio.
	 * Here we initalize UART for output only.
	 */
	stdout_uart_init();

	/*
	 * Initialize networking in station mode, and connect to the
	 * access point passed in as WIFI_SSID.
	 */
	if (cyw43_arch_init() != 0) {
		HTTP_LOG_ERROR("Network architecture initialization failed");
		exit(-1);
	}

	cyw43_arch_enable_sta_mode();
	HTTP_LOG_INFO("Connecting to " WIFI_SSID " ...");
	do {
		if (cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD,
						  CYW43_AUTH_WPA2_AES_PSK) != 0)
			continue;
		do {
			/*
			 * In poll mode (pico_cyw43_arch_lwip_poll), we
			 * must call the polling function here. Not
			 * necessary in other modes.
			 */
			cyw43_arch_poll();
			if ((link_status =
			     cyw43_tcpip_link_status(&cyw43_state,
						     CYW43_ITF_STA))
			    != CYW43_LINK_UP) {
				if (link_status < 0) {
					HTTP_LOG_ERROR(
						"WiFi connect error status: %d",
						link_status);
					break;
				}
				sleep_ms(100);
			}
		} while (link_status != CYW43_LINK_UP);
	} while (link_status != CYW43_LINK_UP);

	/*
	 * Signal the other core that the link is up, so that it knows to
	 * start rssi updates.
	 */
	sem_release(&linkup);

	HTTP_LOG_INFO("Connected to " WIFI_SSID);

	/*
	 * After the link comes up, but before starting the http server,
	 * store the PicoW's IP address and MAC address in the static
	 * netinfo object. A pointer to that object will be passed in as
	 * the private pointer of the custom response handler for URL path
	 * /netinfo.
	 *
	 * INIT_OBJ() from picow_http/assertion.h zeroes the object and
	 * sets its magic number field, so that the pointer can be
	 * checked for validity in the custom handler.
	 * See: https://slimhazard.gitlab.io/picow_http/group__assert.html
	 */
	INIT_OBJ(&netinfo, NETINFO_MAGIC);
	cyw43_arch_lwip_begin();
	netif = &cyw43_state.netif[CYW43_ITF_STA];
	strncpy(netinfo.ip, ipaddr_ntoa(netif_ip_addr4(netif)),
		IPADDR_STRLEN_MAX);
	if (cyw43_wifi_get_mac(&cyw43_state, CYW43_ITF_STA, mac) == 0)
		snprintf(netinfo.mac, MAC_ADDR_LEN,
			 "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1],
			 mac[2], mac[3], mac[4], mac[5]);
	else
		HTTP_LOG_ERROR("Could not get mac address");
	cyw43_arch_lwip_end();

	/*
	 * Start with the default configuration for the HTTP server.
	 *
	 * If a list of NTP servers was named at compile time, set it
	 * here.  We set a longer idle timeout than specified in the
	 * default configuration, since Javascript code runs updates every
	 * few seconds for /temp and /rssi (to update the temperature and
	 * signal strength in the UI). Most browsers leave connections
	 * established after a page is loaded, so keep them open to be
	 * re-used for the updates.
	 *
	 * See: https://slimhazard.gitlab.io/picow_http/group__server.html#ga3380001925d9eb091370db86d85f7349
	 */
	cfg = http_default_cfg();
#ifdef NTP_SERVERS
	cfg.ntp_cfg.servers = NTP_SERVERS;
#endif
	cfg.idle_tmo_s = 30;

	/*
	 * Before the http server starts, register the custom handlers for
	 * the URL paths /netinfo, /temp, /rssi and /led. Each of them is
	 * registered for the methods GET and HEAD.
	 *
	 * For /netinfo, we pass in the address of the netinfo object that
	 * was just initialized. The other handlers do not use private
	 * data, so we pass in NULL.
	 *
	 * Custom handlers can be registered after the server starts; for
	 * any requests for a path with an unregistered handler, the
	 * server returns a 404 ("Not found") error response. By
	 * registering before server start, we ensure that the handlers
	 * are available right away.
	 *
	 * See: https://slimhazard.gitlab.io/picow_http/group__resp.html#gac4ee42ee6a8559778bb486dcb6253cfe
	 */
	err = register_hndlr_methods(&cfg, "/netinfo", netinfo_handler,
				     HTTP_METHODS_GET_HEAD, &netinfo);
	PICOW_HTTP_ASSERT(err == ERR_OK);

	err = register_hndlr_methods(&cfg, "/temp", temp_handler,
				     HTTP_METHODS_GET_HEAD, NULL);
	PICOW_HTTP_ASSERT(err == ERR_OK);

	err = register_hndlr_methods(&cfg, "/led", led_handler,
				     HTTP_METHODS_GET_HEAD, NULL);
	PICOW_HTTP_ASSERT(err == ERR_OK);

	err = register_hndlr_methods(&cfg, "/rssi", rssi_handler,
				     HTTP_METHODS_GET_HEAD, NULL);
	PICOW_HTTP_ASSERT(err == ERR_OK);


	/*
	 * Start the server, and turn on the onboard LED when it's
	 * running.
	 *
	 * See: https://slimhazard.gitlab.io/picow_http/group__server.html#gae2b8bdf44100f13cd2c5e18969208ff5
	 */
	while ((err = http_srv_init(&srv, &cfg)) != ERR_OK)
		HTTP_LOG_ERROR("http_init: %d\n", err);
	HTTP_LOG_INFO("http started");
	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);

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
