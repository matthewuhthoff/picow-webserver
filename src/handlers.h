/*
 * Copyright (c) 2022 Geoff Simmons <geoff@simmons.de>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * See LICENSE
 */

#include "lwip/ip_addr.h"
#include "picow_http/http.h"

#include <stdint.h>

#define MAC_ADDR_LEN (sizeof("01:02:03:04:05:06"))

/*
 * struct for information used by the /netinfo custom handler that is
 * not known until runtime -- the PicoW's IP address and MAC address.
 *
 * Using the "magic number" idiom, so that a pointer to an instance of the
 * struct can be checked for validity with macros from
 * picow_http/assertion.h
 *
 * See: https://slimhazard.gitlab.io/picow_http/group__assert.html
 */
struct netinfo {
    unsigned magic;
#define NETINFO_MAGIC (0x4f5dde9f)
    char ip[IPADDR_STRLEN_MAX];
    char mac[MAC_ADDR_LEN];
};

/*
 * Return the most recent temperature sensor reading in degrees Kelvin as
 * a fixed-point Q18.14 number; i.e. 18 integer bits and 14 fractional
 * bits.
 *
 * Return UINT32_MAX on error: the ADC error bit was set, or no ADC
 * reading has been stored.
 */
uint32_t get_temp(void);

/*
 * Return the most recent rssi value for "our" access point, or INT32_MAX
 * if no rssi value has been read.
 */
int32_t get_rssi(void);

/*
 * Custom response handlers for the URL paths:
 * /temp
 * /led
 * /rssi
 * /netinfo
 *
 * Custom handler functions must satisfy typedef hndlr_f from
 * picow_http/http.h
 *
 * See: https://slimhazard.gitlab.io/picow_http/group__resp.html#ga23afab92dd579b34f1190006b6fa1132
 */
err_t temp_handler(struct http* http, void* p);
err_t led_handler(struct http* http, void* p);
err_t picture_handler(struct http* http, void* p);
err_t rssi_handler(struct http* http, void* p);
err_t netinfo_handler(struct http* http, void* p);
