/*
 * Copyright (c) 2024 Geoff Simmons <geoff@simmons.de>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * See LICENSE
 */

/*
 * Common declarations for main() code that initializes the application.
 * Used by both the FreeRTOS and non-FreeRTOS implementations; the latter
 * are threadsafe background mode and poll mode.
 *
 * The initialization functions have the signatures required for a
 * FreeRTOS task. For simplicity, we use the same functions for the
 * non-FreeRTOS modes, even though the parameter may be unnecessary.
 */

#include "pico/sync.h"

/*
 * initiate_temp() initializes the ADC for use of the on-die temperature
 * sensor, initializes the ADC FIFO and an interrupt handler to run when
 * data is written to the FIFO, and starts the ADC in free-running mode.
 */
void initiate_temp(void* params);

/*
 * initiate_rssi() takes the linkup semaphore as a parameter. It starts an
 * async_context at-time worker that periodically gets the rssi value for
 * the signal strength of the AP connection. Before that, it waits for the
 * semaphore to be signaled, after the wifi connection has reached linkup
 * status; because rssi values are invalid until then.
 */
void initiate_rssi(void* params);

/*
 * initiate_picture is the freeRTOS thread running the camera
 */
void initiate_picture(void* params);

/*
 * initiate_http() initializes networking in station mode, and gets a
 * connection with the AP defined by the WIFI_SSID and WIFI_PASSWORD build
 * parameters. When the connection has reached the linkup state, it
 * signals the linkup semaphore.
 *
 * Then it saves information about the station IP address and the PicoW's
 * mac address, which will be returned in responses to the /netinfo
 * endpoint.
 *
 * Next it configures an http server, registers custom response handlers,
 * and starts the server by calling http_srv_init(). Finally it turns on
 * the onboard LED.
 */
void initiate_http(void* params);

/*
 * main_init() runs initialization code common to both non-FreeRTOS and
 * FreeRTOS implementations. In the FreeRTOS version, this is run before
 * the tasks are defined and the task scheduler is started. Including:
 * define binary info; initialize UART for log output; initialize the
 * semaphore; and reset core1.
 */
void main_init(void);

/*
 * The linkup semaphore is signaled when the wifi connections reaches the
 * linkup state. initiate_rssi() waits for it, and initiate_http() signals
 * it.
 */
extern semaphore_t linkup;
