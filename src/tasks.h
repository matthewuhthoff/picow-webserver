/*
 * Copyright (c) 2024 Geoff Simmons <geoff@simmons.de>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * See LICENSE
 */

#include "pico/sync.h"

void initiate_temp(void *params);
void initiate_rssi(void *params);
void initiate_http(void *params);

void main_init(void);

extern semaphore_t linkup;
