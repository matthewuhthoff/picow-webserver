/*
 * Copyright (c) 2022 Geoff Simmons <geoff@simmons.de>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * See LICENSE
 */

const TEMP_UPDATE_INTVL_MS = 2000;
const AP_UPDATE_INTVL_MS = 5000;

let scale = 'K';
const tempValElem = document.getElementById("tempValue");

async function updateTemp() {
    let tempVal = 0;
    let body = '';
    try {
        let response = await fetch("/temp");
        if (!response.ok) {
            /* XXX toast */
            console.log("/temp status: " + response.status);
            return;
        }
        body = await response.text();
    }
    catch (ex) {
        /* XXX toast */
        console.log(ex);
        return;
    }

    let tempK_q20_12 = parseInt(body);
    if (tempK_q20_12 == NaN) {
        /* XXX toast */
        console.log("/temp response body: " + body);
        return;
    }
    let tempK = tempK_q20_12 / 4096.0;

    switch (scale) {
    case 'K':
        tempVal = Math.round(tempK);
        break;
    case 'C':
        tempVal = Math.round(tempK - 273.15);
        break;
    case 'F':
        tempVal = Math.round(1.8 * (tempK - 273.15) + 32);
        break;
    default:
        /* XXX toast */
        console.log("/temp scale: " + scale);
        return;
    }
    tempValElem.textContent = tempVal.toString();
}

function init() {
    updateTemp();
}

if (document.readyState !== "loading") {
    init();
} else {
    document.addEventListener("DOMContentLoaded", init);
}
