/*
 * Copyright (c) 2022 Geoff Simmons <geoff@simmons.de>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * See LICENSE
 */

/*
 * The client-side logic of the app is responsible for:
 *
 * - populating the UI with data retrieved from the server
 *
 * - converting the temperature scale preferred by the user
 *
 * - periodically updating measurements from the server side
 *   (temperature and signal strength)
 */

/*
 * Intervals at which update requests for /temp and /rssi are sent
 * (staggered a bit).
 */
const TEMP_UPDATE_INTVL_MS = 2 * 2718;
const RSSI_UPDATE_INTVL_MS = 3 * 3141;
const PIC_UPDATE_INTVL_MS = 10 * 1000;

/*
 * The preferred temperature scale (K, C, F) is stored in browser
 * localStorage under this key. Then the last scale choice is
 * "remembered" when the app is displayed again, including after
 * browser restart.
 */
const SCALE_PREF_KEY = "scalePref";
const DEFAULT_SCALE_PREF = 'K';

/*
 * The two global variables are current temperature scale preference
 * (K, C, F) and the latest temperature reading from the server.
 */
let scalePref = DEFAULT_SCALE_PREF;
let tempK = 0.0;

/* Cache document elements */
const tempValElem = document.getElementById("tempValue");
const tempScaleElem = document.getElementById("tempScale");
const scaleBtnK = document.getElementById("scaleK");
const scaleBtnC = document.getElementById("scaleC");
const scaleBtnF = document.getElementById("scaleF");

/*
 * Map a temperature scale to the button that represents it.
 * Used to change the style of the button for the chosen scale.
 */
const scale2Btn = {
    "K": scaleBtnK,
    "C": scaleBtnC,
    "F": scaleBtnF,
};

const ledBoxElem = document.getElementById("ledBox");
const ledStateElem = document.getElementById("ledState");
const ledBtnOn = document.getElementById("ledBtnOn");
const ledBtnOff = document.getElementById("ledBtnOff");
const ledBtnToggle = document.getElementById("ledBtnToggle");

const infoTextElem = document.getElementById("info_text");
const rssiTextElem = document.getElementById("rssi_text");
const hostElem = document.getElementById("host");
const ipElem = document.getElementById("ip");
const macElem = document.getElementById("mac");
const ssidElem = document.getElementById("ssid");
const rssiElem = document.getElementById("rssi");
const ssElem = document.getElementById("ss_pct");

const toastElem = document.getElementById("toast");
const errMsgElem = document.getElementById("errMsg");

/*
 * Error messages appear in the "toast" element for this long,
 * and then are hidden.
 */
const TOAST_INTVL_MS = 5000;
let toastTimeout = -1;

/*
 * Common error message function
 * The "toast" element is hidden by default. Fill its contents with
 * the error message, display the element, and start a timer to hide
 * it again.
 */
function toast(err) {
    if (toastTimeout != -1) {
        clearTimeout(toastTimeout);
    }
    errMsgElem.textContent = err.message;
    toastElem.style.display = "block";
    toastTimeout = setTimeout(() => {
        toastElem.style.display = "none";
    }, TOAST_INTVL_MS);
}

/* Common function for fetching the response for a URL path. */
async function getResp(url) {
    let response = await fetch(url);
    if (!response.ok)
        throw new Error(url + " response status: " + response.status);
    return response;
}

/*
 * Render the temperature value, given the temperature in Kelvin and a
 * chosen scale. Scale conversion is done here. The value at the
 * chosen scale is rounded to the nearest integer.
 */
function renderTemp(tmpK, scale) {
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
        throw new Error("temperature scale: " + scale);
    }
    tempValElem.textContent = tempVal.toString();
    tempScaleElem.textContent = scale;
}

/*
 * Update the temperature value.
 *
 * If the browser window is currently not visible, do nothing.
 *
 * Otherwise fetch the value from the /temp request. If the fetch is
 * successful, then the response body is the temperature in degrees
 * Kelvin as a Q18.14 fixed-point number (18 integer bits and 14
 * fractional bits). Scale it down, and call renderTemp() to display
 * the value.
 */
async function updateTemp() {
    if (document.visibilityState === "hidden") {
        return;
    }

    try {
        let response = await getResp("/temp");
        let body = await response.text();
        let tempK_q18_14 = parseInt(body);
        if (tempK_q18_14 == NaN)
            throw new Error("/temp response body: " + body);
        tempK = tempK_q18_14 / 16384.0;
        renderTemp(tempK, scalePref);
    }
    catch (ex) {
        toast(ex);
    }
}

/*
 * Change the current temperature scale preference, and store the
 * preference in browser localStorage.
 */
function updateScalePref(pref) {
    scalePref = pref;
    localStorage.setItem(SCALE_PREF_KEY, pref);
}

/*
 * Actions performed when a temperature scale is chosen.
 *
 * If the current preferred scale is unchanged, do nothing.
 *
 * Otherwise change the styles of the scale buttons -- the
 * currently chosen scale button becomes "plain", and the
 * newly chosen one becomes "active".
 *
 * Update the scale preference, and render the changed temperature
 * value.
 */
function doScale(scale) {
    if (scale == scalePref)
        return;

    let btn = scale2Btn[scale];
    let prevBtn = scale2Btn[scalePref];
    prevBtn.classList.replace("scaleActive", "btnPlain");
    btn.classList.replace("btnPlain", "scaleActive");
    updateScalePref(scale);

    try {
        renderTemp(tempK, scalePref);
    }
    catch (err) {
        toast(err);
    }
}

/* Event listeners for the Kelvin, Celsius and Fahrenheit buttons */
function doScaleK() {
    doScale("K");
}

function doScaleC() {
    doScale("C");
}

function doScaleF() {
    doScale("F");
}

/*
 * Actions performed for LED state update and controls.
 *
 * Fetch the response for the given URL (which may have a query string
 * for turning the LED on or off). If the fetch is successful, then
 * the response body is a bit -- 0 or 1 for currently off or on.
 *
 * Update the LED box to reflect the state -- change the text contents
 * and style.
 */
async function doLed(url) {
    let ledBit = NaN;
    try {
        let response = await getResp(url);
        body = await response.text();
        ledBit = parseInt(body);
        if (ledBit == NaN)
            throw new Error(url + " response body: " + body);
    }
    catch (ex) {
        toast(ex);
        return;
    }

    let ledOn = Boolean(ledBit);
    if (ledOn) {
        ledStateElem.textContent = "ON";
        ledBoxElem.classList.remove("ledOff");
        ledBoxElem.classList.add("ledOn");
        return;
    }
    ledStateElem.textContent = "OFF";
    ledBoxElem.classList.remove("ledOn");
    ledBoxElem.classList.add("ledOff");
}

/* Called at startup to get the current state of the LED. */
async function updateLed() {
    await doLed("/led");
}

/* Event listeners for the LED buttons. */
async function ledDoOn() {
    await doLed("/led?op=on");
}

async function ledDoOff() {
    await doLed("/led?op=off");
}

async function ledDoToggle() {
    await doLed("/led?op=toggle");
}

/*
 * Called at startup to get network information displayed in the UI.
 *
 * Fetch the response for /netinfo. If the fetch is successful, the
 * response body is a JSON document with fields for the hostname,
 * IP address, MAC address, and the SSID of the currently connected
 * access point. Fill in the UI's text elements with these data.
 *
 * These contents are mostly static, and cacheable, so we only
 * fetch the data once at initialization, and on page reload.
 */
async function updateNetinfo() {
    let data = null;
    try {
        let response = await getResp("/netinfo");
        data = await response.json();
    }
    catch (ex) {
        toast(ex);
        return;
    }

    hostElem.textContent = data.host;
    ipElem.textContent = data.ip;
    macElem.textContent = data.mac;
    ssidElem.textContent = data.ssid;
    infoTextElem.style.display = "inline";
}

/*
 * Update the measurement of the access point's signal strength.
 *
 * If the browser window is currently not visible, do nothing.
 *
 * Otherwise fetch the response for /rssi. If the fetch is successful,
 * the response body is a JSON document with the rssi in dBm, and a boolean
 * field indicating whether the rssi is valid.
 *
 * If the value is not valid, do nothing. Otherwise fill the UI element
 * for the rssi, as well as for the signal strength in percent.
 *
 * We use a common formula: %strength = 2 * (100 + rssi)
 * capped to the range 0 to 100.
 */
async function updateRssi() {
    if (document.visibilityState === "hidden") {
        return;
    }

    let data = null;
    try {
        let response = await getResp("/rssi");
        data = await response.json();
    }
    catch (ex) {
        toast(ex);
        return;
    }

    if (data.valid) {
        rssiElem.textContent = data.rssi.toString();
        ssElem.textContent
            = Math.min(Math.max(2 * (100 + data.rssi), 0), 100).toString();
        rssiTextElem.style.display = "inline";
    }
}

async function refreshImage() {
    if (document.visibilityState === "hidden") {
        return;
    }
    const img = document.getElementById('selfie');
    const timestamp = new Date().getTime();
    img.src= `/picture?cb=${timestamp}`;
}
/*
 * Event listener for visibility changes. If the browser window
 * becomes visible, initiate updates for the temperature and signal
 * strength.
 */
async function updateOnVisible() {
    if (document.visibilityState === "hidden") {
        return;
    }

    await updateTemp();
    await updateRssi();
}

/*
 * At initialization:
 *
 * - initiate fetches for the temperature, LED state, network info, and
 *   the signal strength
 *
 * - get the preferred temperature scale from browser localStorage,
 *   or set the default if it's never been stored
 *
 * - set event listeners for visibility changes, the temperature scale
 *   buttons, and the LED buttons
 *
 * - start periodic updates for temperature and signal strength
 */
async function init() {
    await updateTemp();
    let scale = localStorage.getItem(SCALE_PREF_KEY);
    if (scale == null) {
        scale = DEFAULT_SCALE_PREF;
    }
    doScale(scale);
    await updateLed();
    await updateNetinfo();
    await updateRssi();

    document.addEventListener("visibilitychange", updateOnVisible);

    scaleBtnK.addEventListener("click", doScaleK);
    scaleBtnC.addEventListener("click", doScaleC);
    scaleBtnF.addEventListener("click", doScaleF);

    ledBtnOn.addEventListener("click", ledDoOn);
    ledBtnOff.addEventListener("click", ledDoOff);
    ledBtnToggle.addEventListener("click", ledDoToggle);

    setInterval(updateTemp, TEMP_UPDATE_INTVL_MS);
    setInterval(updateRssi, RSSI_UPDATE_INTVL_MS);
    setInterval(refreshImage, PIC_UPDATE_INTVL_MS);
}

/* Run initialization when the document has been loaded. */
if (document.readyState !== "loading") {
    init();
} else {
    document.addEventListener("DOMContentLoaded", init);
}
