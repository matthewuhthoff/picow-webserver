# picow-http-example

picow-http-example is a sample application to demonstrate
[picow-http](https://gitlab.com/slimhazard/picow_http) -- an HTTP server
for the [Raspberry Pi Pico W
microcontroller](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html#raspberry-pi-pico-w-and-pico-wh).

The sample is a single-page application for a web browser that displays:

  * The state of the PicoW's onboard LED (on or off).
    * The user can turn on, turn off, and toggle the LED.
  * Measurements from the PicoW's onboard temperature sensor.
    * The user can choose the temperature scale (Kelvin, Celsius, or
      Fahrenheit).
  * Information about the PicoW's network connection, including the
    signal strength of the access point, displayed in percent and as
    [rssi](https://en.wikipedia.org/wiki/Received_signal_strength_indication)
    in dBm.

While the app is visible in a browser window, the temperature and
signal strength measurements are updated every few seconds.

See the [project
Wiki](https://gitlab.com/slimhazard/picow-http-example/-/wikis/Home) for
[screenshots](https://gitlab.com/slimhazard/picow-http-example/-/wikis/Screenshots)
of the app as seen in various browsers.

These are far from the most interesting things that a PicoW can do;
but the app can be deployed without any additional hardware besides
the PicoW board, or additional software libraries besides picow-http
and the
[SDK](https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html).
The example illustrates:

  * Configuring and embedding static resources: HTML, CSS, Javascript
    and a PNG image.
  * Implementing custom response handlers for dynamic content.
  * Optionally supporting TLS (https) for secure communication. The
    example can be built with or without TLS support.
    * TLS support encompasses embedding a server certificate and
      private key in flash memory.
  * Basic guidelines for separation of concerns:
    * Presentation logic on the client side (in Javascript)
    * Utilizing the browser cache
    * Separation of the HTTP server from core hardware functions on
      the PicoW

Specifically in the sample app:

  * Static resources in the [`www/`](www/) subdirectory are embedded.
  * Four [custom response handlers](src/handlers.c) are implemented for
    temperature, the LED, rssi, and network information.
  * When the build enables TLS support, the server certificate and
    private key in [`www/crt/`](www/crt/) are embedded.
  * Static resources and custom handlers are configured in
    [`www.yaml`](www/www.yaml).

For more information about the picow-http server library, see:

  * the [git repository](https://gitlab.com/slimhazard/picow_http)
  * the picow-http [project
    Wiki](https://gitlab.com/slimhazard/picow_http/-/wikis/home)
  * the [public API documentation](https://slimhazard.gitlab.io/picow_http/)

## Build/Install

### Quick start

Ensure that [requirements](#requirements) are fulfilled. Then:

```shell
$ git clone --recurse-submodules https://gitlab.com/slimhazard/picow-http-example.git
$ cd picow-http-example
$ mkdir build
$ cd build
$ cmake -DPICO_BOARD=pico_w -DWIFI_SSID=my_wifi -DWIFI_PASSWORD=wifi_pass \
        -DHOSTNAME=picow-sample ..
$ make -j
```

On successful build, [load one of the binaries](#deploying-the-app) to
your board in any of the usual ways for the PicoW.

To [test a version that supports TLS](#testing-tls-support), use the
[test CA certificate](www/crt/testca.crt) that was used to sign the
server certificate (the certificate is not self-signed).

Finally point your browser to `http://picow-sample`, if a version
without TLS support was loaded, or `https://picow-sample` for a
version with TLS. The [server log](#monitoring-the-log) can be viewed
via UART.

In the following we go over the process step by step.

### Requirements

The [Pico C
SDK](https://raspberrypi.github.io/pico-sdk-doxygen/index.html) since
version 1.5.0 and its toolchain are required to build the
app. picow-http also requires some [additional
software](https://gitlab.com/slimhazard/picow_http/-/wikis/required-software);
see the link (at the [picow-http project
Wiki](https://gitlab.com/slimhazard/picow_http/-/wikis/home)) for
details.

### Building the app

Start by cloning the repository:

```shell
# Clones the repository into a new directory 'picow-http-example',
# and updates the picow-http submodule.
$ git clone --recurse-submodules https://gitlab.com/slimhazard/picow-http-example.git
```

picow-http is a [git
submodule](https://git-scm.com/book/en/v2/Git-Tools-Submodules) in the
repo at the path [`lib/picow-http`](lib/picow-http); it is
automatically updated when the `--recurse-submodules` option is used
with the `git clone` command.

Like many projects based on the Pico SDK, this project includes [CMake
code](pico_sdk_import.cmake) that integrates the SDK into the
build. This usually requires that the path of the SDK is in the
`PICO_SDK_PATH` environment variable, or it may be passed in as a `-D`
command-line definition in the `cmake` command shown below. See the
[docs](https://datasheets.raspberrypi.com/picow/connecting-to-the-internet-with-pico-w.pdf)
for details. In the following, we will assume that `PICO_SDK_PATH` is
set in the environment.

In the project repo, create a build subdirectory, and call `cmake` in
that directory:

```shell
$ cd picow-http-example
$ mkdir build
$ cd build
$ cmake -DPICO_BOARD=pico_w -DWIFI_SSID=my_wifi -DWIFI_PASSWORD=wifi_pass \
        -DHOSTNAME=picow-sample ..
```

Three of the `-D` command-line options in the `cmake` invocation shown
above are required in any build of the sample app:

  * `PICO_BOARD=pico_w` identifies the PicoW for the SDK as the target
    hardware. This is required in any build based on the SDK that
    targets the PicoW.
    * `PICO_BOARD` is set in the sample project's `cmake` invocation to
      emphasize that is always necessary for picow-http. But it is probably
      easier to include the line `set(PICO_BOARD pico_w)` in your
      project's `CMakeLists.txt`.
  * `WIFI_SSID` and `WIFI_PASSWORD`: This project follows the PicoW
    examples in the
    [`pico-examples`](https://github.com/raspberrypi/pico-examples)
    repository by setting WiFi credentials via CMake definitions. Set
    `WIFI_SSID` to the SSID of the access point (the network name of
    your WiFi router), and `WIFI_PASSWORD` to its password.

One of the `-D` options is required to build versions of the app that
support TLS. It is required for a build that can create all versions
with or without TLS (using `make all` or just `make`), but can be left
out if only versions without TLS are to be built:

  * `HOSTNAME` sets the host name for the PicoW. For a version of the
    app with TLS support, the host name must be `picow-sample`,
    because the server certificate only verifies connections for
    that name. For a version without TLS, `HOSTNAME` may be left
    out, or set to another name. The default hostname is `PicoW`.

There is an optional parameter for `-D` on the `cmake` command line:

  * `NTP_SERVER`: the server to be used for [time
    synchronization](https://slimhazard.gitlab.io/picow_http/group__ntp.html)

The default value of `NTP_SERVER` is a generic pool; it is usually
much better to specify an NTP server or pool that is "closer" to the
location where the PicoW will be deployed. If your WiFi router
provides NTP synchronization, then that is the ideal choice.

After the `cmake` call, the software is built with `make`:

```shell
$ make -j
```

If all goes well, then you have now built binaries that are ready to
deploy.

### Deploying the app

This project builds _four_ versions of the binary:

  * `picow-http-example-background`
  * `picow-http-example-poll`
  * `picow-https-example-background`
  * `picow-https-example-poll`

These result from permutations of:

  * the two network architectures that are currently supported
    by picow-http: threadsafe background mode and poll mode; see the [SDK
    docs](https://raspberrypi.github.io/pico-sdk-doxygen/group__pico__cyw43__arch.html)
    for details. 
  * versions with or without TLS support (linking with `picow_https`
    or `picow_http`, respectively)

The different versions are functionally identical. Their code is
almost entirely the same; the different choices are realized by
configuration in `CMakeLists.txt`. Most applications will choose just
one network architecture, and make one choice about TLS support.
Since this is an example app, it demonstrates how to realize the
various possibilities.

To build specific versions, rather than all of them at once, use one
of the binary names shown above as the `make` target:

```shell
# Build the version with poll mode, and without TLS
$ make -j picow-http-example-poll
```

After a successful invocation of `make`, the build directory contains
the binary artifacts. These can be loaded on the PicoW in any of the
usual ways for a PicoW or Pico; for example by copying the `*.uf2`
file to the `RPI-RP2` USB device that is presented when the board is
in boot select mode. See the [Getting
Started](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)
document for details.

If you are using a picoprobe (see Appendix A in Getting Started), the
build environment [defines](picoprobe_targets.cmake) targets that
flash the various binaries to the PicoW:

```shell
# For use with a picoprobe

# Load the version with threadsafe background mode, without TLS support
$ make flash-picow-http-example-background

# Similarly any of:
$ make flash-picow-http-example-poll
$ make flash-picow-https-example-background
$ make flash-picow-https-example-poll
```

These commands encapsulate the `openocd` calls that load the binary code.

After loading a binary that does not include TLS support, point a
browser to `http://picow-sample` (or a URL with the `HOSTNAME` defined
in the `cmake` invocation as shown above, or `http://PicoW` if the
default host name was configured) to view the application.

See the next section about testing a version that supports TLS.

### Testing TLS support

To test a version that uses TLS, you must use a client configured with
the test [certificate
authority](https://en.wikipedia.org/wiki/Certificate_authority) (CA),
whose certificate is in this repository at
[`www/crt/testca.crt`](www/crt/testca.crt). The server certificate is
signed by the test CA (it is _not_ self-signed), so a client will only
verify the connection if it is configured to use the CA.

Configuring the CA certificate for some of the common web clients:

  * Firefox: Settings > Privacy & Security > View Certificates >
    Authorities > Import
  * Chrome: Settings > Privacy and Security > Security >
    Manage Certificates > Authorities > Import
  * curl: invoke with the `--cacert` argument set to the path of
    the CA certificate, or with `-k` to ignore TLS validation.

The client can then be pointed to `https://picow-sample` to view the
application. The hostname `picow-sample` is required, since the
certificate is only valid for that name.

Details of the server certificate and CA, such as fingerprints and
validity dates, can be displayed in most browsers, usually by clicking
the "padlock" icon and following controls such as "View Certificate".
The details should match the information shown in the [project
Wiki](https://gitlab.com/slimhazard/picow-http-example/-/wikis/Certificate-Details).

Consider removing the test CA from your browser configuration after
testing the app.

For your own application using picow-http, options for validation of a
server certificate with a CA include:

  * having the certificate signed by one of the common CAs that is
    recognized by most browsers
  * creating your own CA to sign the certificate, as is done in this
    project. This requires that clients import the CA certificate, as
    discussed above.
  * using a self-signed certificate. This is reported as a security
    violation by the common browsers, but users can choose to proceed
    nevertheless. This can only be secure if users can be relied on to
    carefully check the certificate before proceeding.

### Monitoring the log

The application is configured to use
[UART](https://raspberrypi.github.io/pico-sdk-doxygen/group__pico__stdio__uart.html)
for log output. The means of viewing UART output are platform
dependent; see the [Getting
Started](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)
document for suggestions that apply to Linux, MacOS and Windows.

When the application starts up after load, the log output is similar
to:

```
Connecting to my_wifi ...
Connected to my_wifi
http started
192.168.1.1 - - [27/Oct/2022:20:40:10 +0000] "GET / HTTP/1.1" 200 729
192.168.1.1 - - [27/Oct/2022:20:40:10 +0000] "GET /sample_app.js HTTP/1.1" 200 1497
192.168.1.1 - - [27/Oct/2022:20:40:10 +0000] "GET /picow.css HTTP/1.1" 200 506
192.168.1.1 - - [27/Oct/2022:20:40:10 +0000] "GET /temp HTTP/1.1" 200 7
192.168.1.1 - - [27/Oct/2022:20:40:10 +0000] "GET /img/favicon.png HTTP/1.1" 200 255
192.168.1.1 - - [27/Oct/2022:20:40:10 +0000] "GET /led HTTP/1.1" 200 1
192.168.1.1 - - [27/Oct/2022:20:40:10 +0000] "GET /rssi HTTP/1.1" 200 28
```

By default, request logs in [Common Log
Format](https://en.wikipedia.org/wiki/Common_Log_Format) are emitted
with dates and times in the [UTC time
zone](https://en.wikipedia.org/wiki/Coordinated_Universal_Time) (and
hence the time zone offset is always `+0000`). See the [log API
docs](https://slimhazard.gitlab.io/picow_http/group__log.html) for
details about setting the log verbosity at compile time in a
picow-http application.

### View binary info

The example app uses the [binary
info](https://raspberrypi.github.io/pico-sdk-doxygen/group__pico__binary__info.html)
facility of the SDK to store meta-information that can be viewed with the
[picotool](https://github.com/raspberrypi/picotool). That can be used, for
example, to find out the WiFi SSID for which the app was built, or the
version of the picow-http library that was linked:

```shell
$ picotool info -b -p picow-https-example-poll.bin 
File picow-https-example-poll.bin:

Program Information
 name:         picow-https-example-poll
 version:      0.5.0
 web site:     https://gitlab.com/slimhazard/picow-http-example
 description:  example app for the picow-http library
 features:     hostname: picow-sample
               AP SSID: my_wifi
               picow-http version: 0.5.0
               lwIP version: 2.2.0d
               arch: poll
               TLS: yes
               mbedtls version: 2.28.1
               UART stdout

Fixed Pin Information
 0:  UART0 TX
```

## How it works

The source code is commented (a bit verbosely) to explain the workings
in detail. See the comments in:

  * the [C source files](src/)
  * [CMakeLists.txt](CMakeLists.txt)
  * the build-time configuration in [`www.yaml`](www/www.yaml)
  * the [Javascript source](www/sample_app.js) for
    client-side/presentation logic
