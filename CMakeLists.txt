cmake_minimum_required(VERSION 3.12)

# Since SDK version 2.0.0, PICO_BOARD must be set prior to the next
# include() (also works for SDK 1.5.1).
set(PICO_BOARD pico_w)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# From pico-sdk, to integrate the SDK into the project.
include(pico_sdk_import.cmake)

# Creates targets for use with a picoprobe, to flash the code and
# reset the app. This requires that the locations of openocd and
# optionally of its search path are in environment variables, or
# passed to cmake as -D definitions. See the included file for
# details.
include(picoprobe_targets.cmake)

# FreeRTOS_Kernel_import.cmake configures FreeRTOS integration if
# FREERTOS_KERNEL_PATH is set either as a cmake variable or in the
# environment. Here we set it directly to point to the submodule.
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/FreeRTOS-Kernel)
include(${CMAKE_CURRENT_LIST_DIR}/FreeRTOS_Kernel_import.cmake)

project(picow-http-example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# picow-http is included in this project as a git submodule, so it can
# be integrated with add_subdirectory().
#
# If instead you have picow-http installed elsewhere on your system,
# use include():
#
# include(path/to/picow-http)
#
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/picow-http)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/Arducam)

# For use with a picoprobe (this has nothing to do with picow-http).
# 'make reset' restarts the app.
picoprobe_add_reset_target()

# If NTP_SERVERS is defined in the cmake invocation, pass in its value
# to the preprocessor. See main.c
if (DEFINED NTP_SERVERS)
	add_compile_definitions(NTP_SERVERS=\"${NTP_SERVERS}\")
endif()

# Optionally override the PicoW default hostname.
if (DEFINED HOSTNAME)
	add_compile_definitions(CYW43_HOST_NAME=\"${HOSTNAME}\")
endif()

# This project has targets for more than one executable, which differ
# in the choice of network architecture (see the SDK docs for
# pico_cyw43_arch), and in TLS support. Most apps will choose just one
# network architecture, and whether or not to support TLS, but the
# example includes all of the combinations for demonstration purposes.
#
# The following definitions summarize elements that the different
# excutables have in common.

# Common source files
# We use the sample lwipopts.h definition from picow-http.
# See: https://gitlab.com/slimhazard/picow_http/-/wikis/Configuring-lwipopts.h
set(SRCS
	${CMAKE_CURRENT_LIST_DIR}/src/tasks.c
	${CMAKE_CURRENT_LIST_DIR}/src/tasks.h
	${CMAKE_CURRENT_LIST_DIR}/src/handlers.c
	${CMAKE_CURRENT_LIST_DIR}/src/handlers.h
	${CMAKE_CURRENT_LIST_DIR}/lib/picow-http/etc/lwipopts.h
)

# Include paths
# Must include the path at which lwipopts.h and is located.
set(INCLUDES
	${CMAKE_CURRENT_LIST_DIR}/src
	${CMAKE_CURRENT_LIST_DIR}/lib/picow-http/etc
  ${CMAKE_CURRENT_LIST_DIR}/lib/Arducam
)

# Static WWW resources to be embedded for the HTTP server.
set(WWWSRCS
	${CMAKE_CURRENT_LIST_DIR}/www/index.html
	${CMAKE_CURRENT_LIST_DIR}/www/picow.css
	${CMAKE_CURRENT_LIST_DIR}/www/img/favicon.png
	${CMAKE_CURRENT_LIST_DIR}/www/sample_app.js
)

# Libraries to be linked
# The different versions link either picow_http (without TLS support)
# or picow_https (with TLS), and different choices for
# pico_cyw43_arch_lwip_*, otherwise the libs are the same.
set(LIBS
	pico_stdio
	pico_stdlib
	pico_multicore
	hardware_adc
	hardware_irq
	hardware_sync
  libarducam
)

# The next sections configure the executables; mostly standard for the
# SDK. The picow_http_gen_handlers() directive is required for use
# with picow-http.

##
## threadsafe background mode, without TLS
##

add_executable(picow-http-example-background
	${CMAKE_CURRENT_LIST_DIR}/src/main-non-freertos.c
	${SRCS}
)

# PICOW_HTTPS indicates TLS support (used for binary info).
target_compile_definitions(picow-http-example-background PRIVATE
	WIFI_SSID=\"${WIFI_SSID}\"
	WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
	PICOW_HTTPS=0
)

target_include_directories(picow-http-example-background PRIVATE ${INCLUDES})

# picow_http links the HTTP server without TLS support.
# pico_cyw43_arch_lwip_threadsafe_background sets the network
# architecture.
target_link_libraries(picow-http-example-background
	picow_http
	pico_cyw43_arch_lwip_threadsafe_background
	${LIBS}
)

# The arguments for picow_http_gen_handlers() are:
# - target name
# - YAML file that configures static resources and custom response
#   handlers
# - Directory that contains the static resources
# - List of static resources; here defined in WWWSRCS above.
# See: https://gitlab.com/slimhazard/picow_http/-/wikis/picow_http_gen_handlers
picow_http_gen_handlers(picow-http-example-background
	${CMAKE_CURRENT_SOURCE_DIR}/www/www.yaml
	${CMAKE_CURRENT_SOURCE_DIR}/www
	${WWWSRCS}
)

# This defines UART, but not USB, as the destination for the HTTP
# server's log output.
pico_enable_stdio_usb(picow-http-example-background 0)
pico_enable_stdio_uart(picow-http-example-background 1)

pico_add_extra_outputs(picow-http-example-background)

# For use with a picoprobe.
# 'make flash-picow-http-example-background' loads the binary to the
# PicoW.
picoprobe_add_flash_target(picow-http-example-background)

pico_set_program_description(picow-http-example-background
	"example app for the picow-http library"
)
pico_set_program_url(picow-http-example-background
	"https://gitlab.com/slimhazard/picow-http-example"
)

##
## poll mode, without TLS
##

add_executable(picow-http-example-poll
	${CMAKE_CURRENT_LIST_DIR}/src/main-non-freertos.c
	${SRCS}
)

target_compile_definitions(picow-http-example-poll PRIVATE
	WIFI_SSID=\"${WIFI_SSID}\"
	WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
	PICOW_HTTPS=0
)

target_include_directories(picow-http-example-poll PRIVATE ${INCLUDES})

# pico_cyw43_arch_lwip_poll sets the network architecture tp poll
# mode.
target_link_libraries(picow-http-example-poll
	picow_http
	pico_cyw43_arch_lwip_poll
	${LIBS}
)

# Exactly as above
picow_http_gen_handlers(picow-http-example-poll
 	${CMAKE_CURRENT_SOURCE_DIR}/www/www.yaml
 	${CMAKE_CURRENT_SOURCE_DIR}/www
 	${WWWSRCS}
)

pico_enable_stdio_usb(picow-http-example-poll 0)
pico_enable_stdio_uart(picow-http-example-poll 1)

pico_add_extra_outputs(picow-http-example-poll)

picoprobe_add_flash_target(picow-http-example-poll)

pico_set_program_description(picow-http-example-poll
	"example app for the picow-http library"
)
pico_set_program_url(picow-http-example-poll
	"https://gitlab.com/slimhazard/picow-http-example"
)

##
## threadsafe background mode, with TLS
##

# For TLS, mbedtls_config.h must be included. We use the sample in the
# picow-http's etc/ directory. The etc/ directory is on the INCLUDES
# list defined above.
set(HTTPS_SRCS
	${SRCS}
	${CMAKE_CURRENT_LIST_DIR}/lib/picow-http/etc/mbedtls_config.h
)

# For TLS, add the server certificate and private key to the WWW
# sources.
set(HTTPS_WWWSRCS
	${WWWSRCS}
	${CMAKE_CURRENT_LIST_DIR}/www/crt/picow-sample.crt
	${CMAKE_CURRENT_LIST_DIR}/www/crt/picow-sample.key
)

add_executable(picow-https-example-background
	${CMAKE_CURRENT_LIST_DIR}/src/main-non-freertos.c
	${HTTPS_SRCS}
)

target_compile_definitions(picow-https-example-background PRIVATE
	WIFI_SSID=\"${WIFI_SSID}\"
	WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
	PICOW_HTTPS=1
)

target_include_directories(picow-https-example-background PRIVATE ${INCLUDES})

# Link picow-https for TLS support.
target_link_libraries(picow-https-example-background
	picow_https
	pico_cyw43_arch_lwip_threadsafe_background
	${LIBS}
)

# For TLS, we use a version of the WWW yaml that includes a
# specification of the certificate and key. The YAML also specifies
# brotli compression for compressible static resources, since common
# browsers accept brotli in responses sent over TLS connections.
picow_http_gen_handlers(picow-https-example-background
	${CMAKE_CURRENT_SOURCE_DIR}/www/www_https.yaml
	${CMAKE_CURRENT_SOURCE_DIR}/www
	${WWWSRCS}
)

pico_enable_stdio_usb(picow-https-example-background 0)
pico_enable_stdio_uart(picow-https-example-background 1)

pico_add_extra_outputs(picow-https-example-background)

picoprobe_add_flash_target(picow-https-example-background)

pico_set_program_description(picow-https-example-background
	"example app for the picow-http library"
)
pico_set_program_url(picow-https-example-background
	"https://gitlab.com/slimhazard/picow-http-example"
)

##
## poll mode, with TLS
##

add_executable(picow-https-example-poll
	${CMAKE_CURRENT_LIST_DIR}/src/main-non-freertos.c
	${HTTPS_SRCS}
)

target_compile_definitions(picow-https-example-poll PRIVATE
	WIFI_SSID=\"${WIFI_SSID}\"
	WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
	PICOW_HTTPS=1
)

target_include_directories(picow-https-example-poll PRIVATE ${INCLUDES})

# Link picow-https for TLS support.
target_link_libraries(picow-https-example-poll
	picow_https
	pico_cyw43_arch_lwip_poll
	${LIBS}
)

picow_http_gen_handlers(picow-https-example-poll
	${CMAKE_CURRENT_SOURCE_DIR}/www/www_https.yaml
	${CMAKE_CURRENT_SOURCE_DIR}/www
	${WWWSRCS}
)

pico_enable_stdio_usb(picow-https-example-poll 0)
pico_enable_stdio_uart(picow-https-example-poll 1)

pico_add_extra_outputs(picow-https-example-poll)

picoprobe_add_flash_target(picow-https-example-poll)

pico_set_program_description(picow-https-example-poll
	"example app for the picow-http library"
)
pico_set_program_url(picow-https-example-poll
	"https://gitlab.com/slimhazard/picow-http-example"
)

##
## Note: we only build FreeRTOS sample apps with NO_SYS=0, or sys
## mode. NO_SYS=1 (or nosys) is currently not supported by picow-http.
## NO_SYS=0 is set by default for FreeRTOS builds in picow-http's
## sample lwipopts.h.
##

##
## SMP FreeRTOS in sys mode, without TLS
##
add_executable(picow-http-example-freertos-sys-smp
	${CMAKE_CURRENT_LIST_DIR}/src/main-freertos.c
	${SRCS}
)

target_compile_definitions(picow-http-example-freertos-sys-smp PRIVATE
	WIFI_SSID=\"${WIFI_SSID}\"
	WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
	PICOW_HTTPS=0
	HAVE_FREERTOS=1
)

target_include_directories(picow-http-example-freertos-sys-smp PRIVATE
	${INCLUDES}
)

target_link_libraries(picow-http-example-freertos-sys-smp
	picow_http
	pico_cyw43_arch_lwip_sys_freertos
	${LIBS}
	FreeRTOS-Kernel-Heap4
)

picow_http_gen_handlers(picow-http-example-freertos-sys-smp
	${CMAKE_CURRENT_SOURCE_DIR}/www/www.yaml
	${CMAKE_CURRENT_SOURCE_DIR}/www
	${WWWSRCS}
)

pico_enable_stdio_usb(picow-http-example-freertos-sys-smp 0)
pico_enable_stdio_uart(picow-http-example-freertos-sys-smp 1)

pico_add_extra_outputs(picow-http-example-freertos-sys-smp)

picoprobe_add_flash_target(picow-http-example-freertos-sys-smp)

pico_set_program_description(picow-http-example-freertos-sys-smp
	"example app for the picow-http library"
)
pico_set_program_url(picow-http-example-freertos-sys-smp
	"https://gitlab.com/slimhazard/picow-http-example"
)

##
## SMP FreeRTOS in sys mode, with TLS
##
add_executable(picow-https-example-freertos-sys-smp
	${CMAKE_CURRENT_LIST_DIR}/src/main-freertos.c
	${HTTPS_SRCS}
)

target_compile_definitions(picow-https-example-freertos-sys-smp PRIVATE
	WIFI_SSID=\"${WIFI_SSID}\"
	WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
	PICOW_HTTPS=1
	HAVE_FREERTOS=1
)

target_include_directories(picow-https-example-freertos-sys-smp PRIVATE
	${INCLUDES}
)

target_link_libraries(picow-https-example-freertos-sys-smp
	picow_https
	pico_cyw43_arch_lwip_sys_freertos
	${LIBS}
	FreeRTOS-Kernel-Heap4
)

picow_http_gen_handlers(picow-https-example-freertos-sys-smp
	${CMAKE_CURRENT_SOURCE_DIR}/www/www_https.yaml
	${CMAKE_CURRENT_SOURCE_DIR}/www
	${WWWSRCS}
)

pico_enable_stdio_usb(picow-https-example-freertos-sys-smp 0)
pico_enable_stdio_uart(picow-https-example-freertos-sys-smp 1)

pico_add_extra_outputs(picow-https-example-freertos-sys-smp)

picoprobe_add_flash_target(picow-https-example-freertos-sys-smp)

pico_set_program_description(picow-https-example-freertos-sys-smp
	"example app for the picow-http library"
)
pico_set_program_url(picow-https-example-freertos-sys-smp
	"https://gitlab.com/slimhazard/picow-http-example"
)

##
## Single-core FreeRTOS in sys mode, without TLS
##
add_executable(picow-http-example-freertos-sys-single
	${CMAKE_CURRENT_LIST_DIR}/src/main-freertos.c
	${SRCS}
)

target_compile_definitions(picow-http-example-freertos-sys-single PRIVATE
	WIFI_SSID=\"${WIFI_SSID}\"
	WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
	PICOW_HTTPS=0
	HAVE_FREERTOS=1
	configNUMBER_OF_CORES=1
)

target_include_directories(picow-http-example-freertos-sys-single PRIVATE
	${INCLUDES}
)

target_link_libraries(picow-http-example-freertos-sys-single
	picow_http
	pico_cyw43_arch_lwip_sys_freertos
	${LIBS}
	FreeRTOS-Kernel-Heap4
)

picow_http_gen_handlers(picow-http-example-freertos-sys-single
	${CMAKE_CURRENT_SOURCE_DIR}/www/www.yaml
	${CMAKE_CURRENT_SOURCE_DIR}/www
	${WWWSRCS}
)

pico_enable_stdio_usb(picow-http-example-freertos-sys-single 0)
pico_enable_stdio_uart(picow-http-example-freertos-sys-single 1)

pico_add_extra_outputs(picow-http-example-freertos-sys-single)

picoprobe_add_flash_target(picow-http-example-freertos-sys-single)

pico_set_program_description(picow-http-example-freertos-sys-single
	"example app for the picow-http library"
)
pico_set_program_url(picow-http-example-freertos-sys-single
	"https://gitlab.com/slimhazard/picow-http-example"
)

##
## Single-core FreeRTOS in sys mode, with TLS
##
add_executable(picow-https-example-freertos-sys-single
	${CMAKE_CURRENT_LIST_DIR}/src/main-freertos.c
	${HTTPS_SRCS}
)

target_compile_definitions(picow-https-example-freertos-sys-single PRIVATE
	WIFI_SSID=\"${WIFI_SSID}\"
	WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
	PICOW_HTTPS=1
	HAVE_FREERTOS=1
	configNUMBER_OF_CORES=1
)

target_include_directories(picow-https-example-freertos-sys-single PRIVATE
	${INCLUDES}
)

target_link_libraries(picow-https-example-freertos-sys-single
	picow_https
	pico_cyw43_arch_lwip_sys_freertos
	${LIBS}
	FreeRTOS-Kernel-Heap4
)

picow_http_gen_handlers(picow-https-example-freertos-sys-single
	${CMAKE_CURRENT_SOURCE_DIR}/www/www_https.yaml
	${CMAKE_CURRENT_SOURCE_DIR}/www
	${WWWSRCS}
)

pico_enable_stdio_usb(picow-https-example-freertos-sys-single 0)
pico_enable_stdio_uart(picow-https-example-freertos-sys-single 1)

pico_add_extra_outputs(picow-https-example-freertos-sys-single)

picoprobe_add_flash_target(picow-https-example-freertos-sys-single)

pico_set_program_description(picow-https-example-freertos-sys-single
	"example app for the picow-http library"
)
pico_set_program_url(picow-https-example-freertos-sys-single
	"https://gitlab.com/slimhazard/picow-http-example"
)

##
## "no-sys" FreeRTOS: with threadsafe background architecture and NO_SYS=1.
## Only supported since SDK version 2.0.0.
##
if (${PICO_SDK_VERSION_STRING} VERSION_GREATER_EQUAL "2.0.0")
	# Without TLS
	add_executable(picow-http-example-freertos-nosys
		${CMAKE_CURRENT_LIST_DIR}/src/main-freertos.c
		${SRCS}
	)

	target_compile_definitions(picow-http-example-freertos-nosys PRIVATE
		WIFI_SSID=\"${WIFI_SSID}\"
		WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
		HAVE_FREERTOS=1
		NO_SYS=1
	)

	target_include_directories(picow-http-example-freertos-nosys PRIVATE
		${INCLUDES}
	)

	target_link_libraries(picow-http-example-freertos-nosys
		picow_http
		pico_cyw43_arch_lwip_threadsafe_background
		${LIBS}
		FreeRTOS-Kernel-Heap4
	)

	picow_http_gen_handlers(picow-http-example-freertos-nosys
		${CMAKE_CURRENT_SOURCE_DIR}/www/www.yaml
		${CMAKE_CURRENT_SOURCE_DIR}/www
		${WWWSRCS}
	)

	pico_enable_stdio_usb(picow-http-example-freertos-nosys 1)
	pico_enable_stdio_uart(picow-http-example-freertos-nosys 0)

	pico_add_extra_outputs(picow-http-example-freertos-nosys)

	picoprobe_add_flash_target(picow-http-example-freertos-nosys)

	pico_set_program_description(picow-http-example-freertos-nosys
		"example app for the picow-http library"
	)
	pico_set_program_url(picow-http-example-freertos-nosys
		"https://gitlab.com/slimhazard/picow-http-example"
	)

	# With TLS
	add_executable(picow-https-example-freertos-nosys
		${CMAKE_CURRENT_LIST_DIR}/src/main-freertos.c
		${HTTPS_SRCS}
	)

	target_compile_definitions(picow-https-example-freertos-nosys PRIVATE
		WIFI_SSID=\"${WIFI_SSID}\"
		WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
		PICOW_HTTPS=1
		HAVE_FREERTOS=1
		NO_SYS=1
	)

	target_include_directories(picow-https-example-freertos-nosys PRIVATE
		${INCLUDES}
	)

	target_link_libraries(picow-https-example-freertos-nosys
		picow_https
		pico_cyw43_arch_lwip_threadsafe_background
		${LIBS}
		FreeRTOS-Kernel-Heap4
	)

	picow_http_gen_handlers(picow-https-example-freertos-nosys
		${CMAKE_CURRENT_SOURCE_DIR}/www/www_https.yaml
		${CMAKE_CURRENT_SOURCE_DIR}/www
		${WWWSRCS}
	)

	pico_enable_stdio_usb(picow-https-example-freertos-nosys 0)
	pico_enable_stdio_uart(picow-https-example-freertos-nosys 1)

	pico_add_extra_outputs(picow-https-example-freertos-nosys)

	picoprobe_add_flash_target(picow-https-example-freertos-nosys)

	pico_set_program_description(picow-https-example-freertos-nosys
		"example app for the picow-http library"
	)
	pico_set_program_url(picow-https-example-freertos-nosys
		"https://gitlab.com/slimhazard/picow-http-example"
	)
endif()

##
## Use the output of 'git describe', if available, as the version
## string
##
set(PROGRAM_VERSION "unknown")
find_program(GIT git)
if (NOT GIT_FOUND)
	execute_process(COMMAND ${GIT} describe --always
		OUTPUT_VARIABLE GIT_DESCRIBE_OUTPUT
		OUTPUT_STRIP_TRAILING_WHITESPACE
		RESULT_VARIABLE GIT_DESCRIBE_RESULT
	)
	if (GIT_DESCRIBE_RESULT EQUAL "0")
		set(PROGRAM_VERSION ${GIT_DESCRIBE_OUTPUT})
	else()
		message(WARNING "git describe returned ${GIT_DESCRIBE_RESULT}")
	endif()
else()
	message(WARNING "git not found")
endif()
message("version: ${PROGRAM_VERSION}")

pico_set_program_version(picow-http-example-background ${PROGRAM_VERSION})
pico_set_program_version(picow-http-example-poll ${PROGRAM_VERSION})
pico_set_program_version(picow-https-example-background ${PROGRAM_VERSION})
pico_set_program_version(picow-https-example-poll ${PROGRAM_VERSION})
pico_set_program_version(picow-http-example-freertos-sys-smp ${PROGRAM_VERSION})
pico_set_program_version(picow-https-example-freertos-sys-smp
                         ${PROGRAM_VERSION}
)
pico_set_program_version(picow-http-example-freertos-sys-single
                         ${PROGRAM_VERSION}
)
pico_set_program_version(picow-https-example-freertos-sys-single
                         ${PROGRAM_VERSION}
)

if (${PICO_SDK_VERSION_STRING} VERSION_GREATER_EQUAL "2.0.0")
	pico_set_program_version(picow-http-example-freertos-nosys
		${PROGRAM_VERSION}
	)
	pico_set_program_version(picow-https-example-freertos-nosys
		${PROGRAM_VERSION}
	)
endif()
