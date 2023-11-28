set(PROGRAM_NAME pico_w_neopixel_server)

# TODO remove after https://github.com/georgerobotics/cyw43-driver/pull/39 gets into the SDK
# The file has code written for an older version of lwIP hidden behind `#if LWIP_MDNS_RESPONDER` clause
# so we change `LWIP_MDNS_RESPONDER` to a different `NON_EXISTANT_MACRO` to make that `#if` never be true
set(FILE_TO_CHANGE ${PICO_CYW43_DRIVER_PATH}/src/cyw43_lwip.c)
file(READ ${FILE_TO_CHANGE} FILE_CONTENTS)
string(REPLACE "LWIP_MDNS_RESPONDER" "NON_EXISTANT_MACRO" FILE_CONTENTS "${FILE_CONTENTS}")
file(WRITE ${FILE_TO_CHANGE} "${FILE_CONTENTS}")

add_executable(${PROGRAM_NAME}
        main.cpp
        )
# generate the header file into the source tree as it is included in the RP2040 datasheet
file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)
pico_generate_pio_header(pico_w_neopixel_server ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
target_compile_definitions(${PROGRAM_NAME} PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(${PROGRAM_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )
target_link_libraries(${PROGRAM_NAME}
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_http
        pico_lwip_mdns
        pico_stdlib
        hardware_adc
        hardware_pio
        )

pico_enable_stdio_usb(${PROGRAM_NAME} TRUE)
pico_enable_stdio_uart(${PROGRAM_NAME} FALSE)
suppress_tinyusb_warnings()

pico_add_extra_outputs(${PROGRAM_NAME})
