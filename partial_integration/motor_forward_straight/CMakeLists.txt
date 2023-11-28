include(FreeRTOS_Kernel_import.cmake)
add_executable(motor_forward_straight
motor_forward_straight.c
        ${PICO_LWIP_CONTRIB_PATH}/apps/ping/ping.c
        )
target_compile_definitions(motor_forward_straight PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        PING_USE_SOCKETS=1
        )
target_include_directories(motor_forward_straight PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        ${PICO_LWIP_CONTRIB_PATH}/apps/ping
        )
target_link_libraries(motor_forward_straight
        hardware_adc
        hardware_pwm
        pico_cyw43_arch_lwip_sys_freertos
        pico_stdlib
        pico_lwip_iperf
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        )
pico_enable_stdio_usb(motor_forward_straight 1)
pico_add_extra_outputs(motor_forward_straight)
