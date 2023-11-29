# Replace 'hello' with file name (without extension)
# Define the target name as a variable
set(target_name barcode_scanner_server)

add_executable(${target_name} ${target_name}.c)

target_compile_definitions(${target_name} PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
target_include_directories(${target_name} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(${target_name} 
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
        hardware_adc
)

# allow the user to use serial monitor
pico_enable_stdio_usb(${target_name} 1)
pico_enable_stdio_uart(${target_name} 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(${target_name})

# add url via pico_set_program_url
example_auto_set_url(${target_name})
