# Replace 'hello' with file name (without extension)
# Define the target name as a variable
set(target_name motor_turn_magno)

add_executable(${target_name} ${target_name}.c)

# pull in common dependencies and additional pwm hardware support
target_link_libraries(${target_name} 
        pico_stdlib 
        pico_cyw43_arch_none
        hardware_i2c
        pico_magnometer
        hardware_pwm
)

# allow the user to use serial monitor
pico_enable_stdio_usb(${target_name} 1)
pico_enable_stdio_uart(${target_name} 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(${target_name})

# add url via pico_set_program_url
example_auto_set_url(${target_name})
