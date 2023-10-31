add_executable(cmExmaple cmExample.c)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(cmExmaple pico_stdlib hardware_uart pico_ultrasonic)

pico_enable_stdio_usb(cmExmaple 1)
pico_enable_stdio_uart(cmExmaple 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(cmExmaple)

# add url via pico_set_program_url
example_auto_set_url(cmExmaple)


