set(target_name magnometer)

add_library(pico_magnometer ${target_name}.h ${target_name}.c)

target_link_libraries(pico_magnometer 
        pico_stdlib 
        hardware_i2c
)

target_include_directories(pico_magnometer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
