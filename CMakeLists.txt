cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(BME280_read C CXX ASM) 
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${PROJECT_NAME}
        BME280_read.cpp
        )

target_sources (${PROJECT_NAME}
        PUBLIC 
        ${CMAKE_CURRENT_LIST_DIR}/BME280/bme280_i2c.cpp
        ${CMAKE_CURRENT_LIST_DIR}/RPi-Pico-SSD1306-library/SSD1306.cpp
        ${CMAKE_CURRENT_LIST_DIR}/RPi-Pico-SSD1306-library/GFX.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/BME280
        ${CMAKE_CURRENT_LIST_DIR}/SSD1306    
)

# Pull in our (to be renamed) simple get you started dependencies
target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_i2c)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})

# add url via pico_set_program_url
#example_auto_set_url(i2c_bus_scan)
