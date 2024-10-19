add_library(stepperDriver BasicDriver.cpp BasicDriver.h)

target_link_libraries(stepperDriver pico_stdlib)

target_include_directories(stepperDriver PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

add_library(stepperDriverObject stepperObject.cpp stepperObject.h)

target_link_libraries(stepperDriverObject pico_stdlib)

target_include_directories(stepperDriverObject PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

pico_enable_stdio_usb(stepperDriver 1)
pico_enable_stdio_uart(stepperDriver 0)