cmake_minimum_required(VERSION 3.13)

# Built standalone -> also build the examples. Used via add_subdirectory() -> library only.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(PICO_BOARD pico2 CACHE STRING "Board type")
    include(pico_sdk_import.cmake)
    project(pico2_dynamixel C CXX ASM)
    pico_sdk_init()
    set(DXL_BUILD_EXAMPLES ON)
endif()

add_library(pico2_dynamixel INTERFACE)
target_sources(pico2_dynamixel INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/dynamixel.c
    ${CMAKE_CURRENT_LIST_DIR}/dxl_packet.c
    ${CMAKE_CURRENT_LIST_DIR}/dxl_regs.c)
target_include_directories(pico2_dynamixel INTERFACE ${CMAKE_CURRENT_LIST_DIR})
pico_generate_pio_header(pico2_dynamixel ${CMAKE_CURRENT_LIST_DIR}/dynamixel.pio)
target_link_libraries(pico2_dynamixel INTERFACE pico_stdlib hardware_pio hardware_dma hardware_clocks)

if(DXL_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
