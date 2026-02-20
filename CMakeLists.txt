cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(can C CXX ASM)
 
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
 
pico_sdk_init()
 
# make a executable for blink.c
add_executable(can
    can.c
)
 
pico_enable_stdio_usb(can 1)
pico_enable_stdio_uart(can 1)
 
target_link_libraries(can
    pico_stdlib    
    hardware_gpio    
    hardware_timer    
    hardware_spi
)
 
# also generate uf2 and elf and bin and map files
pico_add_extra_outputs(can)
 
#add debug info
target_compile_definitions(can PRIVATE PICO_DEBUG=1)
target_compile_options(can PRIVATE -g3)

# be explicit about using rp2350
target_include_directories(can PRIVATE
    ${PICO_SDK_PATH}/src/rp2350/hardware_regs/include
    ${PICO_SDK_PATH}/src/rp2350/hardware_structs/include
)
 
# cmake debugging text
message(STATUS "Building custom can project for ${PICO_PLATFORM} (${PICO_BOARD})")
 