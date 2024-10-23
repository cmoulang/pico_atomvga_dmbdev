cmake_minimum_required(VERSION 3.12)

set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)
project(atomvga_project)

pico_sdk_init()

set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-parameter" )

#
# Atom Build
#

add_executable(atomvga
  atomvga.c
  genlock.c
  atom_if.c
  )

pico_generate_pio_header(atomvga ${CMAKE_CURRENT_LIST_DIR}/sm.pio)
pico_generate_pio_header(atomvga ${CMAKE_CURRENT_LIST_DIR}/genlock.pio)

target_compile_definitions(atomvga PUBLIC -DGENLOCK= -DINCLUDE_BRIGHT_ORANGE=1 -DPLATFORM=PLATFORM_ATOM -DPICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS=800 -DPICO_SCANVIDEO_SCANLINE_BUFFER_COUNT=16)

target_link_libraries(atomvga PRIVATE
  pico_multicore
  pico_stdlib
  pico_scanvideo_dpi
  hardware_pio
  hardware_pwm
  )

pico_add_extra_outputs(atomvga)
pico_enable_stdio_uart(atomvga 1)
#pico_set_linker_script(atomvga ${CMAKE_CURRENT_LIST_DIR}/memmap_copy_to_ram.ld)
pico_set_linker_script(atomvga ${CMAKE_CURRENT_LIST_DIR}/memmap_copy_to_ram_pico2.ld)

pico_set_binary_type(atomvga copy_to_ram)

#
# Dragon Build
#

add_executable(dragonvga
  atomvga.c
  eeprom.c
  atom_if.c
  )

pico_generate_pio_header(dragonvga ${CMAKE_CURRENT_LIST_DIR}/sm.pio)
  
pico_generate_pio_header(dragonvga ${CMAKE_CURRENT_LIST_DIR}/atomvga.pio)

pico_generate_pio_header(dragonvga ${CMAKE_CURRENT_LIST_DIR}/atomvga_out.pio)

target_compile_definitions(dragonvga PUBLIC -DPLATFORM=PLATFORM_DRAGON -DPICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS=800 -DPICO_SCANVIDEO_SCANLINE_BUFFER_COUNT=16)

target_link_libraries(dragonvga PRIVATE
  pico_multicore
  pico_stdlib
  pico_scanvideo_dpi
  hardware_pio
  hardware_i2c
  hardware_pwm
  )

pico_add_extra_outputs(dragonvga)

pico_enable_stdio_uart(dragonvga 1)

pico_set_binary_type(dragonvga copy_to_ram)
