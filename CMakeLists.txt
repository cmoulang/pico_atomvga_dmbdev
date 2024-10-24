cmake_minimum_required(VERSION 3.12)

set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)
project(atomvga_project)

pico_sdk_init()



add_compile_definitions(PICO_NO_FPGA_CHECK=1)
add_compile_definitions(PICO_BARE_METAL=1)
add_compile_definitions(PICO_CXX_ENABLE_EXCEPTIONS=0)
#add_compile_definitions(PICO_STDIO_UART=0)

set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-parameter" )

#
# Atom Build
#

add_executable(atomvga
  main.cc
  atomvga.c
  genlock.c
  atom_if.c
  atom_sid.cc
  exodecr.c
  reSID16/envelope.cc
  reSID16/extfilt.cc
  reSID16/pot.cc
  reSID16/filter.cc
  reSID16/sid.cc
  reSID16/voice.cc
  reSID16/wave.cc
  )

pico_generate_pio_header(atomvga ${CMAKE_CURRENT_LIST_DIR}/sm.pio)
pico_generate_pio_header(atomvga ${CMAKE_CURRENT_LIST_DIR}/genlock.pio)

target_compile_definitions(atomvga PUBLIC -DGENLOCK= -DINCLUDE_BRIGHT_ORANGE=1 -DPLATFORM=PLATFORM_ATOM -DPICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS=800 -DPICO_SCANVIDEO_SCANLINE_BUFFER_COUNT=16)
target_compile_definitions(atomvga PRIVATE PICO_MALLOC_PANIC=1)
target_compile_definitions(atomvga PRIVATE PICO_USE_MALLOC_MUTEX=0)
target_compile_definitions(atomvga PRIVATE PICO_DEBUG_MALLOC=0)
target_compile_options(atomvga PRIVATE -save-temps -fverbose-asm)

target_link_libraries(atomvga PRIVATE
  pico_multicore
  pico_stdlib
  pico_scanvideo_dpi
  hardware_pio
  hardware_pwm
  )

pico_add_extra_outputs(atomvga)
pico_enable_stdio_uart(atomvga 1)
# pico_set_linker_script(atomvga ${CMAKE_CURRENT_LIST_DIR}/memmap_copy_to_ram.ld)
pico_set_linker_script(atomvga ${CMAKE_CURRENT_LIST_DIR}/memmap_copy_to_ram_pico2.ld)

pico_set_binary_type(atomvga copy_to_ram)

#
# Atom R65C02 Build
#

add_executable(atomvga_r65c02
  atomvga.c
  atom_if.c
  )

pico_generate_pio_header(atomvga_r65c02 ${CMAKE_CURRENT_LIST_DIR}/sm.pio)

pico_generate_pio_header(atomvga_r65c02 ${CMAKE_CURRENT_LIST_DIR}/r65c02.pio)

pico_generate_pio_header(atomvga_r65c02 ${CMAKE_CURRENT_LIST_DIR}/atomvga_out.pio)

pico_generate_pio_header(atomvga_r65c02 ${CMAKE_CURRENT_LIST_DIR}/genlock.pio)

target_compile_definitions(atomvga_r65c02 PUBLIC -DPLATFORM=PLATFORM_ATOM -DPICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS=800 -DPICO_SCANVIDEO_SCANLINE_BUFFER_COUNT=16 -DR65C02=1)

target_link_libraries(atomvga_r65c02 PRIVATE
  main.cc
  pico_multicore
  pico_stdlib
  pico_scanvideo_dpi
  hardware_pio
  hardware_pwm
  )

pico_add_extra_outputs(atomvga_r65c02)

pico_enable_stdio_uart(atomvga_r65c02 1)

pico_set_binary_type(atomvga_r65c02 copy_to_ram)

target_compile_definitions(atomvga_r65c02 PRIVATE PICO_MALLOC_PANIC=0)


pico_set_linker_script(atomvga_r65c02 ${CMAKE_CURRENT_LIST_DIR}/memmap_copy_to_ram.ld)

#
# Dragon Build
#

add_executable(dragonvga
  main.cc
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
