#pragma once
#include "pico/stdlib.h"
#include "atom_if.h"

#ifdef __cplusplus
extern "C" {
#endif

void as_init();
void as_main_loop(eb_int32_fifo_t *fifo);

#ifdef __cplusplus
}
#endif
