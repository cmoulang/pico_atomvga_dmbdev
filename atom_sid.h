#pragma once
#include "pico/stdlib.h"
#include "atom_if.h"

#ifdef __cplusplus
extern "C" {
#endif

// The Atom SID sound board uses #BDC0 to #BDDF
#define SID_BASE_ADDR 0xBDC0
#define SID_WRITEABLE 25
#define SID_LEN 29

void as_init();
void as_main_loop(eb_int32_fifo_t *fifo);
void as_sid_write(int address, int data);

#ifdef __cplusplus
}
#endif
