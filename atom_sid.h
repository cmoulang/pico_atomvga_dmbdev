#pragma once
#include "pico/stdlib.h"
#include "atom_if.h"

#define FIFO_LEN 16
extern volatile int fifo_buffer[FIFO_LEN];
extern volatile int fifo_in;

// The Atom SID sound board uses #BDC0 to #BDDF
#define SID_BASE_ADDR 0xBDC0
#define SID_WRITEABLE 25
#define SID_LEN 29

#ifdef __cplusplus
extern "C"
{
#endif
    void as_init();
    void as_run();

    static inline void as_update_reg(uint8_t reg, uint8_t data)
    {
        eb_set(SID_BASE_ADDR + reg, data);
    }

    static inline void as_sid_write(uint16_t address, uint8_t data)
    {
        int y = ((address - SID_BASE_ADDR) << 8) | data;
        fifo_buffer[fifo_in] = y;
        fifo_in = (fifo_in + 1) % FIFO_LEN;
    }

#ifdef __cplusplus
}
#endif
