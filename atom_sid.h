#pragma once

#include "pico/stdlib.h"

#define SC_PIN 21
#define SC_SAMPLE_RATE 44100
#define SC_TICK_US (1000000 / SC_SAMPLE_RATE)
#define SC_NOOF_VOICES 3
#define SC_PWM_WRAP ((UINT16_MAX * 3) >> 6)

// The Atom SID sound board uses #BDC0 to #BDDF
#define SID_BASE_ADDR 0xBDC0
#define SID_LEN 29

#define C64_CLOCK 985248

#ifdef __cplusplus
extern "C" {
#endif

void initReSID();

#ifdef __cplusplus
}
#endif

