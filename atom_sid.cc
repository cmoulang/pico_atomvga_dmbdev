#include "atom_sid.h"
#include "reSID16/sid.h"
#include <stdio.h>

#define C64_CLOCK 1000000

SID16* sid16;

extern "C" void as_init()
{
    puts("INIT SID CALLED - 1");
    sid16 = new SID16();
    sid16->set_chip_model( MOS8580 );
    sid16->reset();
    sid16->set_sampling_parameters( C64_CLOCK, SAMPLE_INTERPOLATE, 44100 );

}