
#include "atom_sid.h"
#include "reSID16/sid.h"

#include <stdio.h>

SID16* sid16;

extern "C" void initReSID()
{
    puts("InitReSID");
    sid16 = new SID16();
    sid16->set_chip_model(MOS8580);
    sid16->reset();
    sid16->set_sampling_parameters(C64_CLOCK, SAMPLE_INTERPOLATE, 44100);
    sid16->input(0);
}
