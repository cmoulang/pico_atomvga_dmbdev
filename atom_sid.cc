#include "atom_sid.h"
#include "atom_if.h"
#include "reSID16/sid.h"
#include "hardware/pwm.h"
#include <hardware/clocks.h>
#include <math.h>

#include <stdio.h>

#include "reSID_LUT.h"

#define C64_CLOCK 1000000ll
#define AS_SAMPLE_RATE 20000
#define AS_TICK_US 1000000ll / AS_SAMPLE_RATE
#define AS_PIN 21
#define AS_PWM_BITS 10
#define AS_PWM_WRAP (1 << AS_PWM_BITS)

volatile int fifo_buffer[FIFO_LEN];
volatile int fifo_in;
volatile int fifo_out;

static inline bool fifo_get(int *data)
{
    bool result;
    if (fifo_in == fifo_out)
    {
        result = false;
    }
    else
    {
        *data = fifo_buffer[fifo_out];
        fifo_out = (fifo_out + 1) % FIFO_LEN;
        result = true;
    }
    return result;
}

SID16 *sid16 = NULL;

void init_dac()
{
    gpio_set_dir(AS_PIN, GPIO_OUT);
    gpio_set_function(AS_PIN, GPIO_FUNC_PWM);

    int audio_pin_slice = pwm_gpio_to_slice_num(AS_PIN);
    pwm_config c = pwm_get_default_config();
    pwm_config_set_clkdiv(&c, 1);
    pwm_config_set_phase_correct(&c, false);
    pwm_config_set_wrap(&c, AS_PWM_WRAP);
    pwm_init(audio_pin_slice, &c, true);
    pwm_set_gpio_level(AS_PIN, 0);
    gpio_set_drive_strength(AS_PIN, GPIO_DRIVE_STRENGTH_12MA);
    pwm_set_enabled(audio_pin_slice, true);
}

static struct repeating_timer as_timer;
static struct repeating_timer debug_timer;

volatile uint64_t last_time;

void tick(SID16 *sid)
{
    uint64_t curr_time = time_us_64();
    uint elapsed = (uint)(curr_time - last_time);
    elapsed = elapsed & 0xFF;
    if (elapsed == 0)
    {
        elapsed = 1;
    }
    sid->clock(elapsed);
    last_time = curr_time;
}

extern "C" void as_init()
{
    int rate = AS_SAMPLE_RATE;
    int interval = AS_TICK_US;
    puts("INIT SID CALLED - fifo " __TIMESTAMP__);
    printf("Sample rate: %d/s\n", rate);
    printf("Sample interval: %dus\n", interval);

    fifo_in = 0;
    fifo_out = 0;

    sid16 = new SID16();
    sid16->set_chip_model(MOS8580);
    // sid16->set_chip_model(MOS6581);
    sid16->reset();
    bool ok = sid16->set_sampling_parameters(C64_CLOCK, SAMPLE_INTERPOLATE, AS_SAMPLE_RATE);

    hard_assert(ok);
    sid16->input(0);

    init_dac();

    eb_set_perm(SID_BASE_ADDR, EB_PERM_WRITE_ONLY, 25);
    eb_set_perm(SID_BASE_ADDR + 25, EB_PERM_READ_ONLY, 4);
}

extern "C" void as_main_loop()
{

    for (;;)
    {
        uint target_time = time_us_32() + AS_TICK_US;
        int sample = sid16->output(AS_PWM_BITS);
        sample = sample + (1 << (AS_PWM_BITS - 1));
        pwm_set_gpio_level(AS_PIN, sample);

        int ticks = AS_TICK_US;
        int x;
        while (fifo_get(&x))
        {
            int address = x >> 8;
            int data = x & 0xFF;
            sid16->write(address, data);
            sid16->clock();
            ticks--;
        }
        sid16->clock(ticks);
        while (target_time != time_us_32())
        {
            tight_loop_contents();
        }
    }
}