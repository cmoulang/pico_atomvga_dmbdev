#include "atom_sid.h"
#include "resid-0.16/sid.h"
#include "hardware/pwm.h"
#include <hardware/clocks.h>
#include <math.h>

#include <stdio.h>

#define C64_CLOCK 1000000
#define AS_TICK_US 24
#define AS_SAMPLE_RATE 1000000 / AS_TICK_US
#define AS_PIN 21
#define AS_PWM_BITS 11
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

SID *sid16 = NULL;

static void init_dac()
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

extern "C" void as_init()
{
    int rate = AS_SAMPLE_RATE;
    int interval = AS_TICK_US;
    puts("INIT SID CALLED - interrupt driven version " __DATE__ " " __TIME__);
    printf("Sample rate: %d/s\n", rate);
    printf("Sample interval: %dus\n", interval);

    fifo_in = 0;
    fifo_out = 0;

    sid16 = new SID();
    // sid16->set_chip_model(MOS8580);
    sid16->set_chip_model(MOS6581);
    sid16->reset();
    bool ok = sid16->set_sampling_parameters(C64_CLOCK, SAMPLE_INTERPOLATE, AS_SAMPLE_RATE);
    // bool ok = sid16->set_sampling_parameters(C64_CLOCK, SAMPLE_FAST, AS_SAMPLE_RATE);
    hard_assert(ok);

    sid16->reset();

    sid16->input(0);
    for (int i=0; i<SID_LEN; i++)
    {
        sid16->write(i,0);
    }

    init_dac();

    eb_set_perm(SID_BASE_ADDR, EB_PERM_WRITE_ONLY, 0x19);
    eb_set_perm(SID_BASE_ADDR + 0x1A, EB_PERM_READ_ONLY, 4);
}

#ifdef DEBUG_SID_DATA
int debug_count = 0;
uint16_t debug_buf[500];
#endif

static inline void do_sample()
{
    // Output current sample
    int sample = sid16->output(AS_PWM_BITS);
    sample = sample + (1 << (AS_PWM_BITS - 1));
    pwm_set_gpio_level(AS_PIN, sample);

    // process any writes to the SID registers
    int ticks = AS_TICK_US;
    int x;

    while (fifo_get(&x))
    {
#ifdef DEBUG_SID_DATA
        if (debug_count >= 0)
        {
            if (debug_count < sizeof debug_buf / 2)
            {
                debug_buf[debug_count++] = x;
            }
            else
            {
                for (int i = 0; i < sizeof debug_buf / 2; i++)
                {
                    printf("%d\n", debug_buf[i]);
                }
                debug_count = -1;
            }
        }
#endif
        int address = (x >> 8) & 0x1F;
        int data = x & 0xFF;
        sid16->write(address, data);
        sid16->clock(1);
        ticks--;
    }
    if (ticks > 0)
    {
        sid16->clock(ticks);
    }
    // Update the read-only SID regs
    as_update_reg(0x19, sid16->read(0x19));
    as_update_reg(0x1A, sid16->read(0x1A));
    as_update_reg(0x1B, sid16->read(0x1B));
    as_update_reg(0x1C, sid16->read(0x1C));
}

static bool as_timer_callback(repeating_timer_t *)
{
    do_sample();
    return true;
}

static struct repeating_timer as_timer;

extern "C" void as_run()
{
#ifdef SID_POLL_LOOP
    uint32_t last_time = 0;
    for (;;)
    {
        uint32_t cur_time = time_us_32();
        while (cur_time == last_time || cur_time % AS_TICK_US)
        {
            cur_time = time_us_32();
        }
        do_sample();
    }
#else
    bool ok = add_repeating_timer_us(-(int64_t)AS_TICK_US, as_timer_callback, NULL, &as_timer);
    hard_assert(ok);
#endif
}