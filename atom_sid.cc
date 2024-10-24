#include "atom_sid.h"
#include "atom_if.h"
#include "reSID16/sid.h"
#include "hardware/pwm.h"
#include <hardware/clocks.h>
#include <math.h>

#include <stdio.h>

#include "reSID_LUT.h"

#define C64_CLOCK 1000000ll

#define AS_SAMPLE_RATE 44100
#define AS_TICK_US 1000000ll / AS_SAMPLE_RATE
#define AS_PIN 21
#define AS_PWM_WRAP (1 << 10)


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

void pstate(SID16::State &s)
{
    puts("sid_register");
    for (int i = 0; i < 21; i++)
    {
        printf("%2x ", s.sid_register[i]);
    }
    printf("\n bus_value: %x\n", s.bus_value);
    printf("bus_value_ttl : %x\n", s.bus_value_ttl);

    for (int i = 0; i < 3; i++)
    {
        printf("accumulator[%d]=%d\n", i, s.accumulator[i]);
        printf("shift_register[%d]=%d\n", i, s.shift_register[i]);
        printf("rate_counter[%d]=%d\n", i, s.rate_counter[i]);
        printf("rate_counter_period[%d]=%d\n", i, s.rate_counter_period[i]);
        printf("exponential_counter[%d]=%d\n", i, s.exponential_counter[i]);
        printf("exponential_counter_period[%d]=%d\n", i, s.exponential_counter_period[i]);
        printf("envelope_counter[%d]=%d\n", i, s.envelope_counter[i]);
        printf("envelope_state[%d]=%d\n", i, s.envelope_state[i]);
        printf("hold_zero[%d]=%d\n", i, s.hold_zero[i]);
    }
}

int max_sample = 0;
int min_sample = INT16_MAX;

uint64_t as_timer_callback_count = 0;
extern "C" bool as_timer_callback(struct repeating_timer *t)
{
    as_timer_callback_count++;
    //    SID16 *sid = (SID16 *)t->user_data;

    tick(sid16);

    int sample = sid16->output(10);
    sample = sample + (1 << 9);

    if (sample > max_sample) max_sample = sample;
    if (sample < min_sample) min_sample = sample;
    // if (sample == 0) sample = rand() & 0xFF;
    pwm_set_gpio_level(AS_PIN, sample);

    return true;
}

extern "C" bool debug_timer_callback(struct repeating_timer *t)
{
    printf("{%d %d} ", min_sample, max_sample);
    return true;
}

extern "C" void as_init()
{
    puts("INIT SID CALLED - fifo");

    sid16 = new SID16();
    // sid16->set_chip_model(MOS8580);
    sid16->set_chip_model(MOS6581);
    sid16->reset();
    bool ok = sid16->set_sampling_parameters(C64_CLOCK, SAMPLE_INTERPOLATE, AS_SAMPLE_RATE);
    sid16->input(0);

    hard_assert(ok);

    init_dac();

    eb_set_perm(SID_BASE_ADDR, EB_PERM_WRITE_ONLY, 25);
    eb_set_perm(SID_BASE_ADDR + 25, EB_PERM_READ_ONLY, 4);

    ok = add_repeating_timer_us(-AS_TICK_US, as_timer_callback, sid16, &as_timer);
    hard_assert(ok);
    // ok = add_repeating_timer_us(-1000000ll, debug_timer_callback, sid16, &debug_timer);
    // hard_assert(ok);
}

extern "C" void as_sid_write(int address, int data)
{
}

extern "C" void as_main_loop(eb_int32_fifo_t *fifo)
{
    last_time = time_us_64();
    for (;;)
    {
        int x;
        if (eb_int32_fifo_get(fifo, &x))
        {
            int address = x >> 8;
            int data = x & 0xFF;
            sid16->write(address, data);
            sid16->clock();
            last_time++;
        }
    }
}