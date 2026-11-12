/**
  ******************************************************************************
  * @file    adc_scan.c
  * @brief   8AIV acquisition engine (see adc_scan.h). TIM2 update interrupt,
  *          one polled ADS8688 frame per slot.
  ******************************************************************************
  */
#include "adc_scan.h"

#include <string.h>

#include "ads8688.h"
#include "board_io.h"

#define PRIME_SLOTS     2u      /* frame-list restart slots, data discarded    */

typedef enum {
    ST_PAUSED = 0,
    ST_RANGE,                   /* range writes / read-back, one per slot      */
    ST_PRIME,                   /* MAN_Ch(0), data discarded                   */
    ST_RUN,                     /* integration window                          */
} scan_state_t;

/* ---- thread -> ISR requests -------------------------------------------- */
static volatile uint8_t  s_range_want[SCAN_CHANNELS];   /* by ADS channel      */
static volatile uint8_t  s_range_mask;                  /* ADS channels to write */
static volatile uint16_t s_win_next;                    /* slots, 0 = no change */

/* ---- ISR private state ---------------------------------------------------- */
static volatile uint8_t  s_state;
static uint8_t  s_in_of_adc[SCAN_CHANNELS];
static uint8_t  s_range_have[SCAN_CHANNELS];            /* by ADS channel      */
static uint16_t s_win_slots;
static uint16_t s_k;                    /* slot inside the window              */
static uint8_t  s_prime_left;
static uint8_t  s_verify_ch;
static uint8_t  s_verify_due;
static uint8_t  s_retry_mask;
static uint16_t s_range_err;
static uint16_t s_overruns;
static volatile uint32_t s_slots;

static uint32_t s_sum[SCAN_CHANNELS];
static uint64_t s_sumsq[SCAN_CHANNELS];
static uint16_t s_n[SCAN_CHANNELS];
static uint16_t s_min[SCAN_CHANNELS];
static uint16_t s_max[SCAN_CHANNELS];
static uint16_t s_hi[SCAN_CHANNELS];
static uint16_t s_lo[SCAN_CHANNELS];

/* ---- published windows ----------------------------------------------------- */
static adc_win_t s_pub[2];              /* window with sequence s lives in s & 1 */
static volatile uint32_t s_pub_seq;

static volatile uint8_t s_running;

static void win_reset(void)
{
    for (uint8_t i = 0u; i < SCAN_CHANNELS; i++) {
        s_sum[i] = 0u; s_sumsq[i] = 0u; s_n[i] = 0u;
        s_min[i] = 0xFFFFu; s_max[i] = 0u; s_hi[i] = 0u; s_lo[i] = 0u;
    }
    s_k = 0u;
}

static void publish(void)
{
    const uint32_t seq = s_pub_seq + 1u;
    adc_win_t* p = &s_pub[seq & 1u];
    p->seq = seq;
    for (uint8_t i = 0u; i < SCAN_CHANNELS; i++) {
        p->sum[i] = s_sum[i];       p->sumsq[i] = s_sumsq[i];
        p->n[i] = s_n[i];           p->vmin[i] = s_min[i];
        p->vmax[i] = s_max[i];      p->clip_hi[i] = s_hi[i];
        p->clip_lo[i] = s_lo[i];
        p->range[i]   = s_range_have[BOARD_ADC_MAP[i]];
    }
    p->win_ms    = (s_win_slots == SCAN_WIN_20MS_SLOTS) ? 20u : 100u;
    p->range_err = s_range_err;
    p->overruns  = s_overruns;
    __DMB();
    s_pub_seq = seq;
}

/* Window boundary: apply a queued window length, decide what comes next. */
static void boundary(void)
{
    const uint16_t wn = s_win_next;
    if (wn != 0u) { s_win_slots = wn; s_win_next = 0u; }
    if (s_retry_mask != 0u) {
        __atomic_fetch_or(&s_range_mask, s_retry_mask, __ATOMIC_RELAXED);
        s_retry_mask = 0u;
    }
    win_reset();
    s_verify_due = 1u;
    s_prime_left = PRIME_SLOTS;
    s_state = ST_RANGE;
}

static void sample(uint8_t input, uint16_t code)
{
    s_sum[input]   += code;
    s_sumsq[input] += (uint64_t)code * code;
    s_n[input]++;
    if (code < s_min[input]) { s_min[input] = code; }
    if (code > s_max[input]) { s_max[input] = code; }
    if (code == 0xFFFFu)     { s_hi[input]++; }
    if (code == 0x0000u)     { s_lo[input]++; }
}

void TIM2_IRQHandler(void)
{
    TIM2->SR = ~TIM_SR_UIF;
    s_slots++;

    switch (s_state) {
    case ST_RUN: {
        const uint8_t a = (uint8_t)(s_k & 7u);   /* channel whose data arrives now */
        const uint16_t d = (uint16_t)board_adc_frame(ADS_CMD_MAN((s_k + 1u) & 7u));
        sample(s_in_of_adc[a], d);
        if (++s_k >= s_win_slots) {
            publish();
            boundary();
        }
        break;
    }

    case ST_RANGE: {
        const uint8_t m = s_range_mask;
        if (m != 0u) {
            const uint8_t ch = (uint8_t)__builtin_ctz(m);
            __atomic_fetch_and(&s_range_mask, (uint8_t)~(1u << ch), __ATOMIC_RELAXED);
            const uint8_t code = s_range_want[ch];
            (void)board_adc_frame(ADS_REG_WRITE(ADS_REG_RANGE(ch), code));
            const uint32_t f = board_adc_frame(ADS_REG_READ(ADS_REG_RANGE(ch)));
            if (ADS_REG_DATA(f) != code) { s_range_err++; s_retry_mask |= (uint8_t)(1u << ch); }
            else                         { s_range_have[ch] = code; }
        } else if (s_verify_due != 0u) {
            const uint8_t ch = s_verify_ch;
            const uint8_t code = s_range_want[ch];
            const uint32_t f = board_adc_frame(ADS_REG_READ(ADS_REG_RANGE(ch)));
            if (ADS_REG_DATA(f) != code) {
                s_range_err++;
                (void)board_adc_frame(ADS_REG_WRITE(ADS_REG_RANGE(ch), code));   /* self-heal */
                s_retry_mask |= (uint8_t)(1u << ch);   /* and confirm at the next boundary */
            }
            s_verify_ch = (uint8_t)((ch + 1u) & 7u);
            s_verify_due = 0u;
        } else {
            s_state = ST_PRIME;
        }
        break;
    }

    case ST_PRIME:
        (void)board_adc_frame(ADS_CMD_MAN(0u));
        if (--s_prime_left == 0u) { s_state = ST_RUN; }
        break;

    default:
        break;
    }

    if ((TIM2->SR & TIM_SR_UIF) != 0u) { s_overruns++; }
}

void adc_scan_start(const uint8_t range_code[SCAN_CHANNELS], uint16_t window_ms)
{
    for (uint8_t in = 0u; in < SCAN_CHANNELS; in++) {
        const uint8_t a = BOARD_ADC_MAP[in];
        s_in_of_adc[a]   = in;
        s_range_want[a]  = range_code[in];
        s_range_have[a]  = range_code[in];
    }
    s_range_mask = 0u;
    s_retry_mask = 0u;
    s_win_next   = 0u;
    s_win_slots  = (window_ms == 20u) ? SCAN_WIN_20MS_SLOTS : SCAN_WIN_100MS_SLOTS;
    s_verify_ch  = 0u;
    s_range_err  = 0u;
    s_overruns   = 0u;
    s_slots      = 0u;
    s_pub_seq    = 0u;
    memset(s_pub, 0, sizeof(s_pub));
    win_reset();
    s_verify_due = 0u;
    s_prime_left = PRIME_SLOTS;
    s_state      = ST_PRIME;

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;
    TIM2->CR1  = 0u;
    TIM2->PSC  = 0u;
    TIM2->ARR  = SCAN_SLOT_TICKS - 1u;
    TIM2->CNT  = 0u;
    TIM2->EGR  = TIM_EGR_UG;
    TIM2->SR   = 0u;
    TIM2->DIER = TIM_DIER_UIE;

    NVIC_SetPriority(TIM2_IRQn, 1u);
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);
    s_running = 1u;
    TIM2->CR1 |= TIM_CR1_CEN;
}

bool adc_scan_running(void) { return s_running != 0u; }

void adc_scan_set_range(uint8_t input, uint8_t range_code)
{
    if (input >= SCAN_CHANNELS) { return; }
    const uint8_t a = BOARD_ADC_MAP[input];
    s_range_want[a] = range_code;
    __atomic_fetch_or(&s_range_mask, (uint8_t)(1u << a), __ATOMIC_RELAXED);
}

bool adc_scan_set_window(uint16_t window_ms)
{
    if (window_ms != 20u && window_ms != 100u) { return false; }
    s_win_next = (window_ms == 20u) ? SCAN_WIN_20MS_SLOTS : SCAN_WIN_100MS_SLOTS;
    return true;
}

bool adc_scan_take(adc_win_t* out, uint32_t last_seq)
{
    for (uint8_t tries = 0u; tries < 3u; tries++) {
        const uint32_t s1 = s_pub_seq;
        if (s1 == 0u || s1 == last_seq) { return false; }
        memcpy(out, &s_pub[s1 & 1u], sizeof(*out));
        __DMB();
        /* The buffer of s1 is overwritten only by window s1 + 2. */
        if (out->seq == s1 && (uint32_t)(s_pub_seq - s1) < 2u) { return true; }
    }
    return false;
}

void adc_scan_pause(void)
{
    if (s_running == 0u) { return; }
    NVIC_DisableIRQ(TIM2_IRQn);
    TIM2->CR1 &= ~TIM_CR1_CEN;
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    s_state = ST_PAUSED;
}

void adc_scan_resume(void)
{
    if (s_running == 0u) { return; }
    win_reset();
    s_verify_due = 0u;
    s_prime_left = PRIME_SLOTS;
    s_state = ST_PRIME;
    TIM2->CNT = 0u;
    TIM2->SR  = 0u;
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;
}

uint32_t adc_scan_slots(void)          { return s_slots; }
uint16_t adc_scan_overruns(void)       { return s_overruns; }
uint16_t adc_scan_range_errors(void)   { return s_range_err; }
