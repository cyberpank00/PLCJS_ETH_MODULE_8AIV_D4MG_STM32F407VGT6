/**
  ******************************************************************************
  * @file    adc_scan.h
  * @brief   8AIV acquisition engine: timer-paced ADS8688 round-robin scan with
  *          mains-synchronous integration windows.
  *
  *  Timing (SYSCLK 160 MHz, SPI 10 MHz, TIM2 = 80 MHz):
  *   - Slot = 3125 TIM2 ticks = 39.0625 µs (25.6 kSPS aggregate). One ADS8688
  *     frame (4 µs) per slot, in the TIM2 update interrupt (priority 1, above
  *     FreeRTOS; it makes no RTOS calls). 8 inputs are scanned in a fixed
  *     round robin, so each input is sampled every 8 slots = 312.5 µs.
  *   - Frame k sends MAN_Ch for the channel of frame k+1 and receives the
  *     conversion of the channel selected in frame k-1 (ADS8688 pipeline).
  *   - Window = 2560 slots = 100.000 ms (320 samples per input), an integer
  *     number of 50 Hz and 60 Hz periods, or 512 slots = 20.000 ms (64
  *     samples), an integer number of 50 Hz periods. A window ends exactly on
  *     a multiple of 8 slots, so every input gets the same sample count.
  *   - Between windows the engine spends one RANGE slot per pending range
  *     write plus one read-back of a rotating range register (self-check and
  *     self-heal of the ADS8688 configuration), then two PRIME slots (frame
  *     list restart, data discarded). A window therefore never contains a
  *     slot belonging to a range change or to a different pipeline start.
  *
  *  Everything lives in Flash: before a Flash erase the thread calls
  *  adc_scan_pause() (timer + IRQ off) and afterwards adc_scan_resume().
  *
  *  Thread <-> ISR: the ISR publishes each finished window into one of two
  *  buffers and bumps a sequence number; adc_scan_take() copies a consistent
  *  snapshot (retries internally). The thread requests range changes and a
  *  window length through adc_scan_set_range() / adc_scan_set_window(); the
  *  ISR applies them at the next window boundary.
  ******************************************************************************
  */
#ifndef APPLICATION_ADC_SCAN_H
#define APPLICATION_ADC_SCAN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCAN_CHANNELS          8u
#define SCAN_SLOT_TICKS        3125u    /* TIM2 80 MHz -> 39.0625 µs           */
#define SCAN_WIN_20MS_SLOTS    512u     /* 8 x 64                              */
#define SCAN_WIN_100MS_SLOTS   2560u    /* 8 x 320                             */
#define SCAN_RANGE_DEFAULT     0x05u    /* 0..2.5·VREF, the 0-10 V mode        */

/** One finished window, indexed by INPUT number AI0..AI7. */
typedef struct {
    uint32_t seq;                       /* 1, 2, ...; 0 = nothing published    */
    uint32_t sum[SCAN_CHANNELS];        /* sum of raw codes                    */
    uint64_t sumsq[SCAN_CHANNELS];      /* sum of squared codes (noise)        */
    uint16_t n[SCAN_CHANNELS];          /* samples                             */
    uint16_t vmin[SCAN_CHANNELS];
    uint16_t vmax[SCAN_CHANNELS];
    uint16_t clip_hi[SCAN_CHANNELS];    /* samples at 0xFFFF                   */
    uint16_t clip_lo[SCAN_CHANNELS];    /* samples at 0x0000                   */
    uint8_t  range[SCAN_CHANNELS];      /* ADS8688 range code in force, per input */
    uint16_t win_ms;                    /* 20 or 100                           */
    uint16_t range_err;                 /* read-back mismatches so far (count) */
    uint16_t overruns;                  /* slots whose ISR outlasted the slot  */
} adc_win_t;

/** Start the scan. @p range_code[input] = ADS8688 range register value of each
 *  INPUT; the ADS8688 must already hold exactly these (ads8688_init +
 *  ads8688_set_range). @p window_ms is 20 or 100. */
void adc_scan_start(const uint8_t range_code[SCAN_CHANNELS], uint16_t window_ms);
bool adc_scan_running(void);

/** Queue a range change of one INPUT (applied at the next window boundary). */
void adc_scan_set_range(uint8_t input, uint8_t range_code);
/** Queue a window length change (20 or 100 ms) for the next boundary. */
bool adc_scan_set_window(uint16_t window_ms);

/** Copy the newest window if its sequence differs from @p last_seq.
 *  Returns false when there is nothing new. */
bool adc_scan_take(adc_win_t* out, uint32_t last_seq);

/** Stop / restart the engine around a Flash erase. Resume restarts the window
 *  (the partial one is dropped). Thread context only. */
void adc_scan_pause(void);
void adc_scan_resume(void);

/** Diagnostics. */
uint32_t adc_scan_slots(void);          /* slots since start                   */
uint16_t adc_scan_overruns(void);       /* ISR took longer than one slot       */
uint16_t adc_scan_range_errors(void);   /* range read-back mismatches          */

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_ADC_SCAN_H */
