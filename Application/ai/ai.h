/**
  ******************************************************************************
  * @file    ai.h
  * @brief   8AIV analog-input channel layer.
  *
  *  The scan engine (adc_scan.c) samples in a timer interrupt and publishes
  *  one integration window at a time. Everything here runs in the AI task
  *  (ai_poll) or is a short, lock-protected accessor for the Modbus task.
  *
  *  Channel life cycle: OFF | SETTLE (the window was not yet taken with the
  *  channel's range in force) -> RUN. The ADC itself is supervised by the
  *  scan engine's range-register read-back; a mismatch raises ADC_FAIL for
  *  AI_FAIL_HOLD_WINDOWS windows.
  ******************************************************************************
  */
#ifndef APPLICATION_AI_H
#define APPLICATION_AI_H

#include <stdbool.h>
#include <stdint.h>

#include "ai_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Channel state (Modbus IR356..363, diagnostic). */
#define AI_STATE_OFF        0u
#define AI_STATE_SETTLE     1u
#define AI_STATE_RUN        2u
#define AI_STATE_ADC_FAIL   3u

/* Commands (ai_command). */
#define AI_CMD_RESTART      2u   /* drop the filter, wait for a fresh window   */
#define AI_CMD_APPLY        3u   /* re-read every channel + common block       */

/* Module flags (Modbus IR127). */
#define AI_MOD_ADC_FAIL     0x0001u   /* ADS8688 not running / lost its config */
#define AI_MOD_CONFIG_ERR   0x0002u   /* invalid stored configuration           */
#define AI_MOD_RUNNING      0x0004u   /* scan engine running                    */
#define AI_MOD_RANGE_ERR    0x0008u   /* range read-back mismatch (recent)      */
#define AI_MOD_OVERRUN      0x0010u   /* a slot interrupt outlasted its slot    */

#define AI_FAIL_HOLD_WINDOWS  50u     /* 5 s at 100 ms windows                  */

/* int16 view (holding 0..7): mV; -32768 = no valid value. */
#define AI_I16_INVALID      ((int16_t)-32768)

typedef struct {
    float    value;        /* engineering value, NaN when not valid           */
    float    raw;          /* calibrated input voltage, V                      */
    uint16_t status;       /* AI_ST_*                                          */
    uint16_t seq;          /* update counter                                   */
    uint8_t  state;        /* AI_STATE_*                                       */
} ai_data_t;

/** Raw statistics of the last window, for ADC bring-up and noise checks. */
typedef struct {
    uint16_t mean;         /* mean code                                        */
    uint16_t vmin;
    uint16_t vmax;
    uint16_t clip_hi;      /* samples at 0xFFFF                                */
    uint16_t clip_lo;      /* samples at 0x0000                                */
    uint16_t n;            /* samples in the window                            */
    float    rms;          /* code noise, LSB RMS                              */
} ai_stats_t;

/** Wait for the isolated supply, reset and configure the ADS8688, start the
 *  scan. Returns false when the ADC does not answer; ai_poll() keeps retrying
 *  once a second. */
bool ai_init(void);

/** AI task body, call every ~2 ms: new windows -> values, ADC retry. */
void ai_poll(void);

/** Validate and apply a channel configuration (a range change goes through
 *  the scan engine and the channel settles for one window). */
bool ai_set_config(uint8_t ch, const ai_ch_cfg_t* cfg);
void ai_get_config(uint8_t ch, ai_ch_cfg_t* out);
/** Validate and apply the common block. */
bool ai_set_common(const ai_model_t* m, const ai_diag_t* d, uint16_t window_ms);
bool ai_common_valid(const ai_model_t* m, const ai_diag_t* d, uint16_t window_ms);
void ai_get_common(ai_model_t* m, ai_diag_t* d, uint16_t* window_ms);

void    ai_get(uint8_t ch, ai_data_t* out);
void    ai_get_stats(uint8_t ch, ai_stats_t* out);
int16_t ai_get_int16(uint8_t ch);

bool     ai_command(uint8_t ch, uint8_t cmd);
uint16_t ai_module_flags(void);
uint16_t ai_samples_per_window(void);
uint16_t ai_window_counter(void);
uint16_t ai_init_attempts(void);

/** Channel LEDs: on = RUN with data, slow blink = no data yet, fast blink =
 *  fault, off = OFF. Call from a 10 ms tick. */
void ai_led_tick(uint16_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_AI_H */
