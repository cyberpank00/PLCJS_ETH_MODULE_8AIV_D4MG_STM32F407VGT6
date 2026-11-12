/**
  ******************************************************************************
  * @file    ai_math.h
  * @brief   8AIV conversion maths. Pure C, no HAL: built into the firmware and
  *          into the host tests (tests/).
  *
  *  Signal path:  mean ADC code -> V_p (pin voltage, aim_code_to_vp)
  *                -> V_n (node after the divider, aim_vn)
  *                -> V_in (input terminal, aim_vin)
  *                -> calibrated volts -> engineering value + status.
  ******************************************************************************
  */
#ifndef APPLICATION_AI_MATH_H
#define APPLICATION_AI_MATH_H

#include <stdbool.h>
#include <stdint.h>

#include "ai_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- ranges ------------------------------------------------------------ */
uint8_t aim_range_code(ai_range_t r);              /* ADS8688 range register value */
float   aim_range_fs(ai_range_t r, float v_ref);   /* full-scale bound, V          */
bool    aim_range_bipolar(ai_range_t r);
/** Code (may be a fractional average) -> pin voltage. Straight binary:
 *  bipolar V = (code - 32768)·FS/32768, unipolar V = code·FS/65536. */
float    aim_code_to_vp(float code, ai_range_t r, float v_ref);
/** Pin voltage -> nearest code, clamped to 0..65535. */
uint16_t aim_vp_to_code(float vp, ai_range_t r, float v_ref);

/* ---- modes ------------------------------------------------------------- */
ai_range_t aim_mode_range(uint8_t mode);
/** Signal range of the mode (V): *lo .. *hi. OFF -> 0..0. */
void aim_mode_signal(uint8_t mode, float* lo, float* hi);

/* ---- input network ------------------------------------------------------ */
/** Node voltage (before R_adc) from the pin voltage. R_L = R_adc + R_in. */
float aim_vn(float vp, float vb, const ai_model_t* m);
/** Inverse of aim_vn. */
float aim_vp_from_vn(float vn, float vb, const ai_model_t* m);
/** Input-terminal voltage from the node voltage (divider + load current). */
float aim_vin(float vn, float vb, const ai_model_t* m);
/** Inverse of aim_vin. */
float aim_vn_from_vin(float vin, float vb, const ai_model_t* m);
/** Small-signal gain of the whole network, V_p / V_in at V_in = vb (≈ divider). */
float aim_gain_nominal(float vb, const ai_model_t* m);

/* ---- evaluation ---------------------------------------------------------- */
typedef struct {
    float    value;    /* engineering value; NaN if invalid                         */
    float    raw;      /* calibrated input voltage, V; NaN if invalid               */
    uint16_t status;   /* AI_ST_* result bits                                       */
} aim_result_t;

/** One finished window of one input: @p code_mean = sum / n, @p n samples of
 *  which @p clip_hi at 0xFFFF and @p clip_lo at 0x0000. */
void aim_eval(const ai_ch_cfg_t* c, const ai_model_t* m, const ai_diag_t* d,
              float code_mean, uint16_t n, uint16_t clip_hi, uint16_t clip_lo,
              aim_result_t* out);

/** Linear engineering scaling of the mode's signal range (lo == hi -> identity). */
float aim_scale(const ai_ch_cfg_t* c, float volts);

/** Defaults. */
void aim_model_defaults(ai_model_t* m);
void aim_diag_defaults(ai_diag_t* d);
void aim_ch_defaults(ai_ch_cfg_t* c);
/** Range/consistency checks. */
bool aim_ch_valid(const ai_ch_cfg_t* c);
bool aim_model_valid(const ai_model_t* m);
bool aim_diag_valid(const ai_diag_t* d);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_AI_MATH_H */
