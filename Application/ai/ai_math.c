/**
  ******************************************************************************
  * @file    ai_math.c
  * @brief   8AIV conversion maths — see ai_math.h. Pure C, host tested.
  ******************************************************************************
  */
#include "ai_math.h"

#include <string.h>

#define AIM_NAN       (__builtin_nanf(""))
#define AIM_FINITE(x) (__builtin_isfinite(x))

static bool in_range(float x, float lo, float hi)
{
    return AIM_FINITE(x) && x >= lo && x <= hi;
}

/* ---- ranges ------------------------------------------------------------ */
typedef struct {
    uint8_t code;
    uint8_t bipolar;
    float   fs_mul;      /* FS = fs_mul · V_REF */
} range_desc_t;

static const range_desc_t k_ranges[AI_RANGE_COUNT] = {
    [AI_RANGE_PM10] = { 0x00u, 1u, 2.5f   },
    [AI_RANGE_PM5]  = { 0x01u, 1u, 1.25f  },
    [AI_RANGE_P10]  = { 0x05u, 0u, 2.5f   },
    [AI_RANGE_P5]   = { 0x06u, 0u, 1.25f  },
};

uint8_t aim_range_code(ai_range_t r)            { return k_ranges[r].code; }
float   aim_range_fs(ai_range_t r, float v_ref) { return k_ranges[r].fs_mul * v_ref; }
bool    aim_range_bipolar(ai_range_t r)         { return k_ranges[r].bipolar != 0u; }

float aim_code_to_vp(float code, ai_range_t r, float v_ref)
{
    const float fs = aim_range_fs(r, v_ref);
    return aim_range_bipolar(r) ? (code - 32768.0f) * fs / 32768.0f
                                : code * fs / 65536.0f;
}

uint16_t aim_vp_to_code(float vp, ai_range_t r, float v_ref)
{
    const float fs = aim_range_fs(r, v_ref);
    float c = aim_range_bipolar(r) ? vp * 32768.0f / fs + 32768.0f
                                   : vp * 65536.0f / fs;
    c += 0.5f;
    if (!(c > 0.0f))     { return 0u; }
    if (c >= 65535.0f)   { return 65535u; }
    return (uint16_t)c;
}

/* ---- modes ------------------------------------------------------------- */
ai_range_t aim_mode_range(uint8_t mode)
{
    switch (mode) {
    case AI_MODE_V_0_5:   return AI_RANGE_P5;
    case AI_MODE_V_PM10:  return AI_RANGE_PM10;
    case AI_MODE_V_PM5:   return AI_RANGE_PM5;
    default:              return AI_RANGE_P10;     /* 0-10 V, 2-10 V, OFF */
    }
}

void aim_mode_signal(uint8_t mode, float* lo, float* hi)
{
    switch (mode) {
    case AI_MODE_V_0_10: *lo =   0.0f; *hi = 10.0f; break;
    case AI_MODE_V_2_10: *lo =   2.0f; *hi = 10.0f; break;
    case AI_MODE_V_0_5:  *lo =   0.0f; *hi =  5.0f; break;
    case AI_MODE_V_PM10: *lo = -10.0f; *hi = 10.0f; break;
    case AI_MODE_V_PM5:  *lo =  -5.0f; *hi =  5.0f; break;
    default:             *lo =   0.0f; *hi =  0.0f; break;
    }
}

/* ---- input network ------------------------------------------------------ */
static float r_load(const ai_model_t* m) { return m->r_adc + m->r_in; }

float aim_vn(float vp, float vb, const ai_model_t* m)
{
    return vb + (vp - vb) * r_load(m) / m->r_in;
}

float aim_vp_from_vn(float vn, float vb, const ai_model_t* m)
{
    return vb + (vn - vb) * m->r_in / r_load(m);
}

/* KCL at the node: (Vin - Vn)/R_top = Vn/R_shunt + (Vn - Vb)/R_L. */
float aim_vin(float vn, float vb, const ai_model_t* m)
{
    const float rl = r_load(m);
    return vn * (1.0f + m->r_top / m->r_shunt + m->r_top / rl) - m->r_top * vb / rl;
}

float aim_vn_from_vin(float vin, float vb, const ai_model_t* m)
{
    const float rl = r_load(m);
    return (vin + m->r_top * vb / rl) / (1.0f + m->r_top / m->r_shunt + m->r_top / rl);
}

float aim_gain_nominal(float vb, const ai_model_t* m)
{
    return aim_vp_from_vn(aim_vn_from_vin(vb + 1.0f, vb, m), vb, m)
         - aim_vp_from_vn(aim_vn_from_vin(vb, vb, m), vb, m);
}

/* ---- evaluation ---------------------------------------------------------- */
float aim_scale(const ai_ch_cfg_t* c, float volts)
{
    if (c->scale_lo == c->scale_hi) { return volts; }
    float s0, s1;
    aim_mode_signal(c->mode, &s0, &s1);
    if (s0 == s1) { return volts; }
    return c->scale_lo + (volts - s0) * (c->scale_hi - c->scale_lo) / (s1 - s0);
}

void aim_eval(const ai_ch_cfg_t* c, const ai_model_t* m, const ai_diag_t* d,
              float code_mean, uint16_t n, uint16_t clip_hi, uint16_t clip_lo,
              aim_result_t* out)
{
    out->value  = AIM_NAN;
    out->raw    = AIM_NAN;
    out->status = 0u;

    if (c->mode == AI_MODE_OFF) { out->status = AI_ST_OFF; return; }
    if (n == 0u)                { out->status = AI_ST_NO_DATA; return; }

    const ai_range_t r = aim_mode_range(c->mode);
    const float vb  = m->v_b[r];
    const float vp  = aim_code_to_vp(code_mean, r, m->v_ref);
    const float vin = aim_vin(aim_vn(vp, vb, m), vb, m);
    const float v   = c->cal_gain * vin + c->cal_offset_v;

    uint16_t st = 0u;
    if ((uint32_t)clip_hi * 2u >= n) { st |= AI_ST_ADC_CLIP | AI_ST_OVERRANGE; }
    if (aim_range_bipolar(r) && (uint32_t)clip_lo * 2u >= n) { st |= AI_ST_ADC_CLIP | AI_ST_UNDERRANGE; }

    if (c->mode == AI_MODE_V_0_10 || c->mode == AI_MODE_V_2_10) {
        if (v > d->v_over_v) { st |= AI_ST_OVERRANGE; }
    }
    if (c->mode == AI_MODE_V_2_10 && v < d->v210_open_v) { st |= AI_ST_WIRE_BREAK; }

    out->status = st;
    /* Range violations are shown (saturated) but flagged; a wire break is NaN. */
    if ((st & AI_ST_WIRE_BREAK) == 0u) {
        out->raw   = v;
        out->value = aim_scale(c, v);
    }
}

/* ---- defaults / validation ---------------------------------------------- */
void aim_model_defaults(ai_model_t* m)
{
    m->v_ref   = 4.096f;
    m->r_top   = 1000.0f;
    m->r_shunt = 120000.0f;
    m->r_adc   = 1000.0f;
    m->r_in    = 1.0e6f;
    /* ADS8688 SBAS582C, input current (V_IN - V_B) / R_IN per range. */
    m->v_b[AI_RANGE_PM10] = 2.25f;
    m->v_b[AI_RANGE_PM5]  = 2.00f;
    m->v_b[AI_RANGE_P10]  = 2.50f;
    m->v_b[AI_RANGE_P5]   = 2.50f;
}

void aim_diag_defaults(ai_diag_t* d)
{
    d->v210_open_v = 1.8f;
    d->v_over_v    = 10.2f;
}

void aim_ch_defaults(ai_ch_cfg_t* c)
{
    memset(c, 0, sizeof(*c));
    c->mode     = AI_MODE_V_0_10;
    c->cal_gain = 1.0f;
}

bool aim_ch_valid(const ai_ch_cfg_t* c)
{
    if (c->mode >= AI_MODE_COUNT)                       { return false; }
    if (!in_range(c->cal_gain, 0.5f, 2.0f))             { return false; }
    if (!in_range(c->cal_offset_v, -1.0f, 1.0f))        { return false; }
    if (!AIM_FINITE(c->scale_lo) || !AIM_FINITE(c->scale_hi)) { return false; }
    return true;
}

bool aim_model_valid(const ai_model_t* m)
{
    if (!in_range(m->v_ref, 3.0f, 5.0f))        { return false; }
    if (!in_range(m->r_top, 1.0f, 1.0e5f))      { return false; }
    if (!in_range(m->r_shunt, 1.0e3f, 1.0e7f))  { return false; }
    if (!in_range(m->r_adc, 0.0f, 1.0e5f))      { return false; }
    if (!in_range(m->r_in, 1.0e5f, 1.0e8f))     { return false; }
    for (uint8_t i = 0u; i < AI_RANGE_COUNT; i++) {
        if (!in_range(m->v_b[i], 0.0f, 5.0f))   { return false; }
    }
    return true;
}

bool aim_diag_valid(const ai_diag_t* d)
{
    return in_range(d->v210_open_v, 0.0f, 10.0f) && in_range(d->v_over_v, 5.0f, 12.0f);
}
