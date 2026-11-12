/* Host tests of the 8AIV conversion maths (ai_math.c). */
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "ai_math.h"

static int near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

static ai_model_t M;
static ai_diag_t  D;

static ai_ch_cfg_t cfg(uint8_t mode)
{
    ai_ch_cfg_t c;
    aim_ch_defaults(&c);
    c.mode = mode;
    return c;
}

/* Code of a terminal voltage in the range of @p mode, as the ADC would report it. */
static float code_for_vin(uint8_t mode, float vin)
{
    const ai_range_t r = aim_mode_range(mode);
    const float vb = M.v_b[r];
    const float vp = aim_vp_from_vn(aim_vn_from_vin(vin, vb, &M), vb, &M);
    return (float)aim_vp_to_code(vp, r, M.v_ref);
}

static void test_ranges(void)
{
    assert(aim_range_code(AI_RANGE_PM10) == 0x00u && aim_range_code(AI_RANGE_PM5) == 0x01u);
    assert(aim_range_code(AI_RANGE_P10) == 0x05u && aim_range_code(AI_RANGE_P5) == 0x06u);
    assert(near(aim_range_fs(AI_RANGE_P10, 4.096f), 10.24f, 1e-4f));
    assert(near(aim_range_fs(AI_RANGE_P5, 4.096f), 5.12f, 1e-4f));
    assert(near(aim_code_to_vp(32768.0f, AI_RANGE_P10, 4.096f), 5.12f, 1e-4f));
    assert(near(aim_code_to_vp(32768.0f, AI_RANGE_PM10, 4.096f), 0.0f, 1e-6f));
    assert(near(aim_code_to_vp(0.0f, AI_RANGE_PM10, 4.096f), -10.24f, 1e-4f));
    assert(near(aim_code_to_vp(65535.0f, AI_RANGE_PM5, 4.096f), 5.12f * 32767.0f / 32768.0f, 1e-3f));
    for (float v = 0.0f; v < 10.2f; v += 0.37f) {
        const uint16_t c = aim_vp_to_code(v, AI_RANGE_P10, 4.096f);
        assert(near(aim_code_to_vp((float)c, AI_RANGE_P10, 4.096f), v, 10.24f / 65536.0f));
    }
    assert(aim_vp_to_code(-1.0f, AI_RANGE_P10, 4.096f) == 0u);
    assert(aim_vp_to_code(20.0f, AI_RANGE_P10, 4.096f) == 65535u);
    assert(aim_vp_to_code(0.0f, AI_RANGE_PM10, 4.096f) == 32768u);
}

static void test_modes(void)
{
    float lo, hi;
    assert(aim_mode_range(AI_MODE_V_0_10) == AI_RANGE_P10);
    assert(aim_mode_range(AI_MODE_V_2_10) == AI_RANGE_P10);
    assert(aim_mode_range(AI_MODE_V_0_5) == AI_RANGE_P5);
    assert(aim_mode_range(AI_MODE_V_PM10) == AI_RANGE_PM10);
    assert(aim_mode_range(AI_MODE_V_PM5) == AI_RANGE_PM5);
    aim_mode_signal(AI_MODE_V_2_10, &lo, &hi); assert(lo == 2.0f && hi == 10.0f);
    aim_mode_signal(AI_MODE_V_PM5, &lo, &hi);  assert(lo == -5.0f && hi == 5.0f);
    aim_mode_signal(AI_MODE_OFF, &lo, &hi);    assert(lo == hi);
}

/* The input network and its inverse must round-trip. The small-signal gain is
 * the divider 1 / (1 + R_top/R_shunt + R_top/R_L) times the pin attenuation
 * R_in / R_L: 0.99075 * 0.999 = 0.98976 (the 120 k shunt alone would give
 * 0.99174; the 1 MOhm ADC load and the 1 k series resistor cost another 0.2 %). */
static void test_network(void)
{
    for (int r = 0; r < AI_RANGE_COUNT; r++) {
        const float vb = M.v_b[r];
        for (float vin = -10.0f; vin <= 10.0f; vin += 1.7f) {
            const float vn = aim_vn_from_vin(vin, vb, &M);
            assert(near(aim_vin(vn, vb, &M), vin, 1e-4f));
            const float vp = aim_vp_from_vn(vn, vb, &M);
            assert(near(aim_vn(vp, vb, &M), vn, 1e-4f));
        }
        const float g = aim_gain_nominal(vb, &M);
        assert(g > 0.9895f && g < 0.9900f);
    }
    const float vb = M.v_b[AI_RANGE_P10];
    const float vp = aim_vp_from_vn(aim_vn_from_vin(10.0f, vb, &M), vb, &M);
    assert(near(vp, 9.90f, 0.01f));
}

static void test_eval(void)
{
    aim_result_t r;
    ai_ch_cfg_t c = cfg(AI_MODE_V_0_10);

    aim_eval(&c, &M, &D, code_for_vin(AI_MODE_V_0_10, 5.0f), 320u, 0u, 0u, &r);
    assert(r.status == 0u && near(r.raw, 5.0f, 0.005f) && near(r.value, 5.0f, 0.005f));

    aim_eval(&c, &M, &D, code_for_vin(AI_MODE_V_0_10, 10.0f), 320u, 0u, 0u, &r);
    assert(r.status == 0u && near(r.raw, 10.0f, 0.005f));

    /* Above the over threshold: flagged, value still shown. */
    aim_eval(&c, &M, &D, code_for_vin(AI_MODE_V_0_10, 10.3f), 320u, 0u, 0u, &r);
    assert((r.status & AI_ST_OVERRANGE) != 0u && isfinite(r.value));

    /* Half of the samples at the rail. */
    aim_eval(&c, &M, &D, 65535.0f, 320u, 160u, 0u, &r);
    assert((r.status & (AI_ST_OVERRANGE | AI_ST_ADC_CLIP)) == (AI_ST_OVERRANGE | AI_ST_ADC_CLIP));
    aim_eval(&c, &M, &D, 60000.0f, 320u, 159u, 0u, &r);
    assert((r.status & AI_ST_ADC_CLIP) == 0u);

    /* Unipolar: code 0 is just 0 V, never UNDERRANGE. */
    aim_eval(&c, &M, &D, 0.0f, 320u, 0u, 320u, &r);
    assert((r.status & AI_ST_UNDERRANGE) == 0u);

    /* Bipolar. */
    c = cfg(AI_MODE_V_PM10);
    aim_eval(&c, &M, &D, code_for_vin(AI_MODE_V_PM10, -7.5f), 320u, 0u, 0u, &r);
    assert(r.status == 0u && near(r.raw, -7.5f, 0.01f));
    aim_eval(&c, &M, &D, 0.0f, 320u, 0u, 320u, &r);
    assert((r.status & (AI_ST_UNDERRANGE | AI_ST_ADC_CLIP)) == (AI_ST_UNDERRANGE | AI_ST_ADC_CLIP));

    /* 0-5 V. */
    c = cfg(AI_MODE_V_0_5);
    aim_eval(&c, &M, &D, code_for_vin(AI_MODE_V_0_5, 4.0f), 64u, 0u, 0u, &r);
    assert(r.status == 0u && near(r.raw, 4.0f, 0.003f));

    /* 2-10 V live zero. */
    c = cfg(AI_MODE_V_2_10);
    aim_eval(&c, &M, &D, code_for_vin(AI_MODE_V_2_10, 6.0f), 320u, 0u, 0u, &r);
    assert(r.status == 0u && near(r.raw, 6.0f, 0.005f));
    aim_eval(&c, &M, &D, code_for_vin(AI_MODE_V_2_10, 0.3f), 320u, 0u, 0u, &r);
    assert((r.status & AI_ST_WIRE_BREAK) != 0u && !isfinite(r.value) && !isfinite(r.raw));

    /* OFF / no data. */
    c = cfg(AI_MODE_OFF);
    aim_eval(&c, &M, &D, 1000.0f, 320u, 0u, 0u, &r);
    assert(r.status == AI_ST_OFF && !isfinite(r.value));
    c = cfg(AI_MODE_V_0_10);
    aim_eval(&c, &M, &D, 1000.0f, 0u, 0u, 0u, &r);
    assert(r.status == AI_ST_NO_DATA && !isfinite(r.value));
}

static void test_cal_scale(void)
{
    aim_result_t r;
    ai_ch_cfg_t c = cfg(AI_MODE_V_0_10);
    c.cal_gain = 1.01f;
    c.cal_offset_v = -0.02f;
    aim_eval(&c, &M, &D, code_for_vin(AI_MODE_V_0_10, 5.0f), 320u, 0u, 0u, &r);
    assert(near(r.raw, 1.01f * 5.0f - 0.02f, 0.01f));

    /* Engineering scaling: 0..10 V -> 0..100 %. */
    c = cfg(AI_MODE_V_0_10);
    c.scale_lo = 0.0f; c.scale_hi = 100.0f;
    aim_eval(&c, &M, &D, code_for_vin(AI_MODE_V_0_10, 2.5f), 320u, 0u, 0u, &r);
    assert(near(r.value, 25.0f, 0.1f) && near(r.raw, 2.5f, 0.005f));

    /* 2..10 V -> 4..20 mA equivalent. */
    c = cfg(AI_MODE_V_2_10);
    c.scale_lo = 4.0f; c.scale_hi = 20.0f;
    aim_eval(&c, &M, &D, code_for_vin(AI_MODE_V_2_10, 6.0f), 320u, 0u, 0u, &r);
    assert(near(r.value, 12.0f, 0.05f));

    /* lo == hi: identity. */
    c = cfg(AI_MODE_V_0_10);
    c.scale_lo = 5.0f; c.scale_hi = 5.0f;
    assert(aim_scale(&c, 3.3f) == 3.3f);
}

static void test_validation(void)
{
    ai_ch_cfg_t c = cfg(AI_MODE_V_0_10);
    assert(aim_ch_valid(&c));
    c.mode = AI_MODE_COUNT;              assert(!aim_ch_valid(&c));
    c = cfg(1); c.cal_gain = 0.1f;       assert(!aim_ch_valid(&c));
    c = cfg(1); c.cal_gain = NAN;        assert(!aim_ch_valid(&c));
    c = cfg(1); c.cal_offset_v = 2.0f;   assert(!aim_ch_valid(&c));
    c = cfg(1); c.scale_hi = INFINITY;   assert(!aim_ch_valid(&c));

    assert(aim_model_valid(&M) && aim_diag_valid(&D));
    ai_model_t m = M; m.r_in = 10.0f;            assert(!aim_model_valid(&m));
    m = M; m.v_ref = NAN;                        assert(!aim_model_valid(&m));
    m = M; m.v_b[AI_RANGE_P10] = 9.0f;           assert(!aim_model_valid(&m));
    ai_diag_t d = D; d.v_over_v = 20.0f;         assert(!aim_diag_valid(&d));
}

int main(void)
{
    aim_model_defaults(&M);
    aim_diag_defaults(&D);
    test_ranges();
    test_modes();
    test_network();
    test_eval();
    test_cal_scale();
    test_validation();
    puts("8AIV conversion maths tests passed");
    return 0;
}
