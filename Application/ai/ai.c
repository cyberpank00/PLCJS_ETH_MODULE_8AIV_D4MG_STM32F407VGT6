/**
  ******************************************************************************
  * @file    ai.c
  * @brief   8AIV analog-input channel layer (see ai.h).
  ******************************************************************************
  */
#include "ai.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#include "adc_scan.h"
#include "ads8688.h"
#include "ai_math.h"
#include "board_io.h"
#include "settings.h"
#include "stm32f4xx_hal.h"
#include "wdg.h"

#define AI_BOOT_WAIT_MS     500u    /* isolated supply + ADS8688 power-up      */
#define AI_RETRY_MS         1000u

typedef struct {
    ai_ch_cfg_t cfg;
    uint8_t     exp_range;      /* ADS8688 range code the config needs         */
    bool        settle;         /* waiting for a window with exp_range in force */
    bool        ema_valid;
    float       ema;            /* filtered volts                              */
    ai_data_t   data;
    ai_stats_t  stats;
} rt_t;

static rt_t        s_rt[AI_CHANNELS];
static ai_model_t  s_model;
static ai_diag_t   s_diag;
static uint16_t    s_window_ms;
static bool        s_cfg_err;
static bool        s_adc_fail;       /* ADS8688 did not come up                 */
static uint16_t    s_fail_windows;   /* ADC_FAIL hold counter                   */
static uint16_t    s_last_range_err;
static uint32_t    s_last_seq;
static uint16_t    s_win_counter;
static uint16_t    s_samples;
static uint16_t    s_init_attempts;
static uint32_t    s_next_retry_ms;

static uint32_t now_ms(void) { return HAL_GetTick(); }

static uint8_t range_code_of(uint8_t mode)
{
    return aim_range_code(aim_mode_range(mode));
}

static void reset_channel(rt_t* r)
{
    r->ema_valid = false;
    r->settle    = true;
    memset(&r->stats, 0, sizeof(r->stats));
    r->data.value  = __builtin_nanf("");
    r->data.raw    = __builtin_nanf("");
    r->data.status = (r->cfg.mode == AI_MODE_OFF) ? AI_ST_OFF : AI_ST_NO_DATA;
    r->data.state  = (r->cfg.mode == AI_MODE_OFF) ? AI_STATE_OFF : AI_STATE_SETTLE;
}

/* ---- ADC start-up --------------------------------------------------------- */
static bool adc_bring_up(void)
{
    s_init_attempts++;
    uint8_t ranges[AI_CHANNELS];
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) { ranges[ch] = s_rt[ch].exp_range; }

    /* RST, every range = 0..2.5·VREF, read back; then the per-input ranges. */
    if (!ads8688_init(aim_range_code(AI_RANGE_P10))) { return false; }
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        if (ranges[ch] != aim_range_code(AI_RANGE_P10) &&
            !ads8688_set_range(BOARD_ADC_MAP[ch], ranges[ch])) {
            return false;
        }
    }
    adc_scan_start(ranges, s_window_ms);
    return true;
}

bool ai_init(void)
{
    const settings_t* s = settings_get();
    s_cfg_err   = settings_config_error();
    s_model     = s->model;
    s_diag      = s->diag;
    s_window_ms = s->window_ms;
    if (!ai_common_valid(&s_model, &s_diag, s_window_ms)) {
        aim_model_defaults(&s_model);
        aim_diag_defaults(&s_diag);
        s_window_ms = SETTINGS_DEF_WINDOW_MS;
        s_cfg_err   = true;
    }
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        memset(&s_rt[ch], 0, sizeof(s_rt[ch]));
        s_rt[ch].cfg = s->ch[ch];
        if (!aim_ch_valid(&s_rt[ch].cfg)) { aim_ch_defaults(&s_rt[ch].cfg); s_cfg_err = true; }
        s_rt[ch].exp_range = range_code_of(s_rt[ch].cfg.mode);
        reset_channel(&s_rt[ch]);
    }

    board_led_init();
    spi_init();

    while (HAL_GetTick() < AI_BOOT_WAIT_MS) {
        wdg_kick();
        osDelay(10u);
    }

    s_adc_fail = !adc_bring_up();
    s_next_retry_ms = now_ms() + AI_RETRY_MS;
    return !s_adc_fail;
}

/* ---- window -> values -------------------------------------------------------- */
static void process_window(const adc_win_t* w)
{
    s_win_counter++;
    s_samples = w->n[0];

    if (w->range_err != s_last_range_err) {
        s_last_range_err = w->range_err;
        s_fail_windows = AI_FAIL_HOLD_WINDOWS;
    } else if (s_fail_windows > 0u) {
        s_fail_windows--;
    }

    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        rt_t* r = &s_rt[ch];
        taskENTER_CRITICAL();
        const ai_ch_cfg_t cfg = r->cfg;
        const uint8_t exp = r->exp_range;
        taskEXIT_CRITICAL();

        ai_data_t d = r->data;
        ai_stats_t st = r->stats;
        bool settle = r->settle;

        if (cfg.mode == AI_MODE_OFF) {
            d.value = d.raw = __builtin_nanf("");
            d.status = AI_ST_OFF;
            d.state  = AI_STATE_OFF;
            r->ema_valid = false;
        } else if (w->range[ch] != exp) {
            settle = true;
            d.value = d.raw = __builtin_nanf("");
            d.status = AI_ST_NO_DATA;
            d.state  = AI_STATE_SETTLE;
            r->ema_valid = false;
        } else {
            settle = false;
            const uint16_t n = w->n[ch];
            const float mean = (n != 0u) ? (float)w->sum[ch] / (float)n : 0.0f;
            st.n = n;
            st.mean = (uint16_t)(mean + 0.5f);
            st.vmin = w->vmin[ch];
            st.vmax = w->vmax[ch];
            st.clip_hi = w->clip_hi[ch];
            st.clip_lo = w->clip_lo[ch];
            if (n != 0u) {
                const float var = (float)((double)w->sumsq[ch] / (double)n - (double)mean * (double)mean);
                st.rms = (var > 0.0f) ? sqrtf(var) : 0.0f;
            } else {
                st.rms = 0.0f;
            }

            aim_result_t res;
            aim_eval(&cfg, &s_model, &s_diag, mean, n, st.clip_hi, st.clip_lo, &res);

            if (cfg.filter_tau_ms != 0u && isfinite(res.raw)) {
                if (!r->ema_valid) { r->ema = res.raw; r->ema_valid = true; }
                else {
                    const float a = (float)w->win_ms / ((float)cfg.filter_tau_ms + (float)w->win_ms);
                    r->ema += a * (res.raw - r->ema);
                }
                res.raw   = r->ema;
                res.value = aim_scale(&cfg, r->ema);
            } else if (!isfinite(res.raw)) {
                r->ema_valid = false;
            }

            d.value  = res.value;
            d.raw    = res.raw;
            d.status = res.status;
            d.state  = AI_STATE_RUN;
            d.seq++;
            if (s_adc_fail || s_fail_windows > 0u) {
                d.status |= AI_ST_ADC_FAIL;
                d.value = d.raw = __builtin_nanf("");
                d.state = AI_STATE_ADC_FAIL;
            }
            if (s_cfg_err) { d.status |= AI_ST_CONFIG_ERR; }
        }

        taskENTER_CRITICAL();
        r->data = d;
        r->stats = st;
        r->settle = settle;
        taskEXIT_CRITICAL();
    }
}

void ai_poll(void)
{
    if (!adc_scan_running()) {
        if ((int32_t)(now_ms() - s_next_retry_ms) >= 0) {
            s_adc_fail = !adc_bring_up();
            s_next_retry_ms = now_ms() + AI_RETRY_MS;
        }
        return;
    }
    wdg_kick();
    adc_win_t w;
    if (adc_scan_take(&w, s_last_seq)) {
        s_last_seq = w.seq;
        process_window(&w);
    }
}

/* ---- configuration ------------------------------------------------------------ */
bool ai_set_config(uint8_t ch, const ai_ch_cfg_t* cfg)
{
    if (ch >= AI_CHANNELS || !aim_ch_valid(cfg)) { return false; }
    rt_t* r = &s_rt[ch];
    const uint8_t code = range_code_of(cfg->mode);
    taskENTER_CRITICAL();
    const bool mode_changed = (cfg->mode != r->cfg.mode);
    const bool range_changed = (code != r->exp_range);
    const bool tau_changed = (cfg->filter_tau_ms != r->cfg.filter_tau_ms);
    r->cfg = *cfg;
    r->exp_range = code;
    if (mode_changed || range_changed || tau_changed) { reset_channel(r); }
    taskEXIT_CRITICAL();
    if (range_changed && adc_scan_running()) { adc_scan_set_range(ch, code); }
    return true;
}

void ai_get_config(uint8_t ch, ai_ch_cfg_t* out)
{
    if (ch >= AI_CHANNELS) { return; }
    taskENTER_CRITICAL();
    *out = s_rt[ch].cfg;
    taskEXIT_CRITICAL();
}

bool ai_common_valid(const ai_model_t* m, const ai_diag_t* d, uint16_t window_ms)
{
    return aim_model_valid(m) && aim_diag_valid(d) && (window_ms == 20u || window_ms == 100u);
}

bool ai_set_common(const ai_model_t* m, const ai_diag_t* d, uint16_t window_ms)
{
    if (!ai_common_valid(m, d, window_ms)) { return false; }
    taskENTER_CRITICAL();
    s_model = *m;
    s_diag  = *d;
    s_window_ms = window_ms;
    taskEXIT_CRITICAL();
    if (adc_scan_running()) { (void)adc_scan_set_window(window_ms); }
    return true;
}

void ai_get_common(ai_model_t* m, ai_diag_t* d, uint16_t* window_ms)
{
    taskENTER_CRITICAL();
    *m = s_model;
    *d = s_diag;
    *window_ms = s_window_ms;
    taskEXIT_CRITICAL();
}

bool ai_command(uint8_t ch, uint8_t cmd)
{
    if (cmd == AI_CMD_APPLY) {
        const settings_t* s = settings_get();
        bool ok = ai_set_common(&s->model, &s->diag, s->window_ms);
        for (uint8_t i = 0u; i < AI_CHANNELS; i++) { ok = ai_set_config(i, &s->ch[i]) && ok; }
        return ok;
    }
    if (cmd == AI_CMD_RESTART && ch < AI_CHANNELS) {
        taskENTER_CRITICAL();
        reset_channel(&s_rt[ch]);
        taskEXIT_CRITICAL();
        return true;
    }
    return false;
}

/* ---- accessors ------------------------------------------------------------------- */
void ai_get(uint8_t ch, ai_data_t* out)
{
    if (ch >= AI_CHANNELS) { memset(out, 0, sizeof(*out)); return; }
    taskENTER_CRITICAL();
    *out = s_rt[ch].data;
    taskEXIT_CRITICAL();
}

void ai_get_stats(uint8_t ch, ai_stats_t* out)
{
    if (ch >= AI_CHANNELS) { memset(out, 0, sizeof(*out)); return; }
    taskENTER_CRITICAL();
    *out = s_rt[ch].stats;
    taskEXIT_CRITICAL();
}

int16_t ai_get_int16(uint8_t ch)
{
    ai_data_t d;
    ai_get(ch, &d);
    if ((d.status & (uint16_t)~(AI_ST_VALUE_OK_MASK | AI_ST_ADC_CLIP)) != 0u || !isfinite(d.raw)) {
        return AI_I16_INVALID;
    }
    const float v = d.raw * 1000.0f;   /* mV */
    if (v > 32767.0f)  { return 32767; }
    if (v < -32767.0f) { return -32767; }
    return (int16_t)lrintf(v);
}

uint16_t ai_module_flags(void)
{
    uint16_t f = 0u;
    if (s_adc_fail || s_fail_windows > 0u) { f |= AI_MOD_ADC_FAIL; }
    if (s_cfg_err)                          { f |= AI_MOD_CONFIG_ERR; }
    if (adc_scan_running())                 { f |= AI_MOD_RUNNING; }
    if (s_fail_windows > 0u)                { f |= AI_MOD_RANGE_ERR; }
    if (adc_scan_overruns() != 0u)          { f |= AI_MOD_OVERRUN; }
    return f;
}

uint16_t ai_samples_per_window(void) { return s_samples; }
uint16_t ai_window_counter(void)     { return s_win_counter; }
uint16_t ai_init_attempts(void)      { return s_init_attempts; }

void ai_led_tick(uint16_t period_ms)
{
    static uint16_t t;
    t = (uint16_t)((t + period_ms) % 1000u);
    const bool slow = t < 500u;
    const bool fast = (t % 250u) < 125u;
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        ai_data_t d;
        ai_get(ch, &d);
        bool on;
        if (d.state == AI_STATE_OFF)                                                   { on = false; }
        else if ((d.status & (uint16_t)~(AI_ST_NO_DATA | AI_ST_VALUE_OK_MASK | AI_ST_CONFIG_ERR)) != 0u) { on = fast; }
        else if ((d.status & AI_ST_NO_DATA) != 0u)                                    { on = slow; }
        else                                                                          { on = true; }
        board_led_set(ch, on ? 1u : 0u);
    }
}
