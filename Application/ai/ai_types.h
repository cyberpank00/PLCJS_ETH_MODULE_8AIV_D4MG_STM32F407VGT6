/**
  ******************************************************************************
  * @file    ai_types.h
  * @brief   8AIV analog front-end: shared types (no HAL dependency).
  *
  *  Used by the persistent settings, the conversion maths (ai_math, host
  *  tested), the channel layer (ai) and the Modbus adapter. Channel numbers
  *  everywhere are input numbers AI0..AI7; the ADS8688 channel number never
  *  leaves board_pins.h / adc_scan.c.
  ******************************************************************************
  */
#ifndef APPLICATION_AI_TYPES_H
#define APPLICATION_AI_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AI_CHANNELS         8u

/* Channel mode. Stored in Flash and on Modbus: keep the values. */
typedef enum {
    AI_MODE_OFF     = 0,
    AI_MODE_V_0_10  = 1,   /* 0..10 V,  ADC 0..10.24 V                        */
    AI_MODE_V_2_10  = 2,   /* 2..10 V live zero, below 1.8 V = wire break      */
    AI_MODE_V_0_5   = 3,   /* 0..5 V,   ADC 0..5.12 V                         */
    AI_MODE_V_PM10  = 4,   /* ±10 V,    ADC ±10.24 V                          */
    AI_MODE_V_PM5   = 5,   /* ±5 V,     ADC ±5.12 V                           */
    AI_MODE_COUNT
} ai_mode_t;

/* ADS8688 input ranges used by the modes (index into the range table). */
typedef enum {
    AI_RANGE_PM10 = 0,   /* ±2.5·VREF    code 0x00  ±10.24 V  V_B 2.25 V */
    AI_RANGE_PM5  = 1,   /* ±1.25·VREF   code 0x01  ±5.12 V   V_B 2.00 V */
    AI_RANGE_P10  = 2,   /* 0..2.5·VREF  code 0x05  0..10.24  V_B 2.50 V */
    AI_RANGE_P5   = 3,   /* 0..1.25·VREF code 0x06  0..5.12   V_B 2.50 V */
    AI_RANGE_COUNT
} ai_range_t;

/* Channel status bits (Modbus IR332..339). Any bit except OVERRANGE /
 * UNDERRANGE / ADC_CLIP makes the value NaN. */
#define AI_ST_NO_DATA       0x0001u
#define AI_ST_OVERRANGE     0x0002u
#define AI_ST_UNDERRANGE    0x0004u
#define AI_ST_WIRE_BREAK    0x0008u
#define AI_ST_ADC_CLIP      0x0010u   /* samples at the ADC rail                */
#define AI_ST_CONFIG_ERR    0x0020u
#define AI_ST_ADC_FAIL      0x0040u   /* ADS8688 does not answer / lost config  */
#define AI_ST_OFF           0x0080u   /* channel mode OFF                       */
#define AI_ST_VALUE_OK_MASK (AI_ST_OVERRANGE | AI_ST_UNDERRANGE)

/* Input-network model (sheet 3): Vin -> R_top -> node (R_shunt to GND) ->
 * R_adc -> ADS8688 pin, whose input is R_in to the bias V_B of the range.
 * Service parameters. */
typedef struct {
    float v_ref;      /* ADS8688 internal reference, V        (4.096)  */
    float r_top;      /* R34/R36/... series, Ω                (1000)   */
    float r_shunt;    /* R35/R45/... node -> GND, Ω           (120000) */
    float r_adc;      /* R33/R40/... node -> AIN pin, Ω       (1000)   */
    float r_in;       /* ADS8688 input resistance, Ω          (1e6)    */
    float v_b[AI_RANGE_COUNT];   /* ADC input bias per range, V      */
} ai_model_t;

/* Result-diagnostic thresholds. Service parameters. */
typedef struct {
    float v210_open_v;   /* 1.8  : 2..10 V mode, below -> WIRE_BREAK       */
    float v_over_v;      /* 10.2 : 0..10 / 2..10 V modes, above -> OVERRANGE */
} ai_diag_t;

/* Per-channel configuration. Stored in Flash: layout frozen. */
typedef struct {
    float    cal_gain;        /* V_cal = cal_gain · V_in + cal_offset_v    */
    float    cal_offset_v;
    float    scale_lo;        /* engineering value at the signal low end   */
    float    scale_hi;        /* ... at the high end; lo == hi -> volts    */
    uint16_t mode;            /* ai_mode_t                                 */
    uint16_t filter_tau_ms;   /* EMA time constant, 0 = off                */
} ai_ch_cfg_t;

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_AI_TYPES_H */
