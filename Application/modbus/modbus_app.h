/**
  ******************************************************************************
  * @file    modbus_app.h
  * @brief   Modbus register-map adapter for the 8AIV voltage analog-input
  *          module (8 inputs, 0-10 V / 2-10 V / 0-5 V / ±10 V / ±5 V, one
  *          ADS8688 behind an isolator).
  *
  *  Float values are IEEE-754 32-bit, transmitted as two 16-bit registers with
  *  the HIGH word first (big-endian word order: register[N] = bits 31..16).
  *  Multi-channel quantities are grouped by quantity: 8 consecutive registers
  *  (or 8 register pairs) are channels AI0..AI7.
  *
  *  ---- Holding Registers (FC03/06/16) — compact per-channel block --------
  *      0..7    reading, int16 (RO): mV. −32768 (0x8000) = no valid value.
  *      8..15   mode: 0 OFF, 1 V 0-10, 2 V 2-10 (live zero), 3 V 0-5,
  *                4 V ±10, 5 V ±5 (default 1). A change settles the channel
  *                for one window.
  *      16..23  filter time constant, ms (EMA over windows, 0 = off, default 0)
  *
  *  ---- Holding Registers — per-channel float32 parameters (×8 each) ------
  *      24..39  cal gain (1.0)        40..55  cal offset, V (0.0)
  *      56..71  scale low             72..87  scale high
  *              (engineering value at the low / high end of the signal
  *               range; low == high -> volts). V_cal = gain · V_in + offset.
  *
  *  ---- Holding Registers — common analog block (service) ------------------
  *      132 integration window, ms: 100 (default; whole periods of 50 AND
  *          60 Hz, 320 samples) or 20 (50 Hz only, 64 samples)
  *      133..150 input-network model, float32: 133 V_REF (4.096),
  *               135 R_top (1000 Ω), 137 R_shunt (120000 Ω), 139 R_adc (1000 Ω),
  *               141 R_in (1e6 Ω), 143/145/147/149 V_B of the
  *               ±10.24 / ±5.12 / 0..10.24 / 0..5.12 V ranges
  *      151..154 diagnostics, float32: 151 V2-10 open V (1.8), 153 V over V (10.2)
  *
  *    Writes are staged in the settings and applied once the channel (or the
  *    common block) is consistent. An FC16 request that leaves it
  *    inconsistent is rejected as a whole (exception 3); an FC06 half of a
  *    float is staged until the other half arrives.
  *
  *  ---- Input Registers (FC04, read-only) — grouped by quantity ----------
  *      300..315 value, float32 ×8 (engineering value; NaN if invalid)
  *      316..331 raw, float32 ×8 (calibrated input voltage, V)
  *      332..339 status bits ×8: 0 NO_DATA, 1 OVERRANGE, 2 UNDERRANGE,
  *                 3 WIRE_BREAK, 4 ADC_CLIP, 5 CONFIG_ERR, 6 ADC_FAIL,
  *                 7 OFF. Any bit except 1/2/4 -> value NaN.
  *      340..347 update counter ×8 (u16, wraps)
  *      348..355 mean ADC code of the last window ×8 (u16)
  *      356..363 channel state ×8: 0 OFF, 1 SETTLE, 2 RUN, 3 ADC_FAIL
  *      364..371 min ADC code ×8       372..379 max ADC code ×8
  *      380..395 code noise, LSB RMS, float32 ×8
  *      396..403 samples at 0xFFFF ×8  404..411 samples at 0x0000 ×8
  *    Global:
  *      120 fw major, 121 fw minor, 122/123 uptime s (lo/hi),
  *      125 module id (0x08A5), 126 on-chip temperature (signed 0.1 °C)
  *      127 module flags: bit0 ADC_FAIL, bit1 CONFIG_ERR, bit2 scan running,
  *          bit3 range read-back mismatch (recent), bit4 slot overrun seen
  *      128 samples per input in the last window
  *      129 settings saves since boot (Flash written + read-back), 130 last save
  *      result (0 none, 1 ok, 2 failed)
  *      131 windows completed (u16, wraps)   132 slot overruns
  *      133 range read-back errors           134 ADC bring-up attempts
  *
  *  ---- Holding Registers (FC03/06/16) — global -----------------------------
  *      101 LED mode (0/1/2)                   102 Modbus slave id
  *      103 Modbus TCP port                    104..107 static IP octets
  *      108..111 netmask octets                112..115 gateway octets
  *      116 net mode (0 static/1 DHCP/2 LL)    117 SAVE trigger (0xA5A5)
  *      118 REBOOT (0xB00B) / BOOT (0xB007) / KSZ8863 reset (0x8863)
  *      119 FACTORY RESET trigger (0xDEAD)     130 on-chip temperature (RO)
  *      131 channel command: 0xA200|ch RESTART (drop filter, new window),
  *          0xA300 APPLY (re-apply the whole stored configuration)
  ******************************************************************************
  */
#ifndef APPLICATION_MODBUS_APP_H
#define APPLICATION_MODBUS_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "nanomodbus.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Magic write triggers (FC06/FC10 to specific holding registers). */
#define MODBUS_TRIG_SAVE            0xA5A5u
#define MODBUS_TRIG_REBOOT          0xB00Bu
#define MODBUS_TRIG_FACTORY_RESET   0xDEADu
#define MODBUS_TRIG_BOOTLOADER      0xB007u
/* Hardware-reset the KSZ8863 Ethernet switch (operator recovery command).
 * Written to MB_HR_TRIG_REBOOT (118). Interrupts pass-through traffic for
 * the duration of the chip reset + port auto-negotiation — use only when the
 * switch shows no signs of life. */
#define MODBUS_TRIG_SWITCH_RESET    0x8863u

/* Channel commands (HR131). */
#define MB_CMD_RESTART_BASE         0xA200u
#define MB_CMD_APPLY                0xA300u
#define MB_CMD_CH_MASK              0x00FFu

/* No-init RAM cell shared with the bootloader. */
#define BOOT_REQUEST_FLAG_ADDR      0x2001FFF0u
#define BOOT_REQUEST_MAGIC          0xB007CAFEu

/* ---- Global holding registers ---- */
#define MB_HR_LED_MODE              101u
#define MB_HR_SLAVE_ID              102u
#define MB_HR_TCP_PORT              103u
#define MB_HR_IP_BASE               104u
#define MB_HR_NETMASK_BASE          108u
#define MB_HR_GATEWAY_BASE          112u
#define MB_HR_USE_DHCP              116u
#define MB_HR_TRIG_SAVE             117u
#define MB_HR_TRIG_REBOOT           118u
#define MB_HR_TRIG_FACTORY_RESET    119u
#define MB_HR_TEMPERATURE           130u
#define MB_HR_CH_COMMAND            131u
#define MB_HR_WINDOW_MS             132u

/* ---- Global input registers ---- */
#define MB_IR_FW_VER_MAJOR          120u
#define MB_IR_FW_VER_MINOR          121u
#define MB_IR_UPTIME_LO             122u
#define MB_IR_UPTIME_HI             123u
#define MB_IR_MODULE_ID             125u
#define MB_IR_TEMPERATURE           126u
#define MB_IR_MODULE_FLAGS          127u
#define MB_IR_SAMPLES               128u
#define MB_IR_SAVE_COUNT            129u  /* successful settings saves since boot */
#define MB_IR_SAVE_RESULT           130u  /* last save: 0 none, 1 ok, 2 failed    */
#define MB_IR_WINDOWS               131u
#define MB_IR_OVERRUNS              132u
#define MB_IR_RANGE_ERRORS          133u
#define MB_IR_INIT_ATTEMPTS         134u

#define MB_AI_CHANNELS              8u

/* ---- Compact per-channel block (holding): address = group*8 + ch ---- */
#define MB_HR_CH_BASE               0u
#define MB_HR_CH_GROUP_READING      0u    /* int16, read-only */
#define MB_HR_CH_GROUP_MODE         1u
#define MB_HR_CH_GROUP_FILTER       2u
#define MB_HR_CH_GROUPS             3u    /* registers 0..23 */

/* ---- Per-channel float parameters (holding), grouped by quantity ---- */
#define MB_HR_CH_FLOAT_BASE         24u   /* gain, offset, scale lo, scale hi: 4 x 16 -> 24..87 */
#define MB_HR_CH_FLOAT_GROUPS       4u

/* ---- Common analog block (holding, float32) ---- */
#define MB_HR_MODEL_BASE            133u  /* 9 floats -> 133..150 */
#define MB_HR_MODEL_COUNT           9u
#define MB_HR_DIAG_BASE             151u  /* 2 floats -> 151..154 */
#define MB_HR_DIAG_COUNT            2u

/* ---- Readings (input registers), grouped by quantity ---- */
#define MB_IR_AI_BASE               300u
#define MB_IR_AI_VALUE              300u  /* float32 ×8 -> 300..315 */
#define MB_IR_AI_RAW                316u  /* float32 ×8 -> 316..331 */
#define MB_IR_AI_STATUS             332u  /* u16 ×8     -> 332..339 */
#define MB_IR_AI_SEQ                340u  /* u16 ×8     -> 340..347 */
#define MB_IR_AI_MEAN               348u  /* u16 ×8     -> 348..355 */
#define MB_IR_AI_STATE              356u  /* u16 ×8     -> 356..363 */
#define MB_IR_AI_MIN                364u  /* u16 ×8     -> 364..371 */
#define MB_IR_AI_MAX                372u  /* u16 ×8     -> 372..379 */
#define MB_IR_AI_RMS                380u  /* float32 ×8 -> 380..395 */
#define MB_IR_AI_CLIP_HI            396u  /* u16 ×8     -> 396..403 */
#define MB_IR_AI_CLIP_LO            404u  /* u16 ×8     -> 404..411 */
#define MB_IR_AI_END                412u  /* first address past the block */

/* Module ID (input register 125). */
#define MODULE_ID_08AIV             0x08A5u

/** Initialise the modbus register adapter. Must be called after settings_init(). */
void modbus_app_init(void);

/** Mark a successful modbus transaction (used to drive STAT_LED state). */
void modbus_app_notify_request(void);

/** Get the populated nmbs_callbacks structure for nmbs_server_create(). */
const nmbs_callbacks* modbus_app_get_callbacks(void);

/** Before a settings save: replace staged, inconsistent analog parameters
 *  (e.g. half of a float written by FC06) with the configuration in force,
 *  so an invalid record never reaches Flash. */
void modbus_app_sanitize_staged(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_MODBUS_APP_H */
