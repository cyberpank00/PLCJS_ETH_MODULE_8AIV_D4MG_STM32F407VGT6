/**
  ******************************************************************************
  * @file    board_pins.h
  * @brief   8AIV board file — the ONLY place that knows MCU pins, the SPI
  *          instance and the ADC wiring table. Verified against
  *          DOC/SCH_8AIV.pdf (sheet 5 MCU, sheet 3 ADC / isolator):
  *
  *  Isolated-side signals (through U6 CA-IS3741LB):
  *    SCLK  PA5  SPI1_SCK  (AF5)   idle 0 (CPOL = 0)
  *    MISO  PA6  SPI1_MISO (AF5)   ADS8688 SDO
  *    MOSI  PB5  SPI1_MOSI (AF5)   ADS8688 SDI
  *    CS    PA9  GPIO               ADS8688 /CS, active 0 (net ADC0_CS)
  *  ADS8688 /RST/PD is pulled up to 5VCC_ISO (R38) and DAISY / REFSEL are tied
  *  to GND_ISO: no MCU reset line, software RST command, internal 4.096 V
  *  reference. There is no key chain and no output-enable on this board.
  *
  *  Channel status LEDs (same pins as 8AIC / 8UAI, active high):
  *    AI0 PB15, AI1 PB14, AI2 PB10, AI3 PE15, AI4 PE14, AI5 PE13, AI6 PE12,
  *    AI7 PE11.
  *
  *  Header holds only constants and tables (no HAL include) so the host tests
  *  can use them. Pin set-up lives in board_pins.c.
  ******************************************************************************
  */
#ifndef APPLICATION_BOARD_PINS_H
#define APPLICATION_BOARD_PINS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- GPIO port/pin numbers (port index: A=0, B=1, C=2, ... ) -------------- */
#define BOARD_PORT_A        0u
#define BOARD_PORT_B        1u
#define BOARD_PORT_E        4u

#define BOARD_SCLK_PORT     BOARD_PORT_A
#define BOARD_SCLK_PIN      5u
#define BOARD_MISO_PORT     BOARD_PORT_A
#define BOARD_MISO_PIN      6u
#define BOARD_MOSI_PORT     BOARD_PORT_B
#define BOARD_MOSI_PIN      5u
#define BOARD_SPI_AF        5u

#define BOARD_CS_PORT       BOARD_PORT_A
#define BOARD_CS_PIN        9u

/* ---- ADS8688 channel of each input ----------------------------------------
 * Inputs are not wired in order (sheet 3): AIN_0P = AI4, AIN_1P = AI5,
 * AIN_2P = AI3, AIN_3P = AI2, AIN_4P = AI1, AIN_5P = AI0, AIN_6P = AI7,
 * AIN_7P = AI6. MAN_Ch_n, the range register (0x05 + n) and the pipeline
 * bookkeeping use BOARD_ADC_MAP[input] = n; nothing else may. */
static const uint8_t BOARD_ADC_MAP[8] = { 5u, 4u, 3u, 2u, 0u, 1u, 7u, 6u };

/* ---- Channel status LEDs ---------------------------------------------------- */
static const uint8_t BOARD_LED_PORT[8] = {
    BOARD_PORT_B, BOARD_PORT_B, BOARD_PORT_B, BOARD_PORT_E,
    BOARD_PORT_E, BOARD_PORT_E, BOARD_PORT_E, BOARD_PORT_E,
};
static const uint8_t BOARD_LED_PIN[8] = { 15u, 14u, 10u, 15u, 14u, 13u, 12u, 11u };

/* ---- Functions (board_pins.c) ---------------------------------------------- */
/** First statement of main(): CS = 1 and SCLK = 0 as outputs (level written to
 *  BSRR before the pin is switched to output). */
void board_early_pins(void);
/** Hand SCLK/MISO/MOSI to SPI1 (AF5). Call after the SPI is configured and
 *  enabled with CPOL = 0, so SCLK stays low across the switch. */
void board_spi_pins_af(void);
/** Channel status LEDs as outputs, off. */
void board_led_init(void);
void board_led_set(uint8_t ch, uint8_t on);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_BOARD_PINS_H */
