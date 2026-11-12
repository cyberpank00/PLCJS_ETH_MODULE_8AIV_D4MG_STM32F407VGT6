/**
  ******************************************************************************
  * @file    board_io.h
  * @brief   8AIV thin hardware layer of the ADS8688 bus: SPI1 set-up and one
  *          complete ADC frame. Register level, no HAL.
  *
  *  Bus: SPI1 master, CPOL = 0, CPHA = 1 (ADS8688 mode 1), 16-bit frames, at
  *  PCLK2/8 = 10.0 MHz (SYSCLK 160 MHz, APB2 80 MHz; 10 MHz is the ceiling of
  *  the isolator round trip). The bus never changes mode: there is nothing
  *  else on it.
  ******************************************************************************
  */
#ifndef APPLICATION_BOARD_IO_H
#define APPLICATION_BOARD_IO_H

#include <stdint.h>

#include "board_pins.h"
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_RAMFUNC   __attribute__((section(".RamFunc"), noinline))
#define BOARD_INLINE    static inline __attribute__((always_inline))

#define BOARD_GPIO(port)  ((GPIO_TypeDef*)(GPIOA_BASE + (uint32_t)(port) * 0x400u))
#define BOARD_SPI         SPI1

/** /CS level (BSRR, atomic). */
BOARD_INLINE void board_cs(uint8_t level)
{
    BOARD_GPIO(BOARD_CS_PORT)->BSRR = 1u << (BOARD_CS_PIN + (level ? 0u : 16u));
}

/** ≥ 100 ns busy wait (SYSCLK 160 MHz, 24 cycles + call overhead). */
BOARD_INLINE void board_delay_100ns(void)
{
    for (volatile uint8_t i = 0u; i < 6u; i++) { __NOP(); }
}

/** Configure SPI1 (master, CPOL 0, CPHA 1, 16-bit, PCLK2/8) and hand the pins
 *  to it. */
void spi_init(void);

/** One complete ADS8688 frame: CS low, 16-bit command, 16 bits of data,
 *  CS high. Returns (first halfword << 16) | data. The data halfword is the
 *  conversion result of the channel selected in the PREVIOUS frame. */
uint32_t board_adc_frame(uint16_t cmd);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_BOARD_IO_H */
