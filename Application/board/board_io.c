/**
  ******************************************************************************
  * @file    board_io.c
  * @brief   8AIV thin hardware layer (see board_io.h).
  ******************************************************************************
  */
#include "board_io.h"

void spi_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    (void)RCC->APB2ENR;

    BOARD_SPI->CR1 = 0u;
    BOARD_SPI->CR2 = 0u;
    /* Master, PCLK2/8 = 10 MHz, software NSS, CPOL 0, CPHA 1, 16-bit. */
    BOARD_SPI->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_SSM | SPI_CR1_SSI |
                     SPI_CR1_CPHA | SPI_CR1_DFF;
    BOARD_SPI->CR1 |= SPI_CR1_SPE;

    board_spi_pins_af();
}

BOARD_INLINE void wait_idle(void)
{
    while ((BOARD_SPI->SR & SPI_SR_TXE) == 0u) { }
    while ((BOARD_SPI->SR & SPI_SR_BSY) != 0u) { }
}

uint32_t board_adc_frame(uint16_t cmd)
{
    while ((BOARD_SPI->SR & SPI_SR_RXNE) != 0u) { (void)BOARD_SPI->DR; }
    (void)BOARD_SPI->SR;   /* DR then SR read clears OVR */

    board_cs(0u);
    board_delay_100ns();
    BOARD_SPI->DR = cmd;
    while ((BOARD_SPI->SR & SPI_SR_TXE) == 0u) { }
    BOARD_SPI->DR = 0x0000u;
    while ((BOARD_SPI->SR & SPI_SR_RXNE) == 0u) { }
    const uint16_t first = (uint16_t)BOARD_SPI->DR;
    while ((BOARD_SPI->SR & SPI_SR_RXNE) == 0u) { }
    const uint16_t data = (uint16_t)BOARD_SPI->DR;
    wait_idle();
    board_delay_100ns();
    board_cs(1u);
    board_delay_100ns();

    return ((uint32_t)first << 16) | data;
}
