/**
  ******************************************************************************
  * @file    flash_ram.h
  * @brief   Sector erase + word program executed from RAM.
  *
  *  An erase stalls every instruction/data fetch from Flash for 1..4 s, and the
  *  HAL erase polls HAL_GetTick, whose interrupt is stalled too. The whole
  *  erase/program loop therefore runs from RAM inside a FreeRTOS critical
  *  section, and the 8AIV scan engine is paused around it
  *  (adc_scan_pause/resume): the window in progress is dropped and the scan
  *  restarts cleanly afterwards.
  ******************************************************************************
  */
#ifndef APPLICATION_FLASH_RAM_H
#define APPLICATION_FLASH_RAM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Erase @p sector, program @p words 32-bit words from @p src at @p addr,
 *  then verify. Blocks the calling thread (and every other task) for the
 *  duration of the erase. */
bool flash_ram_write_sector(uint32_t sector, uint32_t addr, const uint32_t* src, uint32_t words);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_FLASH_RAM_H */
