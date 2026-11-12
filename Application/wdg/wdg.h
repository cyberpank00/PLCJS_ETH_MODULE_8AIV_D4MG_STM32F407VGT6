/**
  ******************************************************************************
  * @file    wdg.h
  * @brief   8AIV watchdog policy. MX_IWDG_Init() starts the IWDG at ~8 s like
  *          every module of the family (it also takes over a counter left
  *          running by the bootloader); threads refresh it through wdg_kick().
  *          There is no protection ISR on this board, so the period is never
  *          tightened.
  ******************************************************************************
  */
#ifndef APPLICATION_WDG_H
#define APPLICATION_WDG_H

#ifdef __cplusplus
extern "C" {
#endif

void wdg_kick(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_WDG_H */
