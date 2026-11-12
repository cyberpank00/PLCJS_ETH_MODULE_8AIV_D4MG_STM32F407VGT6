/**
  ******************************************************************************
  * @file    wdg.c
  * @brief   8AIV watchdog refresh (see wdg.h). Register level IWDG.
  ******************************************************************************
  */
#include "wdg.h"

#include "stm32f4xx.h"

void wdg_kick(void)
{
    IWDG->KR = 0xAAAAu;
}
