#include "stm8l15x.h"

#include "delay.h"
#include "timer.h"

void delay_ms(u16 time_ms)   // ms
{
  u16 tick;
  while (time_ms != 0)
  {
    time_ms--;
    tick = (2666 >> SYS_CLOCK_DIVIDER);
    while (tick != 0)
    {
      tick --;
    }
  }
}

void delay_10us(u16 time_10us)
{
  u16 tick;
  while (time_10us != 0)
  {
    time_10us--;
    tick = (26 >> SYS_CLOCK_DIVIDER);
    while (tick != 0)
    {
      tick --;
    }
  }
}
