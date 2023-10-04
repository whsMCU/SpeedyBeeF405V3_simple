/*
 * bsp.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */


#include "bsp.h"
#include "uart.h"


// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;

void HAL_SYSTICK_Callback(void)
{
    sysTickUptime++;
}

void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}

uint32_t millis(void)
{
  return HAL_GetTick();
}

// Return system uptime in microseconds (rollover in 70minutes)

uint32_t micros(void)
{
//	register uint32_t ms, cycle_cnt;
//
//	do {
//		ms = sysTickUptime;
//		cycle_cnt = SysTick->VAL;
//	} while (ms != sysTickUptime);
//	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks; //168
	return htim5.Instance->CNT;
}

void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}

int __io_putchar(int ch)
{
  uartWrite(_DEF_UART1, (uint8_t *)&ch, 1);
  return 1;
}
