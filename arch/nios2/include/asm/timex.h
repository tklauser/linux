#ifndef _ASM_NIOS2_TIMEX_H
#define _ASM_NIOS2_TIMEX_H

#if defined(TIMER_1MS_FREQ)
#define CLOCK_TICK_RATE		TIMER_1MS_FREQ
#else
/* Supply dummy tick-rate. Real value will be read from devicetree */
#define CLOCK_TICK_RATE		(HZ * 100000UL)
#endif

#include <asm-generic/timex.h>

#endif
