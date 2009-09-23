#ifndef _ASM_NIOS2_TIMEX_H
#define _ASM_NIOS2_TIMEX_H

#include <asm/nios.h>

#ifndef  TIMER_1MS_FREQ
#error Your FPGA design does not contain a TIMER_1MS
#endif

#define CLOCK_TICK_RATE	 TIMER_1MS_FREQ /* Underlying HZ */

#include <asm-generic/timex.h>

#endif
