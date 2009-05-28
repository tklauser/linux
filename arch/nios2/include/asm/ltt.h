/*
 * linux/include/asm-nios2/ltt.h
 *
 * Copyright (C) 2005, Mathieu Desnoyers
 * 
 * nios2 definitions for tracing system
 * 
 * Added for standard wrlinux support.
 *
 */

#ifndef _ASM_NIOS2_LTT_H
#define _ASM_NIOS2_LTT_H

#if 0
#define LTT_HAS_TSC

/* Current arch type */
#define LTT_ARCH_TYPE LTT_ARCH_TYPE_MIPS

/* Current variant type */
#define LTT_ARCH_VARIANT LTT_ARCH_VARIANT_NONE

#include <linux/ltt-core.h>
#include <asm/timex.h>
#include <asm/processor.h>

u64 ltt_heartbeat_read_synthetic_tsc(void);

/* MIPS get_cycles only returns a 32 bits TSC (see timex.h). The assumption
 * there is that the reschedule is done every 8 seconds or so, so we must
 * make sure there is at least an event (heartbeat) between  each TSC wrap
 * around. We use the LTT synthetic TSC exactly for this. */
#endif
static inline u32 ltt_get_timestamp32(void)
{
  BUG();
}

static inline u64 ltt_get_timestamp64(void)
{
  BUG();
}

static inline unsigned int ltt_frequency(void)
{
  BUG();
}

static inline u32 ltt_freq_scale(void)
{
  BUG();
}

#endif //_ASM_MIPS_LTT_H
