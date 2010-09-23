/*
 * Copyright (C) 2004 Microtronix Datacom Ltd.
 *
 * Licensed under the GPL-2 or later.
 */
#ifndef _ASM_NIOS2_CACHE_H
#define _ASM_NIOS2_CACHE_H

#include <asm/nios.h>

/* bytes per L1 cache line */
#define L1_CACHE_BYTES		ICACHE_LINE_SIZE
#define L1_CACHE_SHIFT		ICACHE_LINE_SIZE_LOG2

#define ARCH_DMA_MINALIGN	L1_CACHE_BYTES

#define __cacheline_aligned
#define ____cacheline_aligned

#endif
