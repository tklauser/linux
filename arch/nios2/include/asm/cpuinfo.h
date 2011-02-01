/*
 * Copyright (C) 2011 Tobias Klauser <tklauser@distanz.ch>
 *
 * Based on asm/cpuinfo.h from microblaze
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file COPYING in the main directory of this
 * archive for more details.
 */

#ifndef _ASM_NIOS2_CPUINFO_H
#define _ASM_NIOS2_CPUINFO_H

#include <asm/prom.h>

struct cpuinfo {
	/* Core CPU configuration */
	u32 cpu_clock_freq;
	u32 mmu;
	u32 has_div;
	u32 has_mul;
	u32 has_mulx;

	/* CPU caches */
	u32 icache_line_size;
	u32 icache_size;
	u32 dcache_line_size;
	u32 dcache_size;

	/* TLB */
	u32 tlb_num_ways;
	u32 tlb_num_entries;
};

extern struct cpuinfo cpuinfo;

extern void setup_cpuinfo(void);

static inline u32 fcpu(struct device_node *cpu, char *n)
{
	u32 *val;
	return (val = (u32 *) of_get_property(cpu, n, NULL)) ? be32_to_cpup(val) : 0;
}

#endif /* _ASM_NIOS2_CPUINFO_H */
