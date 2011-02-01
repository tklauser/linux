/*
 * Copyright (C) 2011 Tobias Klauser <tklauser@distanz.ch>
 *
 * Based on cpuinfo.c from microblaze
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file COPYING in the main directory of this
 * archive for more details.
 */

#define DEBUG

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/of.h>
#include <asm/cpuinfo.h>

struct cpuinfo cpuinfo;

void __init setup_cpuinfo(void)
{
	struct device_node *cpu;

	cpu = of_find_node_by_type(NULL, "cpu");
	if (!cpu)
		panic("%s: No CPU found in devicetree!\n", __func__);

	pr_debug("%s: CPU %s found\n", __func__, cpu->name);

	/* for now */
#ifdef CONFIG_MMU
	cpuinfo.mmu = 1;
#else
	cpuinfo.mmu = 0;
#endif

	cpuinfo.cpu_clock_freq = fcpu(cpu, "clock-frequency");

	cpuinfo.has_div = fcpu(cpu, "altr,has-div");
	cpuinfo.has_mul = fcpu(cpu, "altr,has-mul");
	cpuinfo.has_mulx = fcpu(cpu, "altr,has-mulx");

	cpuinfo.icache_line_size = fcpu(cpu, "altr,icache-line-size");
	cpuinfo.icache_size = fcpu(cpu, "altr,icache-size");
	cpuinfo.dcache_line_size = fcpu(cpu, "altr,dcache-line-size");
	cpuinfo.dcache_size = fcpu(cpu, "altr,dcache-size");
}
