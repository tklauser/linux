/*
 * Copyright (C) 2011 Tobias Klauser <tklauser@distanz.ch>
 *
 * Based on cpuinfo.c from microblaze
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file COPYING in the main directory of this
 * archive for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/of.h>
#include <asm/cpuinfo.h>

struct cpuinfo cpuinfo;

#ifdef CONFIG_OF

static inline u32 fcpu(struct device_node *cpu, char *n)
{
	u32 *val;
	return (val = (u32 *) of_get_property(cpu, n, NULL)) ? be32_to_cpup(val) : 0;
}

void __init setup_cpuinfo(void)
{
	struct device_node *cpu;

	cpu = of_find_node_by_type(NULL, "cpu");
	if (!cpu)
		panic("%s: No CPU found in devicetree!\n", __func__);

	/* for now */
#ifdef CONFIG_MMU
	cpuinfo.mmu = 1;
#else
	cpuinfo.mmu = 0;
#endif

	cpuinfo.cpu_clock_freq = fcpu(cpu, "clock-frequency");

	cpuinfo.cpu_impl = of_get_property(cpu, "altr,implementation", NULL);
	if (!cpuinfo.cpu_impl)
		cpuinfo.cpu_impl = "<unknown>";

	cpuinfo.has_div = fcpu(cpu, "altr,has-div");
	cpuinfo.has_mul = fcpu(cpu, "altr,has-mul");
	cpuinfo.has_mulx = fcpu(cpu, "altr,has-mulx");

	cpuinfo.icache_line_size = fcpu(cpu, "icache-line-size");
	cpuinfo.icache_size = fcpu(cpu, "icache-size");
	cpuinfo.dcache_line_size = fcpu(cpu, "dcache-line-size");
	cpuinfo.dcache_size = fcpu(cpu, "dcache-size");

	cpuinfo.pid_num_bits = fcpu(cpu, "altr,pid-num-bits");
	cpuinfo.tlb_num_ways = fcpu(cpu, "altr,tlb-num-ways");
	cpuinfo.tlb_num_entries = fcpu(cpu, "altr,tlb-num-entries");
}

#endif /* CONFIG_OF */

#ifdef CONFIG_PROC_FS

/*
 * Get CPU information for use by the procfs.
 */
static int show_cpuinfo(struct seq_file *m, void *v)
{
	int count = 0;
	const u32 clockfreq = cpuinfo.cpu_clock_freq;

	count = seq_printf(m,
			"CPU:\t\tNios II/%s\n"
			"MMU:\t\t%s\n"
			"FPU:\t\tnone\n"
			"Clocking:\t%u.%02u MHz\n"
			"BogoMips:\t%lu.%02lu\n"
			"Calibration:\t%lu loops\n",
			cpuinfo.cpu_impl,
			cpuinfo.mmu ? "present" : "none",
			clockfreq / 1000000, (clockfreq / 100000) % 10,
			(loops_per_jiffy * HZ) / 500000, ((loops_per_jiffy * HZ) / 5000) % 100,
			(loops_per_jiffy * HZ));

	count += seq_printf(m,
			"HW:\n"
			" MUL:\t\t%s\n"
			" MULX:\t\t%s\n"
			" DIV:\t\t%s\n",
			cpuinfo.has_mul ? "yes" : "no",
			cpuinfo.has_mulx ? "yes" : "no",
			cpuinfo.has_div ? "yes" : "no");

	count += seq_printf(m,
			"Icache:\t\t%ukB, line length: %u\n",
			cpuinfo.icache_size >> 10,
			cpuinfo.icache_line_size);

	count += seq_printf(m,
			"Dcache:\t\t%ukB, line length: %u\n",
			cpuinfo.dcache_size >> 10,
			cpuinfo.dcache_line_size);

	if (cpuinfo.mmu)
		count += seq_printf(m,
				"TLB:\t\t%u ways, %u entries\n",
				cpuinfo.tlb_num_ways,
				cpuinfo.tlb_num_entries);

	return 0;
}

static void *cpuinfo_start(struct seq_file *m, loff_t *pos)
{
	unsigned long i = *pos;

	return i < NR_CPUS ? (void *) (i + 1) : NULL;
}

static void *cpuinfo_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return cpuinfo_start(m, pos);
}

static void cpuinfo_stop(struct seq_file *m, void *v)
{
}

const struct seq_operations cpuinfo_op = {
	.start	= cpuinfo_start,
	.next	= cpuinfo_next,
	.stop	= cpuinfo_stop,
	.show	= show_cpuinfo
};

#endif /* CONFIG_PROC_FS */
