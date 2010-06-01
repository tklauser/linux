/*
 * Nios2-specific parts of system setup
 *
 * Copyright (C) 2010 Tobias Klauser <tklauser@distanz.ch>
 * Copyright (C) 2004 Microtronix Datacom Ltd.
 * Copyright (C) 2001 Vic Phillips <vic@microtronix.com>
 *
 * based on kernel/setup.c from m68knommu
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/fb.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/genhd.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/major.h>
#include <linux/bootmem.h>
#include <linux/initrd.h>
#include <linux/seq_file.h>

#ifdef CONFIG_BLK_DEV_INITRD
#include <linux/blkdev.h>
#endif

#include <asm/irq.h>
#include <asm/byteorder.h>
#include <asm/asm-offsets.h>
#include <asm/pgtable.h>
#include <asm/setup.h>

/*
 * Sanity check config options for HW supported mul, mulx, div against
 * the generated header for the specified design.
 */
#if defined(CONFIG_NIOS2_HW_MUL_SUPPORT)
# if HARDWARE_MULTIPLY_PRESENT == 0
#  error Kernel compiled with MUL support although not enabled in design
# endif
#else
# if HARDWARE_MULTIPLY_PRESENT != 0
#  warning Kernel compiled without MUL support although enabled in design
# endif
#endif

#if defined(CONFIG_NIOS2_HW_MULX_SUPPORT)
# if HARDWARE_MULX_PRESENT == 0
#  error Kernel compiled with MULX support although not enabled in design
# endif
#else
# if HARDWARE_MULX_PRESENT != 0
#  warning Kernel compiled without MULX support although enabled in design
# endif
#endif

#if defined(CONFIG_NIOS2_HW_DIV_SUPPORT)
# if HARDWARE_DIVIDE_PRESENT == 0
#  error Kernel compiled with DIV support although not enabled in design
# endif
#else
# if HARDWARE_DIVIDE_PRESENT != 0
#  warning Kernel compiled without DIV support although enabled in design
# endif
#endif

#ifdef CONFIG_CONSOLE
extern struct consw *conswitchp;
#endif

unsigned long rom_length;
unsigned long memory_start;
unsigned long memory_end;

EXPORT_SYMBOL(memory_start);
EXPORT_SYMBOL(memory_end);

#ifndef CONFIG_PASS_CMDLINE
static char default_command_line[] = CONFIG_CMDLINE;
#endif
static char __initdata command_line[COMMAND_LINE_SIZE] = { 0, };


/*				   r1  r2  r3  r4  r5  r6  r7  r8  r9 r10 r11*/
/*				   r12 r13 r14 r15 or2                      ra  fp  sp  gp es  ste  ea*/
static struct pt_regs fake_regs = { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
				    0,  0,  0,  0,  0, (unsigned long)cpu_idle,  0,  0,  0, 0, 0};

extern void __init mmu_init(void);

/* save args passed from u-boot, called from head.S */
asmlinkage void __init nios2_boot_init(unsigned r4, unsigned r5, unsigned r6, unsigned r7)
{
	mmu_init();

#if defined(CONFIG_PASS_CMDLINE)
	if (r4 == 0x534f494e) {	/* r4 is magic NIOS, to become board info check in the future */
#if defined(CONFIG_BLK_DEV_INITRD)
		/*
		 * If the init RAM disk has been configured in, and there's a valid
		 * starting address for it, set it up.
		 */
		if (r5) {
			initrd_start = r5;
			initrd_end = r6;
		}
#endif /* CONFIG_BLK_DEV_INITRD */
		if (r7)
			strncpy(command_line, (char *)r7, COMMAND_LINE_SIZE);
	}
#endif
	if (!command_line[0])
		strncpy(command_line, CONFIG_CMDLINE, COMMAND_LINE_SIZE);
}

void __init setup_arch(char **cmdline_p)
{
	int bootmap_size;
	extern int _stext, _etext;
	extern int _edata, _end;

#ifdef CONFIG_EARLY_PRINTK
	extern void setup_early_printk(void);
	setup_early_printk();
#endif

	memory_start = PAGE_ALIGN((unsigned long)__pa(&_end));
	memory_end = (unsigned long) DDR2_TOP_BASE + DDR2_TOP_SPAN;

#ifndef CONFIG_PASS_CMDLINE
	memcpy(command_line, default_command_line, sizeof(default_command_line));
#endif

	printk("%s/Nios II\n", UTS_SYSNAME);

	init_mm.start_code = (unsigned long) &_stext;
	init_mm.end_code = (unsigned long) &_etext;
	init_mm.end_data = (unsigned long) &_edata;
	init_mm.brk = (unsigned long) &_end;
	init_task.thread.kregs = &fake_regs;

	/* Keep a copy of command line */
	*cmdline_p = &command_line[0];

	memcpy(boot_command_line, command_line, COMMAND_LINE_SIZE);
	boot_command_line[COMMAND_LINE_SIZE-1] = 0;

	/*
	 * give all the memory to the bootmap allocator,  tell it to put the
	 * boot mem_map at the start of memory
	 */
	printk("init_bootmem_node(?,%#lx, %#x, %#lx)\n",
	       PFN_UP(memory_start), PFN_DOWN(PHYS_OFFSET), PFN_DOWN(memory_end));
	bootmap_size = init_bootmem_node(NODE_DATA(0),
					 PFN_UP(memory_start),
					 PFN_DOWN(PHYS_OFFSET),
					 PFN_DOWN(memory_end));

	/*
	 * free the usable memory,  we have to make sure we do not free
	 * the bootmem bitmap so we then reserve it after freeing it :-)
	 */
	printk("free_bootmem(%#lx, %#lx)\n",
	       memory_start, memory_end - memory_start);
	free_bootmem(memory_start, memory_end - memory_start);

        /*
         * Reserve the bootmem bitmap itself as well. We do this in two
         * steps (first step was init_bootmem()) because this catches
         * the (very unlikely) case of us accidentally initializing the
         * bootmem allocator with an invalid RAM area.
	 *
	 * Arguments are start, size
         */
	printk("reserve_bootmem(%#lx, %#x)\n", memory_start, bootmap_size);
        reserve_bootmem(memory_start, bootmap_size, BOOTMEM_DEFAULT);

#ifdef CONFIG_BLK_DEV_INITRD
	if (initrd_start)
		reserve_bootmem(virt_to_phys((void *)initrd_start), initrd_end - initrd_start, BOOTMEM_DEFAULT);
#endif /* CONFIG_BLK_DEV_INITRD */

	/*
	 * get kmalloc into gear
	 */
	paging_init();
#ifdef CONFIG_VT
#if defined(CONFIG_DUMMY_CONSOLE)
	conswitchp = &dummy_con;
#endif
#endif
}

void arch_gettod(int *year, int *month, int *date, int *hour, int *min, int *sec)
{
	*year = *month = *date = *hour = *min = *sec = 0;
}

#ifdef CONFIG_PROC_FS

/*
 *	Get CPU information for use by the procfs.
 */
static int show_cpuinfo(struct seq_file *m, void *v)
{
	int count = 0;
	const char *cpu_impl = CPU_IMPLEMENTATION;
	const u_long clockfreq = CPU_FREQ;

	count = seq_printf(m,
			"CPU:\t\tNios II/%s\n"
#if defined(MMU_PRESENT) && defined(CONFIG_MMU)
			"MMU:\t\t%u-way TLB, %d entries\n"
#else
			"MMU:\t\tnone\n"
#endif
			"FPU:\t\tnone\n"
			"Clocking:\t%lu.%02lu MHz\n"
			"BogoMips:\t%lu.%02lu\n"
			"Calibration:\t%lu loops\n",
			cpu_impl,
#if defined(MMU_PRESENT) && defined(CONFIG_MMU)
			TLB_NUM_WAYS, TLB_NUM_ENTRIES,
#endif
			clockfreq / 1000000, (clockfreq / 100000) % 10,
			(loops_per_jiffy * HZ) / 500000, ((loops_per_jiffy * HZ) / 5000) % 100,
			(loops_per_jiffy * HZ));

	count += seq_printf(m,
			"HW:\n"
			" MUL:\t\t%s\n"
			" MULX:\t\t%s\n"
			" DIV:\t\t%s\n",
			HARDWARE_MULTIPLY_PRESENT ? "yes" : "no",
			HARDWARE_MULX_PRESENT ? "yes" : "no",
			HARDWARE_DIVIDE_PRESENT ? "yes" : "no");

	count += seq_printf(m,
			"Icache:\t\t%ukB, line length: %u\n",
			ICACHE_SIZE >> 10,
			ICACHE_LINE_SIZE);

	count += seq_printf(m,
			"Dcache:\t\t%ukB, line length: %u\n",
			DCACHE_SIZE >> 10,
			DCACHE_LINE_SIZE);

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
