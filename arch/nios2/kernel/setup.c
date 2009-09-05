/*
  21Mar2001    1.1    dgt/microtronix: Altera Excalibur/Nios32 port
  30Jun2003           kenw/microtronix: Remove cmdline check in flash
*/

/*
 *  linux/arch/niosnommu/kernel/setup.c
 *
 *  Copyright (C) 2004       Microtronix Datacom Ltd.
 *  Copyright (C) 2001       Vic Phillips {vic@microtronix.com}
 *  Copyleft  (C) 2000       James D. Schettine {james@telos-systems.com}
 *  Copyright (C) 1999       Greg Ungerer (gerg@moreton.com.au)
 *  Copyright (C) 1998,2000  D. Jeff Dionne <jeff@lineo.ca>
 *  Copyright (C) 1998       Kenneth Albanowski <kjahds@kjahds.com>
 *  Copyright (C) 1995       Hamish Macdonald
 *
 * All rights reserved.          
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 * NON INFRINGEMENT.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/*
 * This file handles the architecture-dependent parts of system setup
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

/* Sanity check config options for HW supported mul, mulx, div against
 * the generated header for the specified design.
 */
#if defined(CONFIG_NIOS2_HW_MUL_SUPPORT)
#if HARDWARE_MULTIPLY_PRESENT == 0
#error Kernel compiled with mul support although not enabled in design
#endif
#else
#if HARDWARE_MULTIPLY_PRESENT != 0
#warning Kernel compiled without mul support although enabled in design
#endif
#endif

#if defined(CONFIG_NIOS2_HW_MULX_SUPPORT)
#if HARDWARE_MULX_PRESENT == 0
#error Kernel compiled with mulx support although not enabled in design
#endif
#else
#if HARDWARE_MULX_PRESENT != 0
#warning Kernel compiled without mulx support although enabled in design
#endif
#endif

#if defined(CONFIG_NIOS2_HW_DIV_SUPPORT)
#if HARDWARE_DIVIDE_PRESENT == 0
#error Kernel compiled with div support although not enabled in design
#endif
#else
#if HARDWARE_DIVIDE_PRESENT != 0
#warning Kernel compiled without div support although enabled in design
#endif
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
static struct pt_regs fake_regs = { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,\
				    0,  0,  0,  0,  0, (unsigned long)cpu_idle,  0,  0,  0, 0, 0};

#define CPU "NIOS2"

extern void initMMU(void);

// save args passed from u-boot, called from head.S
void __init nios2_boot_init(unsigned r4,unsigned r5,unsigned r6,unsigned r7)
{
  initMMU();

#if defined(CONFIG_PASS_CMDLINE)
  if (r4 == 0x534f494e)   // r4 is magic NIOS, to become board info check in the future
    {
#if defined(CONFIG_BLK_DEV_INITRD)
	/*
	 * If the init RAM disk has been configured in, and there's a valid
	 * starting address for it, set it up.
	 */
	if (r5) {
		initrd_start = r5;
		initrd_end = r6;
	}
#endif				/* CONFIG_BLK_DEV_INITRD */
	if (r7)
		strncpy(command_line, (char *)r7, COMMAND_LINE_SIZE);
    }
#endif
  if (!command_line[0]) {
    strncpy(command_line, CONFIG_CMDLINE, COMMAND_LINE_SIZE);
  }
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

	printk("\r\n\nLinux/Nios II-MMU\n");


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
	if (initrd_start) {
	  reserve_bootmem(virt_to_phys((void *)initrd_start), initrd_end - initrd_start, BOOTMEM_DEFAULT);
	}
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

/*
 *	Get CPU information for use by the procfs.
 */

static int show_cpuinfo(struct seq_file *m, void *v)
{
    char *cpu, *fpu;

    cpu = CPU;
    fpu = "none";

#if 0
    /* FIXME: nasys_clock_freq is no longer availble in the new 
     * generated headers for the design. How do we know
     * which frequency the CPU is running at?
     */
    u_long clockfreq;
    clockfreq = nasys_clock_freq;
    MHz=clockfreq/1000000,(clockfreq/100000)%10
#endif

    seq_printf(m, "CPU:\t\t%s\n"
		   "MMU:\t\tways:%d entries:%d\n"
		   "FPU:\t\t%s\n"
		   "Clocking:\t<not supported>\n"
		   "BogoMips:\t%lu.%02lu\n"
		   "Calibration:\t%lu loops\n",
		   cpu, 
	           TLB_NUM_WAYS, TLB_NUM_ENTRIES,
	           fpu,	          
		   (loops_per_jiffy*HZ)/500000,((loops_per_jiffy*HZ)/5000)%100,
		   (loops_per_jiffy*HZ));

	return 0;
}


void arch_gettod(int *year, int *month, int *date, int *hour, int *min, int *sec)
{
	*year = *month = *date = *hour = *min = *sec = 0;
}

static void *cpuinfo_start (struct seq_file *m, loff_t *pos)
{
	return *pos < NR_CPUS ? ((void *) 0x12345678) : NULL;
}

static void *cpuinfo_next (struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return cpuinfo_start (m, pos);
}

static void cpuinfo_stop (struct seq_file *m, void *v)
{
}

const struct seq_operations cpuinfo_op = {
	.start	= cpuinfo_start,
	.next	= cpuinfo_next,
	.stop	= cpuinfo_stop,
	.show	= show_cpuinfo
};

