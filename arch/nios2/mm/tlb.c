/*
 * Nios2 TLB handling
 *
 * Copyright (C) 2009, Wind River Systems Inc
 *   Implemented by fredrik.markstrom@gmail.com and ivarholmqvist@gmail.com
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/pagemap.h>

#include <asm/tlb.h>
#include <asm/mmu_context.h>
#include <asm/pgtable.h>
#include <asm/system.h>

#include <asm/tlbstats.h>
#include <asm/cpuinfo.h>

#define TLB_INDEX_MASK ((((1UL << (cpuinfo.tlb_ptr_sz - cpuinfo.tlb_num_ways_log2))) - 1) << PAGE_SHIFT)

struct tlb_stat statistics;

/* Used as illegal PHYS_ADDR for TLB mappings
 */
/* FIXME: ((1UL << DATA_ADDR_WIDTH) - 1)
 */
#define MAX_PHYS_ADDR 0

/* bit definitions for TLBMISC register */
#define PID_SHIFT     4
#define PID_MASK 0x3fff

#define WAY_SHIFT 20
#define WAY_MASK  0xf

/* Allow access from user space to special virtual address
 * for debugging purposes. TODO: This should probably go away.
 */
#define NIOS_DEBUG_REG 1

#if NIOS_DEBUG_REG
#define DEBUG_ADDR	0xdffff000
#define KD_CURRENT	0
#define KD_PGD		1
#define KD_MMU		2
#define KD_TRACEON	3
#define KD_TRACEOFF	4
#define KD_DEBUG	5
#define KD_PT_ESTATUS   6
#define KD_START_STAT	7

#define nios_iss_set_trace(d)

static void start_new_stats(struct tlb_stat *stat)
{
	printk("tlb.c: stats->local_flush_tlb_mm = %d\r\n",
			stat->local_flush_tlb_mm);
	printk("tlb.c: stats->local_flush_tlb_one_pid = %d\r\n",
			stat->local_flush_tlb_one_pid);
	printk("tlb.c: stats->local_flush_tlb_range = %d\r\n",
			stat->local_flush_tlb_range);
	printk("tlb.c: stats->local_flush_tlb_page = %d\r\n",
			stat->local_flush_tlb_page);
	printk("tlb.c: stats->local_flush_tlb_kernel_range = %d\r\n",
			stat->local_flush_tlb_kernel_range);
	printk("tlb.c: stats->local_flush_tlb_one = %d\r\n",
			stat->local_flush_tlb_one);
	printk("tlb.c: stats->flush_tlb_pid = %d\r\n",
			stat->flush_tlb_pid);
	printk("tlb.c: stats->local_flush_tlb_all = %d\r\n",
			stat->local_flush_tlb_all);
	printk("tlb.c: stats->tlb_c_handler = %d\r\n",
			stat->tlb_c_handler);
	printk("tlb.c: stats->tlb_fast_handler = %d\r\n",
			stat->tlb_fast_handler);
	printk("tlb.c: stats->do_page_fault_prot = %d\r\n",
			stat->do_page_fault_prot);
	printk("tlb.c: stats->do_page_fault = %d\r\n",
			stat->do_page_fault);
	memset(stat, 0, sizeof(*stat));
}

static void nios_debug(struct pt_regs *pt,  unsigned long addr)
{
	int reg = addr & ~PAGE_MASK;

	printk("debug detected %#lx (reg=%d)\n", addr, reg);

	switch (reg) {
	case KD_CURRENT:
		printk("current->comm <%s>\n",current->comm);
		break;
	case KD_TRACEON:
		nios_iss_set_trace(1);
		break;
	case KD_TRACEOFF:
		nios_iss_set_trace(0);
		break;
	case KD_PT_ESTATUS:
		printk("estatus=%#lx\n", pt->estatus);
		break;
	case KD_START_STAT:
		start_new_stats(&statistics);
		break;
	default:
		printk("unknown reg\n");
		break;
	}
}
#endif /* NIOS_DEBUG_REG */

/*
 * All entries common to a mm share an asid.  To effectively flush these
 * entries, we just bump the asid.
 */
void local_flush_tlb_mm(struct mm_struct *mm)
{
	statistics.local_flush_tlb_mm++;

	if (current->mm == mm)
		local_flush_tlb_all();
	else
		memset(&mm->context, 0, sizeof(mm_context_t));
}

/*
 * This one is only used for pages with the global bit set so we don't care
 * much about the ASID.
 *
 * FIXME: This is proken, to prove it mmap a read/write-page, mprotect it to
 *        read only then write to it, the tlb-entry will not be flushed.
 *
 */
void local_flush_tlb_one_pid(unsigned long addr, unsigned long mmu_pid)
{
	unsigned int way;
	unsigned long org_misc;

	statistics.local_flush_tlb_one_pid++;

	pr_debug("Flush tlb-entry for vaddr=%#lx\n", addr);

	/* remember pid/way until we return.
	 * CHECKME: is there a race here when writing org_misc back? */
	org_misc = (RDCTL(CTL_TLBMISC) & ((PID_MASK << PID_SHIFT) | (WAY_MASK << WAY_SHIFT)));

	WRCTL(CTL_PTEADDR, (addr >> PAGE_SHIFT) << 2);

	for (way = 0; way < cpuinfo.tlb_num_ways; way++) {
		unsigned long pteaddr;
		unsigned long tlbmisc;
		unsigned long pid;

		WRCTL(CTL_TLBMISC, (1UL << 19) | (way << 20));
		pteaddr = RDCTL(CTL_PTEADDR);
		tlbmisc = RDCTL(CTL_TLBMISC);
		pid = (tlbmisc >> PID_SHIFT) & PID_MASK;
		if (((((pteaddr >> 2) & 0xfffff)) == (addr >> PAGE_SHIFT)) &&
				pid == mmu_pid) {
			unsigned long vaddr = CONFIG_IO_REGION_BASE + (PAGE_SIZE * cpuinfo.tlb_num_lines) * way + (addr & TLB_INDEX_MASK);
			pr_debug("Flush entry by writing %#lx way=%dl pid=%ld\n", vaddr, way, pid);

			WRCTL(CTL_PTEADDR, (vaddr >> 12) << 2);
			WRCTL(CTL_TLBMISC, (1UL << 18) | (way << 20) | pid << PID_SHIFT);
			WRCTL(CTL_TLBACC, (MAX_PHYS_ADDR >> PAGE_SHIFT));
		}
	}

	WRCTL(CTL_TLBMISC, org_misc);
}

void local_flush_tlb_range(struct vm_area_struct *vma, unsigned long start,
			   unsigned long end)
{
	/* Implemented in mmu_context.c */
	extern unsigned long get_pid_from_context(mm_context_t* ctx);
	unsigned long mmu_pid = get_pid_from_context(&vma->vm_mm->context);

	statistics.local_flush_tlb_range++;

	while (start < end) {
		local_flush_tlb_one_pid(start, mmu_pid);
		start += PAGE_SIZE;
	}
}

void local_flush_tlb_page(struct vm_area_struct *vma, unsigned long address)
{
	statistics.local_flush_tlb_page++;
	local_flush_tlb_one(address);
}

void local_flush_tlb_kernel_range(unsigned long start, unsigned long end)
{
	statistics.local_flush_tlb_kernel_range++;
	while (start < end) {
		local_flush_tlb_one(start);
		start += PAGE_SIZE;
	}
}

/*
 * This one is only used for pages with the global bit set so we don't care
 * much about the ASID.
 */
void local_flush_tlb_one(unsigned long addr)
{
	unsigned int way;
	unsigned long pid, org_misc;

	statistics.local_flush_tlb_one++;
	pr_debug("Flush tlb-entry for vaddr=%#lx\n", addr);

	/* remember pid/way until we return.
	* CHECKME: is there a race here when writing org_misc back? */
	org_misc = (RDCTL(CTL_TLBMISC) & ((PID_MASK << PID_SHIFT) | (WAY_MASK << WAY_SHIFT)));

	WRCTL(CTL_PTEADDR, (addr >> PAGE_SHIFT) << 2);

	for (way = 0; way < cpuinfo.tlb_num_ways; way++) {
		unsigned long pteaddr;
		unsigned long tlbmisc;

		WRCTL(CTL_TLBMISC, (1UL << 19) | (way << 20));
		pteaddr = RDCTL(CTL_PTEADDR);
		tlbmisc = RDCTL(CTL_TLBMISC);

		if ((((pteaddr >> 2) & 0xfffff)) == (addr >> PAGE_SHIFT)) {
			unsigned long vaddr = CONFIG_IO_REGION_BASE + (PAGE_SIZE * cpuinfo.tlb_num_lines) * way + (addr & TLB_INDEX_MASK);

			pid = (tlbmisc >> PID_SHIFT) & PID_MASK;
			pr_debug("Flush entry by writing %#lx way=%dl pid=%ld\n", vaddr, way, pid);

			WRCTL(CTL_PTEADDR, (vaddr >> 12) << 2);
			WRCTL(CTL_TLBMISC, (1UL << 18) | (way << 20) | pid << PID_SHIFT);
			WRCTL(CTL_TLBACC, (MAX_PHYS_ADDR >> PAGE_SHIFT));
		}
	}

	WRCTL(CTL_TLBMISC, org_misc);
}

void dump_tlb_line(unsigned long line)
{
	unsigned int way;
	unsigned long org_misc;

	printk("dump tlb-entries for line=%#lx (addr %08lx)\n", line, line << (PAGE_SHIFT + cpuinfo.tlb_num_ways_log2));

	/* remember pid/way until we return */
	org_misc = (RDCTL(CTL_TLBMISC) & ((PID_MASK << PID_SHIFT) | (WAY_MASK << WAY_SHIFT)));

	WRCTL(CTL_PTEADDR, line << 2);

	for (way = 0; way < cpuinfo.tlb_num_ways; way++) {
		unsigned long pteaddr;
		unsigned long tlbmisc;
		unsigned long tlbacc;

		WRCTL(CTL_TLBMISC, (1UL << 19) | (way << 20));
		pteaddr = RDCTL(CTL_PTEADDR);
		tlbmisc = RDCTL(CTL_TLBMISC);
		tlbacc = RDCTL(CTL_TLBACC);

		if ((tlbacc << PAGE_SHIFT) != (MAX_PHYS_ADDR & PAGE_MASK)) {
			printk("-- way:%02x vpn:0x%08lx phys:0x%08lx pid:0x%02lx flags:%c%c%c%c%c\n",
					way,
					(pteaddr<<(PAGE_SHIFT-2)),
					(tlbacc<<PAGE_SHIFT),
					(tlbmisc>>PID_SHIFT)&PID_MASK,
					((tlbacc>>20)&_PAGE_READ?'r':'-'),
					((tlbacc>>20)&_PAGE_WRITE?'w':'-'),
					((tlbacc>>20)&_PAGE_EXEC?'x':'-'),
					((tlbacc>>20)&_PAGE_GLOBAL?'g':'-'),
					((tlbacc>>20)&_PAGE_CACHED?'c':'-'));
		}
	}

	WRCTL(CTL_TLBMISC, org_misc);
}

void dump_tlb(void)
{
	unsigned int i;
	for (i = 0; i < cpuinfo.tlb_num_lines; i++)
		dump_tlb_line(i);
}

void flush_tlb_pid(unsigned long pid)
{
	unsigned int line;
	unsigned int way;
	unsigned long org_misc;

	statistics.flush_tlb_pid++;

	/* remember pid/way until we return */
	org_misc = (RDCTL(CTL_TLBMISC) & ((PID_MASK << PID_SHIFT) | (WAY_MASK << WAY_SHIFT)));

	for (line = 0; line < cpuinfo.tlb_num_lines; line++) {
		/* FIXME: << TLB_WAY_BITS should probably not be here */
		WRCTL(CTL_PTEADDR, line << 2);

		for (way = 0; way < cpuinfo.tlb_num_ways; way++) {
			unsigned long pteaddr;
			unsigned long tlbmisc;
			unsigned long tlbacc;

			WRCTL(CTL_TLBMISC, (1UL << 19) | (way << 20));
			pteaddr = RDCTL(CTL_PTEADDR);
			tlbmisc = RDCTL(CTL_TLBMISC);
			tlbacc = RDCTL(CTL_TLBACC);

			if (((tlbmisc>>PID_SHIFT)&PID_MASK) == pid) {
				WRCTL(CTL_TLBMISC, (1UL << 18) | (way << 20) | pid << PID_SHIFT);
				WRCTL(CTL_TLBACC, (MAX_PHYS_ADDR >> PAGE_SHIFT));
			}
		}

		WRCTL(CTL_TLBMISC, org_misc);
	}
}

void local_flush_tlb_all(void)
{
	int i;
	unsigned long vaddr = CONFIG_IO_REGION_BASE;
	unsigned int way;
	unsigned long org_misc;

	statistics.local_flush_tlb_all++;

	/* remember pid/way */
	org_misc = (RDCTL(CTL_TLBMISC) & ((PID_MASK << PID_SHIFT) | (WAY_MASK << WAY_SHIFT)));

	/* Map each TLB entry to physcal address 0 with no-access and a bad ptbase */
	for (way = 0; way < cpuinfo.tlb_num_ways; way++) {
		for (i = 0; i < cpuinfo.tlb_num_lines; i++) {
			WRCTL(CTL_PTEADDR, ((vaddr) >> PAGE_SHIFT) << 2);
			WRCTL(CTL_TLBMISC, (1<<18) | (way << 20));
			WRCTL(CTL_TLBACC, (MAX_PHYS_ADDR >> PAGE_SHIFT));
			vaddr += 1UL << 12;
		}
	}

	/* restore pid/way */
	WRCTL(CTL_TLBMISC, org_misc);
}

static int maybe_debug_register_access(struct pt_regs* regs, unsigned long addr)
{
	if (DEBUG_ADDR == (addr & PAGE_MASK)) {
		nios_debug(regs, addr);
		return 1;
	}
	return 0;
}

extern asmlinkage void do_page_fault(struct pt_regs *regs, unsigned long write,
                                     unsigned long address);

/* FIXME: This function should probably be eliminated, we
 *        should do this directly in fault.c
 */
void protection_exception_c(struct pt_regs *regs, int cause,
                            unsigned long addr)
{
	cause /= 4;

	/* Restart the instruction */
	regs->ea -= 4;

	/* Handle debug addresses */
	if (maybe_debug_register_access(regs, addr)) {
		/* Don't restart the instruction. */
		regs->ea += 4;
	} else {
		do_page_fault(regs, cause, addr);
	}
}

void set_mmu_pid(unsigned long pid)
{
	WRCTL(CTL_TLBMISC, (RDCTL(CTL_TLBMISC) & (WAY_MASK << WAY_SHIFT)) | ((pid & PID_MASK) << PID_SHIFT));
}
