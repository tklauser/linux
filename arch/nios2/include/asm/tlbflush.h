/*
 * Copyright (C) 2010 Tobias Klauser <tklauser@distanz.ch>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#ifndef _ASM_NIOS2_TLBFLUSH_H
#define _ASM_NIOS2_TLBFLUSH_H

#ifndef CONFIG_MMU
# include <asm-generic/tlbflush.h>
#else

#include <linux/mm.h>

/*
 * TLB flushing:
 *
 *  - flush_tlb_all() flushes all processes TLB entries
 *  - flush_tlb_mm(mm) flushes the specified mm context TLB entries
 *  - flush_tlb_page(vma, vmaddr) flushes one page
 *  - flush_tlb_range(vma, start, end) flushes a range of pages
 *  - flush_tlb_kernel_range(start, end) flushes a range of kernel pages
 */
extern void local_flush_tlb_all(void);
extern void local_flush_tlb_mm(struct mm_struct *mm);
extern void local_flush_tlb_range(struct vm_area_struct *vma,
				  unsigned long start, unsigned long end);
extern void local_flush_tlb_kernel_range(unsigned long start,
					 unsigned long end);
extern void local_flush_tlb_page(struct vm_area_struct *vma,
				 unsigned long address);
extern void local_flush_tlb_one(unsigned long vaddr);

#define flush_tlb_all		local_flush_tlb_all
#define flush_tlb_mm		local_flush_tlb_mm
#define flush_tlb_range		local_flush_tlb_range
#define flush_tlb_kernel_range	local_flush_tlb_kernel_range
#define flush_tlb_page		local_flush_tlb_page
#define flush_tlb_one		local_flush_tlb_one

#endif /* CONFIG_MMU */

#endif /* _ASM_NIOS2_TLBFLUSH_H */
