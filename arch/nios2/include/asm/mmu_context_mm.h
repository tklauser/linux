/*
 * Copyright (C) 2010 Tobias Klauser <tklauser@distanz.ch>
 * Copyright (C) 1996, 1997, 1998, 1999 by Ralf Baechle
 * Copyright (C) 1999 Silicon Graphics, Inc.
 *
 * based on MIPS asm/mmu_context.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#ifndef _ASM_NIOS2_MMU_CONTEXT_H
#define _ASM_NIOS2_MMU_CONTEXT_H

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm-generic/mm_hooks.h>

/*
 * For the fast tlb miss handlers, we keep a pointer to the current pgd.
 * processor.
 */
extern unsigned long pgd_current;

void enter_lazy_tlb(struct mm_struct *mm, struct task_struct *tsk);

/* ivho: leaving old MIPS comment
 * Initialize the context related info for a new mm_struct
 * instance.
 */
int init_new_context(struct task_struct *tsk, struct mm_struct *mm);

/* Normal, classic MIPS get_new_mmu_context */
void get_new_mmu_context(struct mm_struct *mm, unsigned long cpu);

/* ivho: leaving old MIPS comment
 * Destroy context related info for an mm_struct that is about
 * to be put to rest.
 */
void destroy_context(struct mm_struct *mm);

void switch_mm(struct mm_struct *prev, struct mm_struct *next,
	       struct task_struct *tsk);

void deactivate_mm(struct task_struct *tsk, struct mm_struct *mm);

/* ivho: leaving old MIPS comment
 * After we have set current->mm to a new value, this activates
 * the context for the new mm so we see the new mappings.
 */
void activate_mm(struct mm_struct *prev, struct mm_struct *next);

#endif /* _ASM_NIOS2_MMU_CONTEXT_H */
