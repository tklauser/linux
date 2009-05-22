#ifndef _ASM_NIOS2_TLB_H
#define _ASM_NIOS2_TLB_H

/*
 * NiosII doesn't need any special per-pte or per-vma handling, except
 * we need to flush cache for area to be unmapped.
 */
#define tlb_start_vma(tlb, vma) 				\
	do {							\
		if (!tlb->fullmm)				\
			flush_cache_range(vma, vma->vm_start, vma->vm_end); \
	}  while (0)
#define tlb_end_vma(tlb, vma) do { } while (0)
#define __tlb_remove_tlb_entry(tlb, ptep, address) do { } while (0)

void tlb_flush(struct mmu_gather *tlb);
void local_tlb_flush_all(void);
void initMMU(void);

#include <linux/pagemap.h>
#include <asm-generic/tlb.h>

#endif /* _ASM_NIOS2_TLB_H */
