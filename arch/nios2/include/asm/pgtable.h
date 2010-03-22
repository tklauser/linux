/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * pgd = Page Global Directory
 * pud = Page Upped Directory
 * pmd = Page Middle Directory
 * pte = Page Table Entry
 *
 * Copyright (C) 2003 Ralf Baechle
 */
#ifndef _ASM_NIOS2_PGTABLE_H
#define _ASM_NIOS2_PGTABLE_H

#include <asm/io.h>
#include <asm/page.h>
#include <asm/addrspace.h>

#include <asm/pgtable-bits.h>
#include <asm-generic/pgtable-nopmd.h>

#define FIRST_USER_ADDRESS	0
#define VMALLOC_START	KERNEL_MMU_REGION_BASE
#define VMALLOC_END	   (KERNEL_REGION_BASE-1)

struct mm_struct;

/* Helper macro
 */ 
#define MKP(x,w,r) __pgprot(_PAGE_PRESENT|PAGE_CACHABLE_DEFAULT|\
			    ((x)?_PAGE_EXEC:0)|			 \
			    ((r)?_PAGE_READ:0)|			 \
			    ((w)?_PAGE_WRITE:0))
/*
 * These are the macros that generic kernel code needs 
 * (to populate protection_map[])
 */

/* Remove W bit on private pages for COW support
 */	    
#define __P000	MKP(0,0,0)
#define __P001	MKP(0,0,1)
#define __P010	MKP(0,0,0) /*COW*/
#define __P011	MKP(0,0,1) /*COW*/
#define __P100	MKP(1,0,0) 
#define __P101	MKP(1,0,1)
#define __P110	MKP(1,0,0) /*COW*/
#define __P111	MKP(1,0,1) /*COW*/

/* Shared pages can have exact HW mapping
 */ 
#define __S000	MKP(0,0,0)
#define __S001	MKP(0,0,1)
#define __S010	MKP(0,1,0) 
#define __S011	MKP(0,1,1)
#define __S100	MKP(1,0,0)
#define __S101	MKP(1,0,1)
#define __S110	MKP(1,1,0)
#define __S111	MKP(1,1,1)

/* Used all over the kernel
 */
#define PAGE_KERNEL __pgprot(_PAGE_PRESENT|PAGE_CACHABLE_DEFAULT|_PAGE_READ|_PAGE_WRITE|_PAGE_EXEC|_PAGE_GLOBAL)

/* ivho:PAGE_COPY only used by read_zero_pagealigned() in mem.c
 */
#define PAGE_COPY MKP(0,0,1)


#define PGD_ORDER	0
#define PUD_ORDER	aieeee_attempt_to_allocate_pud
#define PMD_ORDER	1
#define PTE_ORDER	0

#define PTRS_PER_PGD	((PAGE_SIZE << PGD_ORDER) / sizeof(pgd_t))
#define PTRS_PER_PTE	((PAGE_SIZE << PTE_ORDER) / sizeof(pte_t))

#define USER_PTRS_PER_PGD	(KERNEL_MMU_REGION_BASE/PGDIR_SIZE)

#define PGDIR_SHIFT	22
#define PGDIR_SIZE	(1UL << PGDIR_SHIFT)
#define PGDIR_MASK	(~(PGDIR_SIZE-1))

/* ivho: ?
 */ 
#define PTE_FILE_MAX_BITS       28

/* ivho: is vaddr always "unsigned long"? 
 */
struct page * ZERO_PAGE(unsigned long vaddr);


extern pgd_t swapper_pg_dir[PTRS_PER_PGD];
extern pte_t invalid_pte_table[PAGE_SIZE/sizeof(pte_t)];

/*
 * (pmds are folded into puds so this doesn't get actually called,
 * but the define is needed for a generic inline function.)
 */
void set_pmd(pmd_t *pmdptr, pmd_t pmdval);

extern void paging_init(void);

/* to find an entry in a pagetable-directory */
//#define pgd_offset(mm,addr)	((mm)->pgd + pgd_index(addr))
//ivho: checkme type of addr
pgd_t *pgd_offset(struct mm_struct *, unsigned long addr);

void pgtable_cache_init(void);

/* ivho: set back to "static inline" when correct in pgtable.c
 */
int pte_write(pte_t pte);
int pte_dirty(pte_t pte);
int pte_young(pte_t pte);
int pte_file(pte_t pte);
static inline int pte_special(pte_t pte)	{ return 0; }

#define pgprot_noncached pgprot_noncached

static inline pgprot_t pgprot_noncached(pgprot_t _prot)
{
	unsigned long prot = pgprot_val(_prot);

	prot &= ~_PAGE_CACHED;

	return __pgprot(prot);
}

#include <linux/swap.h>
swp_entry_t   __pte_to_swp_entry(pte_t pte);
pte_t         __swp_entry_to_pte(swp_entry_t swp);
unsigned long __swp_type(swp_entry_t);
pgoff_t       __swp_offset(swp_entry_t);
swp_entry_t   __swp_entry(unsigned long, pgoff_t offset);

int pte_none(pte_t pte);
int pte_present(pte_t pte);


/*
 * The following only work if pte_present() is true.
 * Undefined behaviour if not..
 */
pte_t pte_wrprotect(pte_t pte);
pte_t pte_mkclean(pte_t pte);
pte_t pte_mkold(pte_t pte);
pte_t pte_mkwrite(pte_t pte);
pte_t pte_mkdirty(pte_t pte);
static inline pte_t pte_mkspecial(pte_t pte)	{ return pte; }

pte_t pte_mkyoung(pte_t pte);
pte_t pte_modify(pte_t pte, pgprot_t newprot);

int   pmd_present(pmd_t pmd);
void  pmd_clear(pmd_t *pmdp);

/*
 * Certain architectures need to do special things when pte's
 * within a page table are directly modified.  Thus, the following
 * hook is made available.
 */
void set_pte(pte_t *ptep, pte_t pteval);
void set_pte_at(struct mm_struct *mm, unsigned long addr,pte_t *ptep, pte_t pteval);

//int pmd_none2 (pmd_t *pmd);
int pmd_none(pmd_t pmd);
int pmd_bad(pmd_t pmd);


void pte_clear(struct mm_struct *mm, unsigned long addr, pte_t *ptep);

/*
 * Conversion functions: convert a page and protection to a page entry,
 * and a page entry and page directory to the page they refer to.
 */
pte_t mk_pte(struct page *page, pgprot_t pgprot);

void update_mmu_cache(struct vm_area_struct *vma,
				    unsigned long address, pte_t *pte);

void pte_unmap(pte_t *pte);
void pte_unmap_nested(pte_t *pte);

pte_t pgoff_to_pte(pgoff_t off);

/*
 * Conversion functions: convert a page and protection to a page entry,
 * and a page entry and page directory to the page they refer to.
 */
#define pmd_phys(pmd)		virt_to_phys((void *)pmd_val(pmd))
#define pmd_page(pmd)		(pfn_to_page(pmd_phys(pmd) >> PAGE_SHIFT))
#define pmd_page_vaddr(pmd)	pmd_val(pmd)

//#define pud_none(pud) 0
//#define pmd_offset(pmd,off) 0

unsigned long pte_pfn(pte_t pte);
pte_t * pte_offset_map(pmd_t *dir, unsigned long address);
pte_t * pte_offset_map_nested(pmd_t *dir, unsigned long address);

/* to find an entry in a kernel page-table-directory */
pgd_t * pgd_offset_k(unsigned long address);

/* Get the address to the PTE for a vaddr in specfic directory 
 */
pte_t * pte_offset_kernel(pmd_t * dir, unsigned long address);

void pgd_ERROR(pgd_t e);


pte_t pfn_pte(unsigned long  pfn, pgprot_t prot);

pgoff_t pte_to_pgoff(pte_t pte);
struct page * pte_page(pte_t pte);

int kern_addr_valid(unsigned long addr);


#define io_remap_pfn_range(vma, vaddr, pfn, size, prot)	\
	remap_pfn_range(vma, vaddr, pfn, size, prot)

/*
 * We provide our own get_unmapped area to cope with the virtual aliasing
 * constraints placed on us by the cache architecture.
 */

#define HAVE_ARCH_UNMAPPED_AREA

#include <asm-generic/pgtable.h>

#endif /* _ASM_NIOS2_PGTABLE_H */
