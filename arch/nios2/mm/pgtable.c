/*
 * Copyright (C) 2009 Wind River Systems Inc
 *   Implemented by fredrik.markstrom@gmail.com and ivarholmqvist@gmail.com
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/init.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/highmem.h>
#include <asm/fixmap.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <asm/tlbflush.h>

/* pteaddr: 
 *   ptbase | vpn* | zero
 *   31-22  | 21-2 | 1-0
 *   
 *   *vpn is preserved on double fault
 *   
 * tlbacc:
 *   IG   |*flags| pfn
 *   31-25|24-20 | 19-0
 *   
 *   *crwxg
 *   
 * tlbmisc:
 *   resv  |way   |rd | we|pid |dbl|bad|perm|d
 *   31-24 |23-20 |19 | 20|17-4|3  |2  |1   |0
 *  
 */

/*
 * Initialize a new pgd / pmd table with invalid pointers.
 */
static void pgd_init(unsigned long page)
{
	unsigned long *p = (unsigned long *) page;
	int i;

	for (i = 0; i < USER_PTRS_PER_PGD; i += 8) {
		p[i + 0] = (unsigned long) invalid_pte_table;
		p[i + 1] = (unsigned long) invalid_pte_table;
		p[i + 2] = (unsigned long) invalid_pte_table;
		p[i + 3] = (unsigned long) invalid_pte_table;
		p[i + 4] = (unsigned long) invalid_pte_table;
		p[i + 5] = (unsigned long) invalid_pte_table;
		p[i + 6] = (unsigned long) invalid_pte_table;
		p[i + 7] = (unsigned long) invalid_pte_table;
	}
}

pgd_t *pgd_alloc(struct mm_struct *mm)
{
	pgd_t *ret, *init;
	ret = (pgd_t *) __get_free_pages(GFP_KERNEL, PGD_ORDER);
	if (ret) {
		init = pgd_offset(&init_mm, 0UL);
		pgd_init((unsigned long) ret);
		memcpy(ret + USER_PTRS_PER_PGD, init + USER_PTRS_PER_PGD,
		       (PTRS_PER_PGD - USER_PTRS_PER_PGD) * sizeof(pgd_t));
	}

	return ret;
}

void __init pagetable_init(void)
{
	pgd_t *pgd_base;
#ifdef CONFIG_HIGHMEM
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;
#endif
	pr_debug("check impl\n");

	/* Initialize the entire pgd.  */
	pgd_init((unsigned long)swapper_pg_dir);
	pgd_init((unsigned long)swapper_pg_dir
		 + sizeof(pgd_t) * USER_PTRS_PER_PGD);

	pgd_base = swapper_pg_dir;

#ifdef CONFIG_HIGHMEM
	/*
	 * Permanent kmaps:
	 */
	vaddr = PKMAP_BASE;
	fixrange_init(vaddr, vaddr + PAGE_SIZE*LAST_PKMAP, pgd_base);

	pgd = swapper_pg_dir + __pgd_offset(vaddr);
	pud = pud_offset(pgd, vaddr);
	pmd = pmd_offset(pud, vaddr);
	pte = pte_offset_kernel(pmd, vaddr);
	pkmap_page_table = pte;
#endif
}

/* FIXME: Avoiding cache alias for the zero-page is kind of stupid
 *        but we need to do it to avoid alias-warnings in the iss.
 */
extern unsigned long empty_zero_page;
struct page * ZERO_PAGE(unsigned long vaddr)
{
  struct page* page;
  unsigned long poffset;

  poffset = vaddr & ((DCACHE_SIZE - 1) & ~(PAGE_SIZE - 1));

  page = virt_to_page(empty_zero_page + poffset);

  return page;
}

/*
 * (pmds are folded into puds so this doesn't get actually called,
 * but the define is needed for a generic inline function.)
 */
void set_pmd(pmd_t *pmdptr, pmd_t pmdval)
{
  pmdptr->pud.pgd.pgd = pmdval.pud.pgd.pgd;
}

pgd_t *pgd_offset(struct mm_struct *mm, unsigned long addr)
{
  return mm->pgd + pgd_index(addr);
}

void pgtable_cache_init(void)
{
}

/* ivho: set back to "static inline" when correct in pgtable.c
 */
int pte_write(pte_t pte){
  return (pte_val(pte) & (_PAGE_WRITE << 20)) != 0;
}
int pte_dirty(pte_t pte){
  return (pte_val(pte) & (_PAGE_MODIFIED << 20)) != 0;

}
int pte_young(pte_t pte){
  return (pte_val(pte) & (_PAGE_ACCESSED << 20)) != 0;
}
int pte_file(pte_t pte){BUG();}

/* Swap not implemented
 */
swp_entry_t   __pte_to_swp_entry(pte_t pte){BUG();}
pte_t         __swp_entry_to_pte(swp_entry_t swp){BUG();}
unsigned long __swp_type(swp_entry_t swp){BUG();}
pgoff_t       __swp_offset(swp_entry_t swp){BUG();}
swp_entry_t   __swp_entry(unsigned long type, pgoff_t offset){BUG();}

/* FIXME: Today unmapped pages are mapped to the low physical addresses
 * and not 0 (to avoid to trigger the false alias detection in the iss)
 * Also check pte_clear.
 */
int pte_none(pte_t pte){
#if 0
   return (!((pte_val(pte) >> 20) & ~_PAGE_GLOBAL));
#else
   return (!((pte_val(pte) >> 20) & ~(_PAGE_GLOBAL|0xf)));
#endif
}

int pte_present(pte_t pte){
  return ((pte_val(pte) >> 20) & _PAGE_PRESENT) != 0;
}

/*
 * The following only work if pte_present() is true.
 * Undefined behaviour if not..
 */
pte_t pte_wrprotect(pte_t pte)
{
  pte_t p;
  pte_val(p) = pte_val(pte) & ~(_PAGE_WRITE<<20);
  return p;
}
pte_t pte_mkclean(pte_t pte){
   return pte;
}

pte_t pte_mkold(pte_t pte)
{
  pte_val(pte) |= (_PAGE_OLD << 20);
  return pte;
}

pte_t pte_mkwrite(pte_t pte){ 
  pte_val(pte) |= (_PAGE_WRITE << 20);
  return pte;
}

pte_t pte_mkdirty(pte_t pte){
   pte_val(pte) |= (_PAGE_MODIFIED << 20);
  return pte;
}

pte_t pte_mkyoung(pte_t pte){ 
   pte_val(pte) |= (_PAGE_ACCESSED << 20);
   return pte;
}

pte_t pte_modify(pte_t pte, pgprot_t newprot){
	const unsigned long mask = (_PAGE_READ | _PAGE_WRITE | _PAGE_EXEC) << 20;
	pte_val(pte) = (pte_val(pte) & ~mask) | ((pgprot_val(newprot) << 20) & mask);
	return pte;
}


int   pmd_present(pmd_t pmd) {
  return (pmd_val(pmd) !=  (unsigned long) invalid_pte_table) && (pmd_val(pmd) != 0UL);
}

void  pmd_clear(pmd_t *pmdp) {
  pmd_val(*pmdp) = (unsigned long)invalid_pte_table;
}

/*
 * Certain architectures need to do special things when pte's
 * within a page table are directly modified.  Thus, the following
 * hook is made available.
 */
void set_pte(pte_t *ptep, pte_t pteval){
  *ptep = pteval;
}

void set_pte_at(struct mm_struct *mm, unsigned long addr,
		pte_t *ptep, pte_t pteval)
{
   unsigned long paddr = page_to_virt(pte_page(pteval));
   flush_dcache_range(paddr, paddr + PAGE_SIZE);
   set_pte(ptep, pteval);
}

int pmd_none(pmd_t pmd){
  int rv;

  rv  = (pmd_val(pmd) == (unsigned long)invalid_pte_table);
  rv |= (pmd_val(pmd) == 0UL);
  pr_debug("pmd_none returns %#x (pmd=%#lx)\n", rv, pmd_val(pmd));
  return rv;
}

int pmd_bad(pmd_t pmd){
  pr_debug ("pmd_bad returns %#lx\n", pmd_val(pmd) & ~PAGE_MASK);
  return pmd_val(pmd) & ~PAGE_MASK;
}


void pte_clear(struct mm_struct *mm, unsigned long addr, pte_t *ptep){
  pte_t null;
  /* FIXME: check FIXME at pte_none
   */
#if 0
  pte_val(null) = 0;
#else
  pte_val(null) = (addr >> PAGE_SHIFT) & 0xf;
#endif
  set_pte_at(mm, addr, ptep, null);
  flush_tlb_one(addr);
}

/*
 * Conversion functions: convert a page and protection to a page entry,
 * and a page entry and page directory to the page they refer to.
 */
pte_t mk_pte(struct page *page, pgprot_t pgprot){
  pte_t rv;

  pte_val(rv) = (pgprot_val(pgprot) << 20) | page_to_pfn(page);

  return rv;
}

void pte_unmap(pte_t *pte){
  (void)pte;
}

pte_t pgoff_to_pte(pgoff_t off){BUG();}

/*
 * Conversion functions: convert a page and protection to a page entry,
 * and a page entry and page directory to the page they refer to.
 */
unsigned long pte_pfn(pte_t pte){
  return pte_val(pte) & 0xfffff;
}

pte_t * pte_offset_map(pmd_t *dir, unsigned long address){
  pte_t* rv;

  rv = (pte_t *)page_address(pmd_page(*dir)) + 
    ((address >> PAGE_SHIFT) & (PTRS_PER_PTE - 1));
  return rv;
}

/* to find an entry in a kernel page-table-directory */
pgd_t * pgd_offset_k(unsigned long address) 
{
  return pgd_offset(&init_mm, address);
}

/* Get the address to the PTE for a vaddr in specfic directory 
 */
pte_t * pte_offset_kernel(pmd_t * dir, unsigned long address){
  pte_t* rv;

  pr_debug("pte_offset_kernel for address %#lx pmd=%#lx\n",
	 address, pmd_val(*dir) );
  rv = (pte_t *)pmd_page_vaddr(*(dir)) + 
    ((address >> PAGE_SHIFT) & (PTRS_PER_PTE - 1));
  return rv;
}

pte_t pfn_pte(unsigned long  pfn, pgprot_t prot)
{
   pte_t pte;
   pte_val(pte) = pfn | (pgprot_val(prot) << 20);
   return pte;
}

pgoff_t pte_to_pgoff(pte_t pte){
   BUG();
}

struct page * pte_page(pte_t pte){
   return pfn_to_page(pte_pfn(pte));
}

int kern_addr_valid(unsigned long addr){BUG();}
