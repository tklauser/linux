/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2009, Wind River Systems Inc
 * Implemented by fredrik.markstrom@gmail.com and ivarholmqvist@gmail.com
 */

#include <asm/pgtable.h>
#include <linux/mm.h>

void dump_pte(unsigned long baseaddr, pte_t* pte) {
  int i;
  printk("\n==========================================================\n");
  printk("PTE @0x%p dump\n", pte);
  printk("==========================================================\n");
  for(i = 0; i < PTRS_PER_PTE; i++) {
    unsigned long v = pte_val(pte[i]);
    if(pte_present(*pte) || v != 0) {
      printk("%#10lx val=%#10lx (pfn=%ld prot=%#lx\n", 
	     baseaddr + i*PAGE_SIZE, v, v & 0xfffffUL, v >> 20);
    }
    else {
      if(v != 0) {
	/* Oooops, present but not 0... it's probably ok but unimplemented
	 */
	BUG();
      }
    }
  }
}

void dump_pgd(void) {
  int i;
  printk("\n==========================================================\n");
  printk("PGD dump\n");
  printk("==========================================================\n");
  printk("current->mm = %p\n", current->mm);
  if(current->mm == NULL) { return; }
  printk("current->mm->pgd = %p\n", current->mm->pgd);
  
  for(i = 0; i < PTRS_PER_PGD; i++) {
    if(pgd_val(current->mm->pgd[i]) == (unsigned long)invalid_pte_table) {
      continue;
    }
    if(pgd_val(current->mm->pgd[i]) == 0) {
      continue;
    }
    printk("pgd[%d] VA@%#10x = %#10lx\n", i, i << PGDIR_SHIFT,
	   pgd_val(current->mm->pgd[i]));

    {
      pgd_t* pgd = &current->mm->pgd[i];
      pud_t* pud = pud_offset(pgd, i << PGDIR_SHIFT);
      pmd_t* pmd = pmd_offset(pud, i << PGDIR_SHIFT);
      pte_t* pte = pte_offset_map(pmd, i << PGDIR_SHIFT);
      
      dump_pte(i << PGDIR_SHIFT, pte);
    }
  }
}

void dump_page_array(void) {
  int i;
  printk("Page array\n");
  printk("==========================================================\n");
  for(i = 0; i < num_physpages; i++) {
    struct page* page = &mem_map[i];
    printk("Page %4d flags=%#10lx _count=%-2d _mapcount=%-2d index=%ld virtual=%#lx\n",
	   i,
	   page->flags, page_count(page), page_mapcount(page), page->index, 
	   /*(unsigned long)page->virtual*/ 0xdeadbabeUL);
  }
}

void dump_all(void) {
  dump_pgd();
  dump_page_array();
}
