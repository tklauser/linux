/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2009, Wind River Systems Inc
 * Implemented by fredrik.markstrom@gmail.com and ivarholmqvist@gmail.com
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <asm/cacheflush.h>


static void __flush_dcache(unsigned long start, unsigned long end) {
   unsigned long addr;

   start &= ~(DCACHE_LINE_SIZE - 1);
   end += (DCACHE_LINE_SIZE - 1);
   end &= ~(DCACHE_LINE_SIZE - 1);

   if(end > start + DCACHE_SIZE) {
      end = start + DCACHE_SIZE;
   }

   for(addr = start; addr < end; addr += DCACHE_LINE_SIZE) {
      __asm__ __volatile__ ("   flushd 0(%0)\n" 
                            : /* Outputs */
                            : /* Inputs  */ "r"(addr) 
                            /* : No clobber */);
   }   
   //__asm__ __volatile(" flushp\n");
}

static void __flush_icache(unsigned long start, unsigned long end) {
   unsigned long addr;

   start &= ~(ICACHE_LINE_SIZE - 1);
   end += (ICACHE_LINE_SIZE - 1);
   end &= ~(ICACHE_LINE_SIZE - 1);

   if(end > start + ICACHE_SIZE) {
      end = start + ICACHE_SIZE;
   }

   for(addr = start; addr < end; addr += ICACHE_LINE_SIZE) {
      __asm__ __volatile__ ("   flushi %0\n" 
                            : /* Outputs */
                            : /* Inputs  */ "r"(addr) 
                            /* : No clobber */);
   }
   __asm__ __volatile(" flushp\n");
}

static void flush_aliases(struct address_space *mapping, struct page* page) {
   struct mm_struct *mm = current->active_mm;
   struct vm_area_struct *mpnt;
   struct prio_tree_iter iter;
   pgoff_t pgoff;
   
   pgoff = page->index;
   
   flush_dcache_mmap_lock(mapping);
   vma_prio_tree_foreach(mpnt, &iter, &mapping->i_mmap, pgoff, pgoff) {
      unsigned long offset;
      
      if (mpnt->vm_mm != mm)
         continue;
      if (!(mpnt->vm_flags & VM_MAYSHARE))
            continue;
      
      offset = (pgoff - mpnt->vm_pgoff) << PAGE_SHIFT;
      flush_cache_page(mpnt, mpnt->vm_start + offset, page_to_pfn(page));
   }
   flush_dcache_mmap_unlock(mapping);
}

extern void flush_cache_all()  {
   __flush_dcache(0, DCACHE_SIZE);
   __flush_icache(0, ICACHE_SIZE);
}

extern void flush_cache_mm(struct mm_struct *mm) {
   flush_cache_all();
}

extern void flush_cache_dup_mm(struct mm_struct *mm) {
   flush_cache_all();
}

extern void flush_icache_range(unsigned long start,unsigned long end) {
   __flush_icache(start, end);
}

extern void flush_dcache_range(unsigned long start, unsigned long end) {
   __flush_dcache(start, end);
   /* FIXME: Maybe we should remove __flush_icache ? 
    */
   __flush_icache(start, end);
}

extern void flush_cache_range(struct vm_area_struct *vma, unsigned long start, unsigned long end) {
   __flush_dcache(start, end);
   if(vma == NULL || (vma->vm_flags & VM_EXEC)) {
      __flush_icache(start, end);
   }
}

extern void flush_icache_page(struct vm_area_struct *vma, struct page* page) {
   unsigned long start = (unsigned long) page_address(page);
	unsigned long end = start + PAGE_SIZE;

   __flush_icache(start, end);
}

extern void flush_cache_page(struct vm_area_struct *vma, unsigned long vmaddr, unsigned long pfn) {
   unsigned long start = vmaddr;
	unsigned long end = start + PAGE_SIZE;

   __flush_dcache(start, end);
   if(vma->vm_flags & VM_EXEC) {
      __flush_icache(start, end);
   }
}

extern void flush_dcache_page(struct page* page) {
   struct address_space *mapping = page_mapping(page);

   /* Flush this page if there are aliases.
    */
   if(mapping) {
      if (!mapping_mapped(mapping)) {
         clear_bit(PG_arch_1, &page->flags);
      }
      else if(mapping) {
         unsigned long start = (unsigned long) page_address(page);
         __flush_dcache(start, start + PAGE_SIZE);
         flush_aliases(mapping,  page);
      }
   }
}
EXPORT_SYMBOL(flush_dcache_page);

void update_mmu_cache(struct vm_area_struct *vma,
				          unsigned long address, pte_t *pte)
{
	unsigned long pfn = pte_pfn(*pte);
	struct page *page;


   if (!pfn_valid(pfn)) {
		return;
   }

	page = pfn_to_page(pfn);

   if (!PageReserved(page) && !test_bit(PG_arch_1, &page->flags)) {
      unsigned long start = page_to_virt(page);
      struct address_space *mapping;

      __flush_dcache(start, start + PAGE_SIZE);

      mapping = page_mapping(page);
      if (mapping) {
         flush_aliases(mapping, page);
      }
      set_bit(PG_arch_1, &page->flags);
   }
}

extern void copy_user_page(void *vto, void *vfrom, unsigned long vaddr,
                			   struct page *to)
{
   flush_dcache_range(vaddr, vaddr + PAGE_SIZE);
   flush_icache_range(vaddr, vaddr + PAGE_SIZE);
   memcpy(vto, vfrom, PAGE_SIZE);
   flush_dcache_range((unsigned long)vto, (unsigned long)vto + PAGE_SIZE);
}

extern void clear_user_page(void *addr, unsigned long vaddr, struct page* page)
{
   flush_dcache_range(vaddr, vaddr + PAGE_SIZE);
   flush_icache_range(vaddr, vaddr + PAGE_SIZE);
   memset(addr, 0, PAGE_SIZE);
   flush_dcache_range((unsigned long)addr, (unsigned long)addr + PAGE_SIZE);
}

void copy_from_user_page(struct vm_area_struct *vma, struct page *page,
                         unsigned long user_vaddr,
                         void *dst, void *src, int len)
{
   flush_cache_page(vma, user_vaddr, page_to_pfn(page));
	memcpy(dst, src, len);
   flush_dcache_range((unsigned long)src, (unsigned long)src+len);
   if(vma->vm_flags & VM_EXEC) {
      flush_icache_range((unsigned long)src, (unsigned long)src+len);
   }
}

void copy_to_user_page(struct vm_area_struct *vma, struct page *page,
                       unsigned long user_vaddr,
                       void *dst, void *src, int len)
{
   flush_cache_page(vma, user_vaddr, page_to_pfn(page));
	memcpy(dst, src, len);
   flush_dcache_range((unsigned long)dst, (unsigned long)dst+len);
   if(vma->vm_flags & VM_EXEC) {
      flush_icache_range((unsigned long)dst, (unsigned long)dst+len);
   }
}
