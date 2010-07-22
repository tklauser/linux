#ifndef _ASM_NIOS2_CACHEFLUSH_H
#define _ASM_NIOS2_CACHEFLUSH_H

/*
 * Ported from m68knommu.
 *
 * (C) Copyright 2003, Microtronix Datacom Ltd.
 * (C) Copyright 2000-2002, Greg Ungerer <gerg@snapgear.com>
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
#include <linux/mm.h>
#include <asm/page.h>

#define ARCH_IMPLEMENTS_FLUSH_DCACHE_PAGE 1

extern void flush_cache_all(void);
extern void flush_cache_dup_mm(struct mm_struct *mm);
extern void flush_cache_mm(struct mm_struct *mm);
extern void flush_cache_page(struct vm_area_struct *vma, unsigned long vmaddr, unsigned long pfn);
extern void flush_cache_range(struct vm_area_struct *vma, unsigned long start, unsigned long end);
extern void flush_dcache_all(void);
extern void flush_dcache_page(struct page* page);
extern void flush_dcache_range(unsigned long start, unsigned long end);
extern void flush_icache_all(void);
extern void flush_icache_page(struct vm_area_struct *vma, struct page* page);
extern void flush_icache_range(unsigned long start,unsigned long end);
extern void copy_from_user_page(struct vm_area_struct *vma, struct page *page,
                               unsigned long user_vaddr,
                               void *dst, void *src, int len);

extern void copy_to_user_page(struct vm_area_struct *vma, struct page *page,
                              unsigned long user_vaddr,
                              void *dst, void *src, int len);


#define flush_cache_vmap(start, end) flush_dcache_range(start, end)
#define flush_cache_vunmap(start, end) flush_dcache_range(start, end)

#define flush_dcache_mmap_lock(mapping)         do { } while(0)
#define flush_dcache_mmap_unlock(mapping)       do { } while(0)


#endif /* _ASM_NIOS2_CACHEFLUSH_H */
