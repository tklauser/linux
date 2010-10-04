/*
 *  linux/arch/nios2/mm/ioremap.c
 *
 *   Copyright (C) 2009, Wind River Systems Inc
 *   Implemented by fredrik.markstrom@gmail.com and ivarholmqvist@gmail.com
 *
 *  linux/arch/nios2nommu/mm/ioremap.c, based on:
 *
 *  linux/arch/m68knommu/mm/kmap.c
 *
 *  Copyright (C) 2004 Microtronix Datacom Ltd.
 *  Copyright (C) 2000 Lineo, <davidm@lineo.com>
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
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <asm/setup.h>
#include <asm/page.h>
#include <asm/pgalloc.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/tlbflush.h>

static inline void remap_area_pte(pte_t * pte, unsigned long address,
                                  unsigned long size, 
                                  unsigned long phys_addr, unsigned long flags)
{
	unsigned long end;
	unsigned long pfn;
	pgprot_t pgprot = __pgprot(_PAGE_GLOBAL | _PAGE_PRESENT | _PAGE_READ
	                           | _PAGE_WRITE | flags);

	address &= ~PMD_MASK;
	end = address + size;
	if (end > PMD_SIZE)
		end = PMD_SIZE;
	if (address >= end)
		BUG();
	pfn = phys_addr >> PAGE_SHIFT;
	do {
		if (!pte_none(*pte)) {
			printk("remap_area_pte: page already exists\n");
			BUG();
		}
		set_pte(pte, pfn_pte(pfn, pgprot));
		address += PAGE_SIZE;
		pfn++;
		pte++;
	} while (address && (address < end));
}

static inline int remap_area_pmd(pmd_t * pmd, unsigned long address,
                                 unsigned long size, unsigned long phys_addr, 
                                 unsigned long flags)
{
	unsigned long end;

	address &= ~PGDIR_MASK;
	end = address + size;
	if (end > PGDIR_SIZE)
		end = PGDIR_SIZE;
	phys_addr -= address;
	if (address >= end)
		BUG();
	do {
		pte_t * pte = pte_alloc_kernel(pmd, address);
		if (!pte)
			return -ENOMEM;
		remap_area_pte(pte, address, end - address, address + phys_addr, flags);
		address = (address + PMD_SIZE) & PMD_MASK;
		pmd++;
	} while (address && (address < end));
	return 0;
}

static int remap_area_pages(unsigned long address, 
                            unsigned long phys_addr,
                            unsigned long size, unsigned long flags)
{
	int error;
	pgd_t * dir;
	unsigned long end = address + size;

	phys_addr -= address;
	dir = pgd_offset(&init_mm, address);
	flush_cache_all();
	if (address >= end)
		BUG();
	do {
		pud_t *pud;
		pmd_t *pmd;

		error = -ENOMEM;
		pud = pud_alloc(&init_mm, dir, address);
		if (!pud)
			break;
		pmd = pmd_alloc(&init_mm, pud, address);
		if (!pmd)
			break;
		if (remap_area_pmd(pmd, address, end - address,
					 phys_addr + address, flags))
			break;
		error = 0;
		address = (address + PGDIR_SIZE) & PGDIR_MASK;
		dir++;
	} while (address && (address < end));
	flush_tlb_all();
	return error;
}

#define IS_MAPPABLE_UNCACHEABLE(addr) (addr < 0x20000000UL)

/*
 * Map some physical address range into the kernel address space.
 */
void *__ioremap(unsigned long phys_addr, unsigned long size, int cacheflag)
{
	struct vm_struct * area;
	unsigned long offset;
	unsigned long last_addr;
	void * addr;

	/* Don't allow wraparound or zero size 
    */
	last_addr = phys_addr + size - 1;

	if (!size || last_addr < phys_addr)
		return NULL;

	/* Don't allow anybody to remap normal RAM that we're using..
	 */
	if (phys_addr > PHYS_OFFSET && phys_addr < virt_to_phys(high_memory)) {
		char *t_addr, *t_end;
		struct page *page;

		t_addr = __va(phys_addr);
		t_end = t_addr + (size - 1);
		for(page = virt_to_page(t_addr); page <= virt_to_page(t_end); page++)
			if(!PageReserved(page)) {
				return NULL;
         }
	}

	/*
	 * Map uncached objects in the low part of address space to 0xe0000000UL
	 */
	if (IS_MAPPABLE_UNCACHEABLE(phys_addr) && 
       IS_MAPPABLE_UNCACHEABLE(last_addr) &&
       !(cacheflag & _PAGE_CACHED))
   {
		return (void __iomem *)(0xe0000000UL + phys_addr);
   }


	/*
	 * Mappings have to be page-aligned
	 */
	offset = phys_addr & ~PAGE_MASK;
	phys_addr &= PAGE_MASK;
	size = PAGE_ALIGN(last_addr + 1) - phys_addr;

	/*
	 * Ok, go for it..
	 */
	area = get_vm_area(size, VM_IOREMAP);
	if (!area)
		return NULL;
	addr = area->addr;
	if (remap_area_pages((unsigned long) addr, phys_addr, size, cacheflag)) {
		vunmap(addr);
		return NULL;
	}
	return (void __iomem *) (offset + (char *)addr);
}
EXPORT_SYMBOL(__ioremap);

/*
 * __iounmap unmaps nearly everything, so be careful
 * it doesn't free currently pointer/page tables anymore but it
 * wasn't used anyway and might be added later.
 */
void __iounmap(void *addr)
{
	struct vm_struct *p;

	if (addr > (void*)0xe0000000UL)     /* FIXME */
		return;

	p = remove_vm_area((void *) (PAGE_MASK & (unsigned long __force) addr));
	if (!p)
		printk(KERN_ERR "iounmap: bad address %p\n", addr);
   kfree(p);
}
EXPORT_SYMBOL(__iounmap);
