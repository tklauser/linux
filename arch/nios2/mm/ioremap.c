/*  linux/arch/nios2/mm/ioremap.c, based on:
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
 */

#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <asm/setup.h>
#include <asm/segment.h>
#include <asm/page.h>
#include <asm/pgalloc.h>
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/system.h>

/*
 * Map some physical address range into the cached or uncached kernel address space.
 */

void *__ioremap(unsigned long physaddr, unsigned long size, int cacheflag)
{
	if (cacheflag == IOMAP_FULL_CACHING) {
		return (void *)(physaddr & ~0x80000000);
	} else {
		flush_dcache_range(physaddr, physaddr + size);
		return (void *)(physaddr | 0x80000000);
	}
}

/*
 * Unmap a ioremap()ed region again
 */
void iounmap(void *addr)
{
}

/*
 * __iounmap unmaps nearly everything, so be careful
 * it doesn't free currently pointer/page tables anymore but it
 * wans't used anyway and might be added later.
 */
void __iounmap(void *addr, unsigned long size)
{
}
