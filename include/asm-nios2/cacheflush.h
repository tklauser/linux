#ifndef _NIOS2_CACHEFLUSH_H
#define _NIOS2_CACHEFLUSH_H

#include <linux/mm.h>

extern void flush_dcache_range(unsigned long start, unsigned long end);
extern void flush_icache_range(unsigned long start, unsigned long end);

static inline void flush_cache_range(struct vm_area_struct *vma,
				     unsigned long start, unsigned long end)
{
	flush_dcache_range(start, end);
	flush_icache_range(start, end);
}

static inline void flush_cache_all(void)
{
	flush_dcache_range(0, nasys_dcache_size);
	flush_icache_range(0, nasys_icache_size);
}

#define flush_dcache_all()			flush_dcache_range(0, nasys_dcache_size)
#define flush_cache_mm(mm)			do { } while (0)
#define flush_cache_page(vma, vmaddr)		do { } while (0)
#define flush_dcache_page(page)			do { } while (0)
#define flush_dcache_mmap_lock(mapping)		do { } while (0)
#define flush_dcache_mmap_unlock(mapping)	do { } while (0)
#define flush_icache_page(vma,pg)		do { } while (0)
#define flush_icache_user_range(vma,pg,adr,len)	do { } while (0)

#define copy_to_user_page(vma, page, vaddr, dst, src, len) \
	memcpy(dst, src, len)
#define copy_from_user_page(vma, page, vaddr, dst, src, len) \
	memcpy(dst, src, len)

#endif /* _NIOS2_CACHEFLUSH_H */
