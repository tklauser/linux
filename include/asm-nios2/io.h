#ifndef __NIOS2_IO_H__
#define __NIOS2_IO_H__

#define readb(addr) \
    ({ unsigned char __v = (*(volatile unsigned char *) (addr)); __v; })
#define readw(addr) \
    ({ unsigned short __v = (*(volatile unsigned short *) (addr)); __v; })
#define readl(addr) \
    ({ unsigned int __v = (*(volatile unsigned int *) (addr)); __v; })

#define readb_relaxed(addr) readb(addr)
#define readw_relaxed(addr) readw(addr)
#define readl_relaxed(addr) readl(addr)

#define writeb(b, addr) (void)((*(volatile unsigned char *) (addr)) = (b))
#define writew(b, addr) (void)((*(volatile unsigned short *) (addr)) = (b))
#define writel(b, addr) (void)((*(volatile unsigned int *) (addr)) = (b))

#define __raw_readb readb
#define __raw_readw readw
#define __raw_readl readl
#define __raw_writeb writeb
#define __raw_writew writew
#define __raw_writel writel

static inline void io_outsb(unsigned int addr, void *buf, int len)
{
	volatile unsigned char *ap = (volatile unsigned char *)addr;
	unsigned char *bp = (unsigned char *)buf;
	while (len--)
		*ap = *bp++;
}

static inline void io_outsw(unsigned int addr, void *buf, int len)
{
	volatile unsigned short *ap = (volatile unsigned short *)addr;
	unsigned short *bp = (unsigned short *)buf;
	while (len--)
		*ap = *bp++;
}

static inline void io_outsl(unsigned int addr, void *buf, int len)
{
	volatile unsigned int *ap = (volatile unsigned int *)addr;
	unsigned int *bp = (unsigned int *)buf;
	while (len--)
		*ap = *bp++;
}

static inline void io_insb(unsigned int addr, void *buf, int len)
{
	volatile unsigned char *ap = (volatile unsigned char *)addr;
	unsigned char *bp = (unsigned char *)buf;
	while (len--)
		*bp++ = *ap;
}

static inline void io_insw(unsigned int addr, void *buf, int len)
{
	volatile unsigned short *ap = (volatile unsigned short *)addr;
	unsigned short *bp = (unsigned short *)buf;
	while (len--)
		*bp++ = *ap;
}

static inline void io_insl(unsigned int addr, void *buf, int len)
{
	volatile unsigned int *ap = (volatile unsigned int *)addr;
	unsigned int *bp = (unsigned int *)buf;
	while (len--)
		*bp++ = *ap;
}

#define mmiowb()

/*
 *	make the short names macros so specific devices
 *	can override them as required
 */

#define memset_io(a, b, c)	memset((void *)(a), (b), (c))
#define memcpy_fromio(a, b, c)	memcpy((a), (void *)(b), (c))
#define memcpy_toio(a, b, c)	memcpy((void *)(a), (b), (c))

#define inb(addr)    readb(addr)
#define inw(addr)    readw(addr)
#define inl(addr)    readl(addr)
#define outb(x, addr) ((void) writeb(x, addr))
#define outw(x, addr) ((void) writew(x, addr))
#define outl(x, addr) ((void) writel(x, addr))

#define inb_p(addr)    inb(addr)
#define inw_p(addr)    inw(addr)
#define inl_p(addr)    inl(addr)
#define outb_p(x, addr) outb(x, addr)
#define outw_p(x, addr) outw(x, addr)
#define outl_p(x, addr) outl(x, addr)

#define outsb(a, b, l) io_outsb(a, b, l)
#define outsw(a, b, l) io_outsw(a, b, l)
#define outsl(a, b, l) io_outsl(a, b, l)

#define insb(a, b, l) io_insb(a, b, l)
#define insw(a, b, l) io_insw(a, b, l)
#define insl(a, b, l) io_insl(a, b, l)

#define IO_SPACE_LIMIT 0xffffffff

/* Values for nocacheflag and cmode */
#define IOMAP_FULL_CACHING		0
#define IOMAP_NOCACHE_SER		1
#define IOMAP_NOCACHE_NONSER		2
#define IOMAP_WRITETHROUGH		3

extern void *__ioremap(unsigned long physaddr, unsigned long size,
		       int cacheflag);
extern void __iounmap(void *addr, unsigned long size);

static inline void *ioremap(unsigned long physaddr, unsigned long size)
{
	return __ioremap(physaddr, size, IOMAP_NOCACHE_SER);
}

static inline void *ioremap_nocache(unsigned long physaddr, unsigned long size)
{
	return __ioremap(physaddr, size, IOMAP_NOCACHE_SER);
}

static inline void *ioremap_writethrough(unsigned long physaddr,
					 unsigned long size)
{
	return __ioremap(physaddr, size, IOMAP_WRITETHROUGH);
}

static inline void *ioremap_fullcache(unsigned long physaddr,
				      unsigned long size)
{
	return __ioremap(physaddr, size, IOMAP_FULL_CACHING);
}

extern void iounmap(void *addr);

/* Pages to physical address... */
#define page_to_phys(page)      ((page - mem_map) << PAGE_SHIFT)
#define page_to_bus(page)       ((page - mem_map) << PAGE_SHIFT)

/*
 * Macros used for converting between virtual and physical mappings.
 */
#define phys_to_virt(vaddr)	((void *) (vaddr))
#define virt_to_phys(vaddr)	((unsigned long) (vaddr))

#define virt_to_bus virt_to_phys
#define bus_to_virt phys_to_virt

/*
 * Convert a physical pointer to a virtual kernel pointer for /dev/mem
 * access
 */
#define xlate_dev_mem_ptr(p)	__va(p)

/*
 * Convert a virtual cached pointer to an uncached pointer
 */
#define xlate_dev_kmem_ptr(p)	p

/* Macros used for smc91x.c driver */
#define readsb(p, d, l)		insb(p, d, l)
#define readsw(p, d, l)		insw(p, d, l)
#define readsl(p, d, l)		insl(p, d, l)
#define writesb(p, d, l)	outsb(p, d, l)
#define writesw(p, d, l)	outsw(p, d, l)
#define writesl(p, d, l)	outsl(p, d, l)

#endif /* __NIOS2_IO_H__ */
