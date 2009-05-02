#include <linux/mm.h>

void flush_dcache_range(unsigned long start, unsigned long end)
{
	unsigned long addr;

	start &= ~(nasys_dcache_line_size - 1);
	end += (nasys_dcache_line_size - 1);
	end &= ~(nasys_dcache_line_size - 1);

	if (end > start + nasys_dcache_size) {
		end = start + nasys_dcache_size;
	}

	for (addr = start; addr < end; addr += nasys_dcache_line_size) {
	__asm__ __volatile__ ("   flushd 0(%0)\n" 
                            : /* Outputs */
                            : /* Inputs  */ "r"(addr) 
                            /* : No clobber */);
	}
}

void flush_icache_range(unsigned long start, unsigned long end)
{
	unsigned long addr;

	start &= ~(nasys_icache_line_size - 1);
	end += (nasys_icache_line_size - 1);
	end &= ~(nasys_icache_line_size - 1);

	if (end > start + nasys_icache_size) {
		end = start + nasys_icache_size;
	}

	for (addr = start; addr < end; addr += nasys_icache_line_size) {
	__asm__ __volatile__ ("   flushi %0\n" 
                            : /* Outputs */
                            : /* Inputs  */ "r"(addr) 
                            /* : No clobber */);
	}
	__asm__ __volatile(" flushp\n");
}

unsigned long kernel_map(unsigned long paddr, unsigned long size,
			 int nocacheflag, unsigned long *memavailp)
{
	return paddr;
}

int is_in_rom(unsigned long addr)
{
	/* Default case, not in ROM */
	return (0);
}

int __handle_mm_fault(struct mm_struct *mm, struct vm_area_struct *vma,
		      unsigned long address, int write_access)
{
	BUG();
	return VM_FAULT_OOM;
}
