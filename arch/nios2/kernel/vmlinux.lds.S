#include <asm-generic/vmlinux.lds.h>
#include <asm/nios.h>

OUTPUT_FORMAT("elf32-littlenios2", "elf32-littlenios2", "elf32-littlenios2")

OUTPUT_ARCH(nios)
ENTRY(_start)	/* Defined in head.S */

jiffies = jiffies_64;
PAGE_SIZE = 4096;
THREAD_SIZE = 8192;
CACHE_LINE = 32;

SECTIONS
{
	. = DDR2_TOP_BASE | KERNEL_REGION_BASE;

	_text = .;
	_stext = .;
	HEAD_TEXT_SECTION
	.text : {
		TEXT_TEXT
		SCHED_TEXT
		LOCK_TEXT
		IRQENTRY_TEXT
		KPROBES_TEXT
	} =0
	_etext = .;

	EXCEPTION_TABLE(CACHE_LINE)

	__init_begin = .;
	INIT_TEXT_SECTION(PAGE_SIZE)
	INIT_DATA_SECTION(PAGE_SIZE)
	PERCPU(PAGE_SIZE)
	__init_end = .;

        _sdata = .;
	RO_DATA_SECTION(PAGE_SIZE)
	RW_DATA_SECTION(CACHE_LINE, PAGE_SIZE, THREAD_SIZE)
	_edata = .;

	BSS_SECTION(0, 0, 0)
	_end = .;

	STABS_DEBUG
	DWARF_DEBUG
	NOTES

	DISCARDS
}
