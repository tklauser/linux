#include <asm-generic/vmlinux.lds.h>
#include <asm/nios.h>
		
OUTPUT_FORMAT("elf32-littlenios2", "elf32-littlenios2", "elf32-littlenios2")

OUTPUT_ARCH(nios)
ENTRY(_start)	/* Defined in head.S */

jiffies = jiffies_64;

SECTIONS
{
  . = nasys_program_mem;
  /* read-only */
  _stext = . ;
  _text = .;			/* Text and read-only data */
  .text : {
    *(.text)
    SCHED_TEXT
    LOCK_TEXT
    *(.fixup)
    *(.gnu.warning)
  } =0

  . = ALIGN(4) ;
  _etext = .;			/* End of text section */

  . = ALIGN(32);		/* Exception table */
  __start___ex_table = .;
  __ex_table : { *(__ex_table) }
  __stop___ex_table = .;

  RODATA

  /* writeable */
  .data : {			/* Data */
    /*
     * This ALIGN is needed as a workaround for a bug a gcc bug upto 4.1 which
     * limits the maximum alignment to at most 32kB and results in the following
     * warning:
     *
     *  CC      arch/mips/kernel/init_task.o
     * arch/mips/kernel/init_task.c:30: warning: alignment of ‘init_thread_union’
     * is greater than maximum object file alignment.  Using 32768
     */
    . = ALIGN(4096);
    *(.data.init_task)

    *(.data)

    CONSTRUCTORS
  }

  .lit8 : { *(.lit8) }
  .lit4 : { *(.lit4) }
  /* We want the small data sections together, so single-instruction offsets
     can access them all, and initialized data all before uninitialized, so
     we can shorten the on-disk segment size.  */
  .sdata     : { *(.sdata) }

  . = ALIGN(4096);
  __nosave_begin = .;
  .data_nosave : { *(.data.nosave) }
  . = ALIGN(4096);
  __nosave_end = .;

  . = ALIGN(32);
  .data.cacheline_aligned : { *(.data.cacheline_aligned) }

  _edata =  .;			/* End of data section */

  /* will be freed after init */
  . = ALIGN(4096);		/* Init code and data */
  __init_begin = .;
  .init.text : {
	_sinittext = .;
	*(.init.text)
	_einittext = .;
  }
  .init.data : { *(.init.data) }
  . = ALIGN(16);
  __setup_start = .;
  .init.setup : { *(.init.setup) }
  __setup_end = .;

  __initcall_start = .;
  .initcall.init : {
	INITCALLS
  }
  __initcall_end = .;

  __con_initcall_start = .;
  .con_initcall.init : { *(.con_initcall.init) }
  __con_initcall_end = .;
  SECURITY_INIT
    /* .exit.text is discarded at runtime, not link time, to deal with
     references from .rodata */
  .exit.text : { *(.exit.text) }
  .exit.data : { *(.exit.data) }
  . = ALIGN(4096);
  __initramfs_start = .;
  .init.ramfs : { *(.init.ramfs) }
  __initramfs_end = .;
  . = ALIGN(32);
  __per_cpu_start = .;
  .data.percpu  : { *(.data.percpu) }
  __per_cpu_end = .;
  . = ALIGN(4096);
  __init_end = .;
  /* freed after init ends here */

  __bss_start = .;		/* BSS */
  .sbss      : {
    *(.sbss)
    *(.scommon)
  }
  .bss : {
    *(.bss)
    *(COMMON)
  }
  __bss_stop = .;

  _end = . ;

  /* Sections to be discarded */
  /DISCARD/ : {
	*(.exit.text)
	*(.exit.data)
        *(.exitcall.exit)
  }


  STABS_DEBUG

  DWARF_DEBUG

  /* These must appear regardless of  .  */
  .gptab.sdata : { *(.gptab.data) *(.gptab.sdata) }
  .gptab.sbss : { *(.gptab.bss) *(.gptab.sbss) }
  .note : { *(.note) }
}
