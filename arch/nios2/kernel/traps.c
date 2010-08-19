/*
 * arch/niosnommu/kernel/traps.c
 *
 * Copyright 2004 Microtronix Datacom Ltd.
 * Copyright 2001 Vic Phillips
 * Copyright 1995 David S. Miller (davem@caip.rutgers.edu)
 *
 * hacked from:
 *
 * arch/sparcnommu/kernel/traps.c
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

#include <linux/sched.h>  /* for jiffies */
#include <linux/kernel.h>
#include <linux/signal.h>
#include <linux/module.h>
#include <linux/mm.h>

#include <asm/delay.h>
#include <asm/system.h>
#include <asm/ptrace.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/unistd.h>
#include <linux/uaccess.h>

#include <asm/nios.h>

/* #define TRAP_DEBUG */

/*
 * The architecture-independent backtrace generator
 */
void dump_stack(void)
{
	unsigned long stack;

	show_stack(current, &stack);
}

EXPORT_SYMBOL(dump_stack);

/*
 * The show_stack is an external API which we do not use ourselves.
 */

int kstack_depth_to_print = 48;

void show_stack(struct task_struct *task, unsigned long *stack)
{
	unsigned long *endstack, addr;
	extern char _start, _etext;
	int i;

	if (!stack) {
		if (task)
			stack = (unsigned long *)task->thread.ksp;
		else
			stack = (unsigned long *)&stack;
	}

	addr = (unsigned long) stack;
	endstack = (unsigned long *) PAGE_ALIGN(addr);

	printk(KERN_EMERG "Stack from %08lx:", (unsigned long)stack);
	for (i = 0; i < kstack_depth_to_print; i++) {
		if (stack + 1 > endstack)
			break;
		if (i % 8 == 0)
			printk(KERN_EMERG "\n       ");
		printk(KERN_EMERG " %08lx", *stack++);
	}

	printk(KERN_EMERG "\nCall Trace:");
	i = 0;
	while (stack + 1 <= endstack) {
		addr = *stack++;
		/*
		 * If the address is either in the text segment of the
		 * kernel, or in the region which contains vmalloc'ed
		 * memory, it *may* be the address of a calling
		 * routine; if so, print it so that someone tracing
		 * down the cause of the crash will be able to figure
		 * out the call path that was taken.
		 */
		if (((addr >= (unsigned long) &_start) &&
		     (addr <= (unsigned long) &_etext))) {
			if (i % 4 == 0)
				printk(KERN_EMERG "\n       ");
			printk(KERN_EMERG " [<%08lx>]", addr);
			i++;
		}
	}
	printk(KERN_EMERG "\n");
}

void trap_init(void)
{
#ifdef DEBUG
	printk("trap_init reached\n");
#endif
}

/* Breakpoint handler */
asmlinkage void breakpoint_c(struct pt_regs *fp)
{
	siginfo_t info;

/* 	The breakpoint entry code has moved the PC on by 4 bytes, so we must */
/* 	move it back.  This could be done on the host but we do it here */
/* 	because monitor.S of JATG gdbserver does it. */
	fp->ea -= 4;

	//	printk("Breakpoint detected, instr=0x%08lx ea=0x%08lx ra=0x%08lx sp=0x%08lx\n", *(unsigned long*)fp->ea, fp->ea, fp->ra, fp->sp);
	
	info.si_code = TRAP_BRKPT;
	info.si_signo = SIGTRAP;
	info.si_errno = 0;
	info.si_addr = (void *) fp->ea;

	force_sig_info(info.si_signo, &info, current);
}


#ifndef CONFIG_ALIGNMENT_TRAP
/* Alignment handler 
 */
asmlinkage void handle_unaligned_c(struct pt_regs *fp)
{
	siginfo_t info;
   unsigned long addr = RDCTL(CTL_BADADDR);
   int cause = RDCTL(CTL_EXCEPTION)/4;

	fp->ea -= 4;

	if (fixup_exception(fp))
		return;

   if(!user_mode(fp)) {
      printk("Unaligned access from kernel mode, this might be a hardware\n");
      printk("problem, dump registers and restart the instruction\n");
      printk("  BADADDR 0x%08lx\n", addr);
      printk("  cause   %d\n", cause);
      printk("  op-code 0x%08lx\n", *(unsigned long*)(fp->ea));
      show_regs(fp);
      return;
   }

   //	printk("Unaligned access instr=0x%08lx addr=%#lx ea=0x%08lx ra=0x%08lx sp=0x%08lx\n", 
   //   *(unsigned long*)fp->ea, addr, fp->ea, fp->ra, fp->sp);
	
	info.si_code = BUS_ADRALN;
	info.si_signo = SIGBUS;
	info.si_errno = 0;
	info.si_addr = (void *) fp->ea;

	force_sig_info(info.si_signo, &info, current);
}
#endif /* !CONFIG_ALIGNMENT_TRAP */

/* Illegal instruction handler 
 */
asmlinkage void handle_illegal_c(struct pt_regs *fp)
{
	siginfo_t info;

	fp->ea -= 4;

	//	printk("Illegal instruction instr=0x%08lx ea=0x%08lx ra=0x%08lx sp=0x%08lx\n", 
	//          *(unsigned long*)fp->ea, fp->ea, fp->ra, fp->sp);
	
	info.si_code = ILL_PRVOPC;
	info.si_signo = SIGILL;
	info.si_errno = 0;
	info.si_addr = (void *) fp->ea;

	force_sig_info(info.si_signo, &info, current);
}

/* Supervisor instruction handler 
 */
asmlinkage void handle_supervisor_instr(struct pt_regs *fp)
{
	siginfo_t info;

	fp->ea -= 4;

	//	printk("Supervisor instruction instr=0x%08lx ea=0x%08lx ra=0x%08lx sp=0x%08lx\n", 
	//          *(unsigned long*)fp->ea, fp->ea, fp->ra, fp->sp);

	info.si_code = ILL_PRVOPC;
	info.si_signo = SIGILL;
	info.si_errno = 0;
	info.si_addr = (void *) fp->ea;

	force_sig_info(info.si_signo, &info, current);
}


/* Illegal division error
 */
asmlinkage void handle_diverror_c(struct pt_regs *fp)
{
	siginfo_t info;

	fp->ea -= 4;

	//	printk("Division error instr=0x%08lx ea=0x%08lx ra=0x%08lx sp=0x%08lx\n", 
	//          *(unsigned long*)fp->ea, fp->ea, fp->ra, fp->sp);
	
	info.si_code = FPE_INTDIV;
	info.si_signo = SIGFPE;
	info.si_errno = 0;
	info.si_addr = (void *) fp->ea;

	force_sig_info(info.si_signo, &info, current);
}
