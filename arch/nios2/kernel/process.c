/*--------------------------------------------------------------------
 *
 * arch/nios2/kernel/process.c
 *
 * Copyright (C) 2009, Wind River Systems Inc
 * Implemented by fredrik.markstrom@gmail.com and ivarholmqvist@gmail.com
 *
 * Derived from arch/nios2nommu/kernel/process.c
 *
 * Derived from M68knommu
 *
 *  Copyright (C) 1995  Hamish Macdonald
 *  Copyright (C) 2000-2002, David McCullough <davidm@snapgear.com>
 * Copyright (C) 2004   Microtronix Datacom Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 *  68060 fixes by Jesper Skov
 * Jan/20/2004		dgt	    NiosII
 *                            rdusp() === (pt_regs *) regs->sp
 *                            Monday:
 *                             asm-nios2nommu\processor.h now bears
 *                              inline thread_saved_pc
 *                                      (struct thread_struct *t)
 *                             Friday: it's back here now
 *
 ---------------------------------------------------------------------*/


/*
 * This file handles the architecture-dependent parts of process handling..
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/smp_lock.h>
#include <linux/stddef.h>
#include <linux/unistd.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/user.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/uaccess.h>
#include <linux/tick.h>
#include <linux/fs.h>
#include <linux/err.h>

#include <asm/system.h>
#include <asm/setup.h>
#include <asm/pgtable.h>
#include <asm/cacheflush.h>

#define print_mark(...)
#define printd(...)

asmlinkage void ret_from_fork(void);

/*
 * The following aren't currently used.
 */
void (*pm_idle)(void) = NULL;
EXPORT_SYMBOL(pm_idle);

void (*pm_power_off)(void) = NULL;
EXPORT_SYMBOL(pm_power_off);

void default_idle(void)
{
	local_irq_disable();
	if (!need_resched()) {
		local_irq_enable();
		__asm__("nop");   // was asm sleep
	} else
		local_irq_enable();
}

void (*idle)(void) = default_idle;

/*
 * The idle thread. There's no useful work to be
 * done, so just try to conserve power and have a
 * low exit latency (ie sit in a loop waiting for
 * somebody to say that they'd like to reschedule)
 */
void cpu_idle(void)
{
	while (1) {
		tick_nohz_stop_sched_tick(1);
		while (!need_resched())
			idle();
		tick_nohz_restart_sched_tick();
		preempt_enable_no_resched();
		schedule();
		preempt_disable();
	}
}

/*
 * The development boards have no way to pull a board
 * reset. Just jump to the cpu reset address and let
 * the boot loader take care of resetting periperals
 */
void machine_restart(char * __unused)
{
	local_irq_disable();
	if (__builtin_ldwio((void *)RESET_ADDR) == 0xffffffffUL){
	  printk("boot loader not installed, refusing to jump to "
            "jump to reset addr 0x%lx\n", (unsigned long)RESET_ADDR);		 
	  for (;;);
	}
	__asm__ __volatile__ (
	"jmp	%0\n\t"
	: 
	: "r" (RESET_ADDR)
	: "r4");
}

EXPORT_SYMBOL(machine_restart);

void machine_halt(void)
{
	local_irq_disable();
	for (;;);
}

EXPORT_SYMBOL(machine_halt);

/*
 * There is no way to power off the development
 * boards. So just spin lock for now. If you have
 * your own board with power down circuits add you
 * specific code here.
 */

void machine_power_off(void)
{
	local_irq_disable();
	for (;;);
}

EXPORT_SYMBOL(machine_power_off);

void show_regs(struct pt_regs * regs)
{
	printk(KERN_NOTICE "\n");

	printk(KERN_NOTICE "r1:  %08lx r2:  %08lx r3:  %08lx r4:  %08lx\n",
	       regs->r1,  regs->r2,  regs->r3,  regs->r4);

	printk(KERN_NOTICE "r5:  %08lx r6:  %08lx r7:  %08lx r8:  %08lx\n",
	       regs->r5,  regs->r6,  regs->r7,  regs->r8);

	printk(KERN_NOTICE "r9:  %08lx r10: %08lx r11: %08lx r12: %08lx\n",
	       regs->r9,  regs->r10, regs->r11, regs->r12);

	printk(KERN_NOTICE "r13: %08lx r14: %08lx r15: %08lx\n",
	       regs->r13, regs->r14, regs->r15);

	printk(KERN_NOTICE "ra:  %08lx fp:  %08lx sp:  %08lx gp:  %08lx\n",
	       regs->ra,  regs->fp,  regs->sp,  regs->gp);

	printk(KERN_NOTICE "ea:  %08lx estatus:  %08lx\n",
	       regs->ea,  regs->estatus);
}




/*
 * Create a kernel thread
 */
static ATTRIB_NORET void kernel_thread_helper(void *arg, int (*fn)(void *))
{
  int i;
  print_mark(__FILE__, __LINE__);
  printd("################ KTH arg=%p fn=%p sp=%p\n", arg, fn, &i);
  do_exit(fn(arg));
}

int kernel_thread(int (*fn)(void *), void * arg, unsigned long flags)
{
	struct pt_regs regs;
        print_mark(__FILE__, __LINE__);

	memset(&regs, 0, sizeof(regs));
	regs.r4 = (unsigned long)arg;
	regs.r5 = (unsigned long)fn;
	regs.ea = (unsigned long)kernel_thread_helper;
	regs.estatus =  NIOS2_STATUS_PIE_MSK;
	return do_fork(flags | CLONE_VM | CLONE_UNTRACED, 0, &regs, 0, NULL, NULL);
}
EXPORT_SYMBOL(kernel_thread);

void flush_thread(void)
{
	/* Now, this task is no longer a kernel thread. */
	current->thread.flags &= ~NIOS2_FLAG_KTHREAD;

	set_fs(USER_DS);
}

/*
 * "nios2_fork()".. By the time we get here, the
 * non-volatile registers have also been saved on the
 * stack. We do some ugly pointer stuff here.. (see
 * also copy_thread)
 */

asmlinkage int nios2_fork(struct pt_regs *regs)
{
   return do_fork(SIGCHLD, regs->sp, regs, 0, NULL, NULL);
}

/*
 * nios2_execve() executes a new program.
 */
asmlinkage int nios2_execve(struct pt_regs *regs)
{
	int error;
	char * filename;

	lock_kernel();
	filename = getname((char *) regs->r4);
	error = PTR_ERR(filename);
	if (IS_ERR(filename))
		goto out;
	error = do_execve(filename,
			  (const char __user *const __user *) regs->r5,
			  (const char __user *const __user *) regs->r6,
			  regs);
	putname(filename);
out:
	unlock_kernel();
	return error;
}

asmlinkage int nios2_vfork(struct pt_regs *regs)
{
	return do_fork(CLONE_VFORK | CLONE_VM | SIGCHLD, regs->sp, regs, 0, NULL, NULL);
}


asmlinkage int nios2_clone(struct pt_regs* regs)
{
	unsigned long clone_flags;
	unsigned long newsp;
	int __user *parent_tidptr, *child_tidptr;

	clone_flags = regs->r4;
	newsp = regs->r5;
	parent_tidptr = (int __user *) regs->r6;
   child_tidptr = (int __user *) regs->r8;

   if(newsp == 0) {
      newsp = regs->sp;
   }
   
	return do_fork(clone_flags, newsp, regs, 0, 
	               parent_tidptr, child_tidptr);
}

int copy_thread(unsigned long clone_flags,
		unsigned long usp, unsigned long topstk,
		struct task_struct * p, struct pt_regs * regs)
{
	struct pt_regs * childregs;
	struct switch_stack * childstack, *stack;

   childregs = task_pt_regs(p);

   /* Save pointer to registers in thread_struct */
	p->thread.kregs = childregs;

   /* Copy registers */
	*childregs = *regs;

   /* Copy stacktop and copy the top entrys from parent to child
    */
	stack = ((struct switch_stack *) regs) - 1;
	childstack = ((struct switch_stack *) childregs) - 1;
	*childstack = *stack;

	childstack->ra = (unsigned long)ret_from_fork;

	if(childregs->estatus & NIOS2_STATUS_U_MSK) {
	  childregs->sp = usp;
	}
	else {
     childregs->sp = (unsigned long)childstack;
	}
   /* Store the kernel stack in thread_struct */
	p->thread.ksp = (unsigned long)childstack;

   /* Initialize tls register.
    */
   if(clone_flags & CLONE_SETTLS) {
      childstack->r23 = regs->r7;
   }

	/* Set the return value for the child. */
	childregs->r2 = 0;
   childregs->r7 = 0;

	/* Set the return value for the parent. */
	regs->r2 = p->pid;
   regs->r7 = 0;  /* No error */

	return 0;
}

/*
 *	Generic dumping code. Used for panic and debug.
 */
void dump(struct pt_regs *fp)
{
	unsigned long	*sp;
	unsigned char	*tp;
	int		i;

	printk(KERN_EMERG "\nCURRENT PROCESS:\n\n");
	printk(KERN_EMERG "COMM=%s PID=%d\n", current->comm, current->pid);

	if (current->mm) {
		printk(KERN_EMERG "TEXT=%08x-%08x DATA=%08x-%08x BSS=%08x-%08x\n",
			(int) current->mm->start_code,
			(int) current->mm->end_code,
			(int) current->mm->start_data,
			(int) current->mm->end_data,
			(int) current->mm->end_data,
			(int) current->mm->brk);
		printk(KERN_EMERG "USER-STACK=%08x  KERNEL-STACK=%08x\n\n",
			(int) current->mm->start_stack,
			(int)(((unsigned long) current) + THREAD_SIZE));
	}

	printk(KERN_EMERG "PC: %08lx\n", fp->ea);
	printk(KERN_EMERG "SR: %08lx    SP: %08lx\n", (long) fp->estatus, (long) fp);

	printk(KERN_EMERG "r1: %08lx    r2: %08lx    r3: %08lx\n",
		fp->r1, fp->r2, fp->r3);

	printk(KERN_EMERG "r4: %08lx    r5: %08lx    r6: %08lx    r7: %08lx\n",
		fp->r4, fp->r5, fp->r6, fp->r7);
	printk(KERN_EMERG "r8: %08lx    r9: %08lx    r10: %08lx    r11: %08lx\n",
		fp->r8, fp->r9, fp->r10, fp->r11);
	printk(KERN_EMERG "r12: %08lx  r13: %08lx    r14: %08lx    r15: %08lx\n",
		fp->r12, fp->r13, fp->r14, fp->r15);
	printk(KERN_EMERG "or2: %08lx   ra: %08lx     fp: %08lx    sp: %08lx\n",
		fp->orig_r2, fp->ra, fp->fp, fp->sp);
	printk(KERN_EMERG "\nUSP: %08x   TRAPFRAME: %08x\n", (unsigned int) fp->sp,
		(unsigned int) fp);

	printk(KERN_EMERG "\nCODE:");
	tp = ((unsigned char *) fp->ea) - 0x20;
	for (sp = (unsigned long *) tp, i = 0; (i < 0x40);  i += 4) {
		if ((i % 0x10) == 0)
			printk(KERN_EMERG "\n%08x: ", (int) (tp + i));
		printk(KERN_EMERG "%08x ", (int) *sp++);
	}
	printk(KERN_EMERG "\n");

	printk(KERN_EMERG "\nKERNEL STACK:");
	tp = ((unsigned char *) fp) - 0x40;
	for (sp = (unsigned long *) tp, i = 0; (i < 0xc0); i += 4) {
		if ((i % 0x10) == 0)
			printk(KERN_EMERG "\n%08x: ", (int) (tp + i));
		printk(KERN_EMERG "%08x ", (int) *sp++);
	}
	printk(KERN_EMERG "\n");
	printk(KERN_EMERG "\n");

	printk(KERN_EMERG "\nUSER STACK:");
	tp = (unsigned char *) (fp->sp - 0x10);
	for (sp = (unsigned long *) tp, i = 0; (i < 0x80); i += 4) {
		if ((i % 0x10) == 0)
			printk(KERN_EMERG "\n%08x: ", (int) (tp + i));
		printk(KERN_EMERG "%08x ", (int) *sp++);
	}
	printk(KERN_EMERG "\n\n");
}

/*
 * These bracket the sleeping functions..
 */
extern void scheduling_functions_start_here(void);
extern void scheduling_functions_end_here(void);
#define first_sched	((unsigned long) scheduling_functions_start_here)
#define last_sched	((unsigned long) scheduling_functions_end_here)

unsigned long get_wchan(struct task_struct *p)
{
	unsigned long fp, pc;
	unsigned long stack_page;
	int count = 0;
	if (!p || p == current || p->state == TASK_RUNNING)
		return 0;

	stack_page = (unsigned long)p;
	fp = ((struct switch_stack *)p->thread.ksp)->fp;        //;dgt2
	do {
		if (fp < stack_page+sizeof(struct task_struct) ||
		    fp >= 8184+stack_page)                          //;dgt2;tmp
			return 0;
		pc = ((unsigned long *)fp)[1];
		if (!in_sched_functions(pc))
			return pc;
		fp = *(unsigned long *) fp;
	} while (count++ < 16);                                 //;dgt2;tmp
	return 0;
}
EXPORT_SYMBOL(get_wchan);

/*
 * Do necessary setup to start up a newly executed thread.
 * Will statup in user mode (status_extension = 0).
 */
void start_thread(struct pt_regs * regs, unsigned long pc, unsigned long sp)
{
	memset((void *) regs, 0, sizeof(struct pt_regs));
	regs->estatus = NIOS2_STATUS_PIE_MSK|NIOS2_STATUS_U_MSK;
	regs->ea = pc;
	regs->sp = sp;

	/* check if debug flag is set */
	if (current->thread.flags & NIOS2_FLAG_DEBUG ) {
		if ( *(u32*)pc == NIOS2_OP_NOP ) {
			*(u32*)pc = NIOS2_OP_BREAK;
			flush_dcache_range(pc, pc+4);
			flush_icache_range(pc, pc+4);
		}
	}
}

/* Fill in the fpu structure for a core dump.. */
int dump_fpu(struct pt_regs *regs, elf_fpregset_t *r)
{
  return 0;
}

