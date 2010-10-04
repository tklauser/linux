/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * linux/nios2/mm/fault.c
 *
 * Copyright (C) 2009, Wind River Systems Inc
 * Implemented by fredrik.markstrom@gmail.com and ivarholmqvist@gmail.com
 *
 * Based on:
 *
 * linux/nios2nommu/mm/fault.c
 *
 * Copyright (C) 1995 - 2000 by Ralf Baechle
 */

#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/ptrace.h>
#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/module.h>

#include <asm/mmu_context.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/ptrace.h>

#include <asm/tlbstats.h>

extern struct tlb_stat statistics;

/*
 * This routine handles page faults.  It determines the address,
 * and the problem, and then passes it off to one of the appropriate
 * routines.
 */
asmlinkage void do_page_fault(struct pt_regs *regs, unsigned long cause,
                              unsigned long address)
{
	struct vm_area_struct * vma = NULL;
	struct task_struct *tsk = current;
	struct mm_struct *mm = tsk->mm;
	const int field = sizeof(unsigned long) * 2;
	siginfo_t info;
	int fault;
   int write = 0;

	statistics.do_page_fault++;
	info.si_code = SEGV_MAPERR;

	/*
	 * We fault-in kernel-space virtual memory on-demand. The
	 * 'reference' page table is init_mm.pgd.
	 *
	 * NOTE! We MUST NOT take any locks for this case. We may
	 * be in an interrupt or a critical region, and should
	 * only copy the information from the master page table,
	 * nothing more.
	 */
	if (unlikely(address >= VMALLOC_START && address <= VMALLOC_END)) {
	  if (user_mode(regs)) {
	    goto bad_area_nosemaphore;
	  }
	  else {
        goto vmalloc_fault;
	  }
   }
#ifdef MODULE_START
	if (unlikely(address >= MODULE_START && address < MODULE_END))
               goto vmalloc_fault;
#endif
	if(unlikely(address >= TASK_SIZE)) {
		goto bad_area_nosemaphore;
	}
	/*
	 * If we're in an interrupt or have no user
	 * context, we must not take the fault..
	 */
	if (in_atomic() || !mm)
		goto bad_area_nosemaphore;

	down_read(&mm->mmap_sem);
	vma = find_vma(mm, address);
	if (!vma)
		goto bad_area;
	if (vma->vm_start <= address)
		goto good_area;
	if (!(vma->vm_flags & VM_GROWSDOWN))
		goto bad_area;
	if (expand_stack(vma, address))
		goto bad_area;
/*
 * Ok, we have a good vm_area for this memory access, so
 * we can handle it..
 */
good_area:
	info.si_code = SEGV_ACCERR;

   switch(cause) {
      case EXC_SUPERV_INSN_ACCESS:
         goto bad_area;

      case EXC_SUPERV_DATA_ACCESS:
         goto bad_area;

      case EXC_X_PROTECTION_FAULT:
         if (!(vma->vm_flags & VM_EXEC)) {
            goto bad_area;
         }
         break;
      case EXC_R_PROTECTION_FAULT:
         if (!(vma->vm_flags & VM_READ)) {
            goto bad_area;
         }
         break;
      case EXC_W_PROT_FAULT:
         write = 1;
         if (!(vma->vm_flags & VM_WRITE)) {
            goto bad_area;
         }
         break;
	}

survive:
	/*
	 * If for any reason at all we couldn't handle the fault,
	 * make sure we exit gracefully rather than endlessly redo
	 * the fault.
	 */
	fault = handle_mm_fault(mm, vma, address, write);
	if (unlikely(fault & VM_FAULT_ERROR)) {
		if (fault & VM_FAULT_OOM)
			goto out_of_memory;
		else if (fault & VM_FAULT_SIGBUS)
			goto do_sigbus;
		BUG();
	}
	if (fault & VM_FAULT_MAJOR)
		current->maj_flt++;
	else
		current->min_flt++;

	up_read(&mm->mmap_sem);
	return;

/*
 * Something tried to access memory that isn't in our memory map..
 * Fix it, but check if it's kernel or user first..
 */
bad_area:
	up_read(&mm->mmap_sem);

bad_area_nosemaphore:
	/* User mode accesses just cause a SIGSEGV */
	if (user_mode(regs)) {
#if 0
		printk("do_page_fault() #2: sending SIGSEGV to %s for "
		       "invalid %s\n%0*lx (ea == %0*lx)(ra == %0*lx)\n",
		       tsk->comm,
		       write ? "write access to" : "read access from",
		       field, address,
		       field, (unsigned long) regs->ea, 
		       field, (unsigned long) regs->ra);
#endif
		info.si_signo = SIGSEGV;
		info.si_errno = 0;
		/* info.si_code has been set above */
		info.si_addr = (void __user *) address;
		force_sig_info(SIGSEGV, &info, tsk);
		return;
	}

no_context:
	/* Are we prepared to handle this kernel fault? */
	if (fixup_exception(regs))
		return;

	/*
	 * Oops. The kernel tried to access some bad page. We'll have to
	 * terminate things with extreme prejudice.
	 */
	bust_spinlocks(1);

	printk(KERN_ALERT "Unable to handle kernel paging request at "
	       "virtual address %0*lx, epc == %0*lx, ra == %0*lx\n",
	       field, address, field, regs->ea, field,  regs->ra);
	panic("Oops");

/*
 * We ran out of memory, or some other thing happened to us that made
 * us unable to handle the page fault gracefully.
 */
out_of_memory:
	up_read(&mm->mmap_sem);
	if (is_global_init(tsk)) {
		yield();
		down_read(&mm->mmap_sem);
		goto survive;
	}
	printk(KERN_INFO "VM: killing process %s\n", tsk->comm);
	if (user_mode(regs))
		do_group_exit(SIGKILL);
	goto no_context;

do_sigbus:
	up_read(&mm->mmap_sem);

	/* Kernel mode? Handle exceptions or die */
	if (!user_mode(regs))
		goto no_context;
	else
	/*
	 * Send a sigbus, regardless of whether we were in kernel
	 * or user mode.
	 */
#if 0
		printk("do_page_fault() #3: sending SIGBUS to %s for "
		       "invalid %s\n%0*lx (epc == %0*lx, ra == %0*lx)\n",
		       tsk->comm,
		       write ? "write access to" : "read access from",
		       field, address,
		       field, (unsigned long) regs->ea,
		       field, (unsigned long) regs->ra);
#endif
	info.si_signo = SIGBUS;
	info.si_errno = 0;
	info.si_code = BUS_ADRERR;
	info.si_addr = (void __user *) address;
	force_sig_info(SIGBUS, &info, tsk);

	return;
vmalloc_fault:
	{
		/*
		 * Synchronize this task's top level page-table
		 * with the 'reference' page table.
		 *
		 * Do _not_ use "tsk" here. We might be inside
		 * an interrupt in the middle of a task switch..
		 */
		int offset = pgd_index(address);
		pgd_t *pgd, *pgd_k;
		pud_t *pud, *pud_k;
		pmd_t *pmd, *pmd_k;
		pte_t *pte_k;

#if 1
      /* FIXME: Is this entierly correct ?
       */
		pgd = (pgd_t *) pgd_current + offset;
#else
		pgd = &current->mm->pgd[offset];
#endif
		pgd_k = init_mm.pgd + offset;

		if (!pgd_present(*pgd_k))
			goto no_context;
		set_pgd(pgd, *pgd_k);

		pud = pud_offset(pgd, address);
		pud_k = pud_offset(pgd_k, address);
		if (!pud_present(*pud_k))
			goto no_context;
		pmd = pmd_offset(pud, address);
		pmd_k = pmd_offset(pud_k, address);
		if (!pmd_present(*pmd_k))
			goto no_context;
		set_pmd(pmd, *pmd_k);

		pte_k = pte_offset_kernel(pmd_k, address);
		if (!pte_present(*pte_k))
			goto no_context;

		flush_tlb_one(address);
		return;
	}
}
