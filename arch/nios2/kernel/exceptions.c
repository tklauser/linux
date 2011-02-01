/*
 * Hardware exception handling
 *
 * Copyright (C) 2010 Tobias Klauser <tklauser@distanz.ch>
 *
 * based on arch/microblaze/kernel/exceptions.c
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License.  See the file COPYING in the main directory of this
 * archive for more details.
 */

#include <linux/kernel.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/module.h>

#include <asm/exceptions.h>

static DEFINE_SPINLOCK(die_lock);

void die(const char *str, struct pt_regs *regs, long err)
{
	console_verbose();
	spin_lock_irq(&die_lock);
	printk(KERN_WARNING "Oops: %s, sig: %ld\n", str, err);
	show_regs(regs);
	spin_unlock_irq(&die_lock);
	/* do_exit() should take care of panic'ing from an interrupt
	 * context so we don't handle it here
	 */
	do_exit(err);
}

void _exception(int signo, struct pt_regs *regs, int code, unsigned long addr)
{
	siginfo_t info;

	if (!user_mode(regs))
		die("Exception in kernel mode", regs, signo);

	info.si_signo = signo;
	info.si_errno = 0;
	info.si_code = code;
	info.si_addr = (void __user *) addr;
	force_sig_info(signo, &info, current);
}

void unhandled_exception(struct pt_regs *regs, int cause)
{
	cause /= 4;

	pr_warn("Unhandled exception #%d in %s mode\n",
			cause, user_mode(regs) ? "user" : "kernel");

	regs->ea -= 4;
	show_regs(regs);

	/* TODO: What should we do here? WRS code was halting the ISS with
	 * WRCTL(6,1) and spinning forever afterwards. */
}
