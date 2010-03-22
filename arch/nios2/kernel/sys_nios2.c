/*
 * linux/arch/nios2nommu/kernel/sys_nios2.c
 *
 * Copyright (C) 2004 Microtronix Datacom Ltd.
 *
 * This file contains various random system calls that
 * have a non-standard calling sequence on the Linux/nios2nommu
 * platform.
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

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/smp.h>
#include <linux/sem.h>
#include <linux/msg.h>
#include <linux/shm.h>
#include <linux/stat.h>
#include <linux/syscalls.h>
#include <linux/mman.h>
#include <linux/file.h>
#include <linux/utsname.h>
#include <linux/uaccess.h>
#include <linux/ipc.h>
#include <linux/unistd.h>
#include <linux/fs.h>

#include <asm/setup.h>
#include <asm/cachectl.h>
#include <asm/traps.h>
#include <asm/cacheflush.h>

/* sys_cacheflush -- flush the processor cache.  
 */
asmlinkage int
sys_cacheflush (unsigned long addr, int scope, int cache, unsigned long len)
{
   flush_dcache_range(addr, addr+len);
   flush_icache_range(addr, addr+len);
   return(0);
}

asmlinkage int sys_getpagesize(void)
{
	return PAGE_SIZE;
}

/*
 * Do a system call from kernel instead of calling sys_execve so we
 * end up with proper pt_regs.
 */
int kernel_execve(const char *filename, char *const argv[], char *const envp[])
{
	register long __res __asm__ ("r2");
	register long __sc  __asm__ ("r2") = __NR_execve;
	register long __a   __asm__ ("r4") = (long) filename;
	register long __b   __asm__ ("r5") = (long) argv;
	register long __c   __asm__ ("r6") = (long) envp;
	__asm__ __volatile__ ("trap" : "=r" (__res)
			: "0" (__sc), "r" (__a), "r" (__b), "r" (__c)
			: "memory");

	return __res;
}

/* Uncomment if you uncomment the syscall printk tracing in entry.S
 */
#if 0

void print_syscall(int sc) {
   printk("Syscall %d\n", sc);
}


void print_syscall_ret(int rv, int err) {
   printk("  RET %d %d\n", err, rv);
}

#endif
