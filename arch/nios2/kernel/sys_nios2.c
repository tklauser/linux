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

/* common code for old and new mmaps */
static inline long do_mmap2(
	unsigned long addr, unsigned long len,
	unsigned long prot, unsigned long flags,
	unsigned long fd, unsigned long pgoff)
{
	int error = -EBADF;
	struct file * file = NULL;

	flags &= ~(MAP_EXECUTABLE | MAP_DENYWRITE);
	if (!(flags & MAP_ANONYMOUS)) {
		file = fget(fd);
		if (!file)
			goto out;
	}

	down_write(&current->mm->mmap_sem);
	error = do_mmap_pgoff(file, addr, len, prot, flags, pgoff);
	up_write(&current->mm->mmap_sem);

	if (file)
		fput(file);
out:
	return error;
}

asmlinkage long sys_mmap2(unsigned long addr, unsigned long len,
	unsigned long prot, unsigned long flags,
	unsigned long fd, unsigned long pgoff)
{
  return do_mmap2(addr, len, prot, flags, fd, pgoff);
}

asmlinkage long sys_mmap(unsigned long addr, unsigned long len,
                         unsigned long prot, unsigned long flags,
                         unsigned long fd, unsigned long offset)
{
	unsigned long result;

	result = -EINVAL;
	if (offset & ~PAGE_MASK)
		goto out;

	result = do_mmap2(addr, len, prot, flags, fd, offset >> PAGE_SHIFT);

out:
	return result;
}

/*
 * sys_ipc() is the de-multiplexer for the SysV IPC calls..
 *
 * This is really horribly ugly.
 */
asmlinkage int sys_ipc (unsigned int call, int first, int second,
			unsigned long third, void __user *ptr, long fifth)
{
	int version, ret;

	version = call >> 16; /* hack for backward compatibility */
	call &= 0xffff;

	switch (call) {
	case SEMOP:
		return sys_semtimedop (first, (struct sembuf __user *)ptr,
		                       second, NULL);
	case SEMTIMEDOP:
		return sys_semtimedop (first, (struct sembuf __user *)ptr,
				       second,
				       (const struct timespec __user *)fifth);
	case SEMGET:
		return sys_semget (first, second, third);
	case SEMCTL: {
		union semun fourth;
		if (!ptr)
			return -EINVAL;
		if (get_user(fourth.__pad, (void __user *__user *) ptr))
			return -EFAULT;
		return sys_semctl (first, second, third, fourth);
	}

	case MSGSND:
		return sys_msgsnd (first, (struct msgbuf __user *) ptr,
				   second, third);
	case MSGRCV:
		switch (version) {
		case 0: {
			struct ipc_kludge tmp;
			if (!ptr)
				return -EINVAL;

			if (copy_from_user(&tmp,
					   (struct ipc_kludge __user *) ptr,
					   sizeof (tmp)))
				return -EFAULT;
			return sys_msgrcv (first, tmp.msgp, second,
					   tmp.msgtyp, third);
		}
		default:
			return sys_msgrcv (first,
					   (struct msgbuf __user *) ptr,
					   second, fifth, third);
		}
	case MSGGET:
		return sys_msgget ((key_t) first, second);
	case MSGCTL:
		return sys_msgctl (first, second,
				   (struct msqid_ds __user *) ptr);

	case SHMAT:
		switch (version) {
		default: {
			unsigned long raddr;
			ret = do_shmat (first, (char __user *) ptr, second,
					&raddr);
			if (ret)
				return ret;
			return put_user (raddr, (unsigned long __user *) third);
		}
		case 1:	/* iBCS2 emulator entry point */
			if (!segment_eq(get_fs(), get_ds()))
				return -EINVAL;
			return do_shmat (first, (char __user *) ptr, second,
					 (unsigned long *) third);
		}
	case SHMDT:
		return sys_shmdt ((char __user *)ptr);
	case SHMGET:
		return sys_shmget (first, second, third);
	case SHMCTL:
		return sys_shmctl (first, second,
				   (struct shmid_ds __user *) ptr);
	default:
		return -ENOSYS;
	}
}

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
