/*
 * Copyright (C) 2004 Microtronix Datacom Ltd.
 *
 * Taken from m68k.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#ifndef _ASM_NIOS2_SYSTEM_H
#define _ASM_NIOS2_SYSTEM_H

#include <linux/linkage.h>
#include <asm/entry.h>
#include <linux/string.h>
#include <linux/irqflags.h>

asmlinkage void resume(void);

/*
 * switch_to(n) should switch tasks to task ptr, first checking that
 * ptr isn't the current task, in which case it does nothing.  This
 * also clears the TS-flag if the task we switched to has used the
 * math co-processor latest.
 */
#define switch_to(prev, next, last)			\
{							\
	void *_last;					\
	__asm__ __volatile__(				\
		"mov	r4, %1\n"			\
		"mov	r5, %2\n"			\
		"call	resume\n"			\
		"mov	%0,r4\n"			\
		: "=r" (_last)				\
		: "r" (prev), "r" (next)		\
		: "r4","r5","r7","r8","ra");		\
	(last) = _last;					\
}

#define iret() __asm__ __volatile__ ("eret": : :"memory", "ea")

#define nop()			asm volatile ("nop"::)
#define mb()			barrier()
#define rmb()			mb()
#define wmb()			mb()
#define set_mb(var, value)	do { (void) xchg(&var, value); } while (0)

#define smp_mb()	mb()
#define smp_rmb()	rmb()
#define smp_wmb()	wmb()
#define smp_read_barrier_depends()	do { } while(0)

#define xchg(ptr,x) ((__typeof__(*(ptr)))__xchg((unsigned long)(x),(ptr),sizeof(*(ptr))))

struct __xchg_dummy { unsigned long a[100]; };
#define __xg(x) ((volatile struct __xchg_dummy *)(x))

static inline unsigned long __xchg(unsigned long x, volatile void *ptr, int size)
{
	unsigned long tmp, flags;

	local_irq_save(flags);

	switch (size) {
	case 1:
		__asm__ __volatile__(
			"ldb	%0, %2\n"
			"stb	%1, %2\n"
			: "=&r" (tmp)
			: "r" (x), "m" (*__xg(ptr))
			: "memory");
		break;
	case 2:
		__asm__ __volatile__(
			"ldh	%0, %2\n"
			"sth	%1, %2\n"
			: "=&r" (tmp)
			: "r" (x), "m" (*__xg(ptr))
			: "memory");
		break;
	case 4:
		__asm__ __volatile__(
			"ldw	%0, %2\n"
			"stw	%1, %2\n"
			: "=&r" (tmp)
			: "r" (x), "m" (*__xg(ptr))
			: "memory");
		break;
	}

	local_irq_restore(flags);
	return tmp;
}

#define arch_align_stack(x) (x)

#include <asm-generic/cmpxchg.h>
#include <asm-generic/cmpxchg-local.h>

#endif /* _ASM_NIOS2_SYSTEM_H */
