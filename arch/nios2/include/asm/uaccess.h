/*
 * Copyright (C) 2010, Tobias Klauser <tklauser@distanz.ch>
 * Copyright (C) 2009, Wind River Systems Inc
 *   Implemented by fredrik.markstrom@gmail.com and ivarholmqvist@gmail.com
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#ifndef _ASM_NIOS2_UACCESS_H
#define _ASM_NIOS2_UACCESS_H

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/thread_info.h>

#define VERIFY_READ    0
#define VERIFY_WRITE   1

/*
 * The exception table consists of pairs of addresses: the first is the
 * address of an instruction that is allowed to fault, and the second is
 * the address at which the program should continue.  No registers are
 * modified, so it is entirely up to the continuation code to figure out
 * what to do.
 *
 * All the routines below use bits of fixup code that are out of line
 * with the main instruction path.  This means when everything is well,
 * we don't even have to jump over them.  Further, they do not intrude
 * on our cache or tlb entries.
 */
struct exception_table_entry {
	unsigned long insn;
	unsigned long fixup;
};

extern int fixup_exception(struct pt_regs *regs);

/*
 * Segment stuff
 */
#define MAKE_MM_SEG(s)		((mm_segment_t) { (s) } )
#define USER_DS			MAKE_MM_SEG(0x80000000UL)
#define KERNEL_DS		MAKE_MM_SEG(0)

#define get_ds()		(KERNEL_DS)
#define get_fs()		(current_thread_info()->addr_limit)
#define set_fs(seg)		(current_thread_info()->addr_limit = (seg))

#define segment_eq(a, b)	((a).seg == (b).seg)

#define __access_ok(addr, len)			\
	(((signed long)(((long)get_fs().seg) &	\
			((long)(addr) | (((long)(addr)) + (len)) | (len)))) == 0)

#define access_ok(type, addr, len)		\
	likely(__access_ok((addr), (len)))

extern long __copy_from_user(void* to, const void __user *from, unsigned long n);
extern long __copy_to_user(void __user *to, const void *from, unsigned long n);

static inline long copy_from_user(void *to, const void __user *from, unsigned long n)
{
	if (!access_ok(VERIFY_READ, from, n))
		return n;
	return __copy_from_user(to, from, n);
}

static inline long __copy_from_user_inatomic(void *to, const void __user *from,
						unsigned long n)
{
	return __copy_from_user(to, from, n);
}

static inline long __copy_to_user_inatomic(void __user *to, const void *from,
						unsigned long n)
{
	return __copy_to_user(to, from, n);
}

static inline long copy_to_user(void __user *to, const void *from, unsigned long n)
{
	if (!access_ok(VERIFY_WRITE, to, n))
		return n;
	return __copy_to_user(to, from, n);
}

extern long strncpy_from_user(char *__to, const char __user *__from,
				long __len);
extern long strnlen_user(const char __user *s, long n);

__kernel_size_t __clear_user(void __user *addr, __kernel_size_t size);

static inline size_t clear_user(void __user *addr, __kernel_size_t size)
{
	if (!access_ok(VERIFY_WRITE, addr, size))
		return size;
	return __clear_user(addr, size);
}

/* Optimized macros
 */
#define __get_user_asm(val, insn, addr, err)                         \
   {                                                                 \
                                                                     \
      __asm__ __volatile__(                                          \
        "       movi    %0, %3                                  \n"  \
        "1:   " insn " %1, 0(%2)                                \n"  \
        "       movi     %0, 0                                  \n"  \
        "2:                                                     \n"  \
        "       .section __ex_table,\"a\"                       \n"  \
        "       .word 1b, 2b                                    \n"  \
        "       .previous                                       "    \
        : "=&r" (err), "=r" (val)                                    \
        : "r" (addr), "i" (-EFAULT));                                \
   }

#define __get_user_unknown(val, size, ptr, err) do {           \
      err = 0;                                                 \
      if(copy_from_user(&(val), ptr, size)) {                  \
         err = -EFAULT;                                        \
      }                                                        \
   } while(0)

#define __get_user_common(val, size, ptr, err)                     \
   do {                                                            \
      switch (size) {                                              \
         case 1: __get_user_asm(val, "ldbu", ptr, err); break;     \
         case 2: __get_user_asm(val, "ldhu", ptr, err); break;     \
         case 4: __get_user_asm(val, "ldw", ptr, err); break;      \
         default: __get_user_unknown(val, size, ptr, err); break;  \
      }                                                            \
   } while (0)

#define __get_user(x,ptr)                                               \
   ({                                                                   \
      long __gu_err = -EFAULT;                                          \
      const __typeof__(*(ptr)) __user * __gu_ptr = (ptr);               \
      unsigned long __gu_val; /* FIXME: should be __typeof__ */         \
      __get_user_common(__gu_val, sizeof(*(ptr)), __gu_ptr, __gu_err);  \
      (x) = (__typeof__(x))__gu_val;                                    \
      __gu_err;                                                         \
   })

#define get_user(x,ptr)                                                 \
   ({                                                                   \
      long __gu_err = -EFAULT;                                          \
      const __typeof__(*(ptr)) __user * __gu_ptr = (ptr);               \
      unsigned long __gu_val = 0;                                       \
      if (access_ok(VERIFY_READ,  __gu_ptr, sizeof(*__gu_ptr)))		\
         __get_user_common(__gu_val, sizeof(*__gu_ptr), __gu_ptr, __gu_err); \
      (x) = (__typeof__(x))__gu_val;                                    \
      __gu_err;                                                         \
   })

#define __put_user_asm(val, insn, ptr,err)                           \
   {                                                                 \
      __asm__ __volatile__(                                          \
        "       movi    %0, %3                                  \n"  \
        "1:   " insn " %1, 0(%2)                                \n"  \
        "       movi     %0, 0                                  \n"  \
        "2:                                                     \n"  \
        "       .section __ex_table,\"a\"                       \n"  \
        "       .word 1b, 2b                                    \n"  \
        "       .previous                                       \n"  \
        : "=&r" (err)                                                \
        : "r" (val), "r" (ptr), "i" (-EFAULT));                      \
   }

#define __put_user_unknown(val, size, ptr, err) do {           \
      err = 0;                                                 \
      if(copy_to_user(ptr, &(val), size)) {                    \
         err = -EFAULT;                                        \
      }                                                        \
   } while(0)

#define __put_user_common(val, size, ptr, err)                    \
   do {                                                           \
      switch (size) {                                             \
         case 1: __put_user_asm(val, "stb", ptr, err); break;     \
         case 2: __put_user_asm(val, "sth", ptr, err); break;     \
         case 4: __put_user_asm(val, "stw", ptr, err); break;     \
         default: __put_user_unknown(val, size, ptr, err); break; \
      }                                                           \
   } while(0)

#define __put_user(x, ptr)                                              \
   ({                                                                   \
      long __pu_err = -EFAULT;                                          \
      __typeof__(*(ptr)) __user * __pu_ptr = (ptr);                     \
      __typeof__(*(ptr)) __pu_val = (__typeof(*ptr))(x);                \
      if (access_ok(VERIFY_WRITE, __pu_ptr, sizeof(*__pu_ptr)))		\
         __put_user_common(__pu_val, sizeof(*__pu_ptr), __pu_ptr, __pu_err); \
      __pu_err;                                                         \
   })

#define put_user(x, ptr)                                                \
   ({                                                                   \
      long __pu_err = -EFAULT;                                          \
      __typeof__(*(ptr)) __user * __pu_ptr = (ptr);                     \
      __typeof__(*(ptr)) __pu_val = (__typeof(*ptr))(x);                \
      if (access_ok(VERIFY_WRITE, __pu_ptr, sizeof(*__pu_ptr)))		\
         __put_user_common(__pu_val, sizeof(*__pu_ptr), __pu_ptr, __pu_err); \
      __pu_err;                                                         \
   })

#endif /* _ASM_NIOS2_UACCESS_H */
