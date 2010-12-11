/*
 * Copyright (C) 2010 Tobias Klauser <tklauser@distanz.ch>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License.  See the file COPYING in the main directory of this
 * archive for more details.
 */

#ifndef _ASM_NIOS2_EXCEPTIONS_H_
#define _ASM_NIOS2_EXCEPTIONS_H_

void die(const char *str, struct pt_regs *regs, long err);
void _exception(int signr, struct pt_regs *regs, int code, unsigned long addr);

#endif /* _ASM_NIOS2_EXCEPTIONS_H_ */
