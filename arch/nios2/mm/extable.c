/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2009, Wind River Systems Inc
 * Implemented by fredrik.markstrom@gmail.com and ivarholmqvist@gmail.com
 */
#include <linux/module.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>

int fixup_exception(struct pt_regs *regs, unsigned long address)
{
	const struct exception_table_entry *fixup;

	fixup = search_exception_tables(regs->ea);
	if (fixup) {
		regs->ea = fixup->nextinsn;

		return 1;
	}

	return 0;
}
