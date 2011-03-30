/*
 * Based on arch/mips/include/asm/prom.h which is:
 *
 * Copyright (C) 2010 Cisco Systems Inc. <dediao@cisco.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#ifndef _ASM_NIOS2_PROM_H
#define _ASM_NIOS2_PROM_H

extern unsigned long early_altera_uart_or_juart_console(void);
extern int early_init_dt_scan_memory_arch(unsigned long node,
	const char *uname, int depth, void *data);

extern int reserve_mem_mach(unsigned long addr, unsigned long size);
extern void free_mem_mach(unsigned long addr, unsigned long size);

extern void device_tree_init(void);

#endif /* _ASM_NIOS2_PROM_H */
