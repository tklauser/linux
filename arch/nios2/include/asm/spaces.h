/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994 - 1999, 2000, 03, 04 Ralf Baechle
 * Copyright (C) 2000, 2002  Maciej W. Rozycki
 * Copyright (C) 1990, 1999, 2000 Silicon Graphics, Inc.
 */
#ifndef _ASM_NIOS2_GENERIC_SPACES_H
#define _ASM_NIOS2_GENERIC_SPACES_H

#include <asm/nios.h>

/*
 * This handles the memory map.
 * We handle pages at KSEG0 for kernels with 32 bit address space.
 */
#define PAGE_OFFSET		(KERNEL_REGION_BASE + DDR2_TOP_BASE)

/*
 * Memory above this physical address will be considered highmem.
 */
#ifndef HIGHMEM_START
#define HIGHMEM_START		0x20000000UL
#endif

#endif /* _ASM_NIOS2_GENERIC_SPACES_H */
