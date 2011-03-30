/*
 * Copyright (C) 2011 Tobias Klauser <tklauser@distanz.ch>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#ifndef _ASM_NIOS2_IRQ_H
#define _ASM_NIOS2_IRQ_H

#define NR_IRQ 32

static inline void irq_dispose_mapping(unsigned int virq)
{
	return;
}

#include <asm-generic/irq.h>

#endif
