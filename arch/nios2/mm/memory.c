/*
 *  linux/arch/nios2/mm/memory.c
 * 
 *  Copyright (C) 2009, Wind River Systems Inc
 *  Implemented by fredrik.markstrom@gmail.com and ivarholmqvist@gmail.com
 *
 *  Based on:
 *
 *  linux/arch/nio2nommu/mm/memory.c
 *
 *  Copyright (C) 1995  Hamish Macdonald
 *  Copyright (C) 1998  Kenneth Albanowski <kjahds@kjahds.com>,
 *  Copyright (C) 1999-2002, Greg Ungerer (gerg@snapgear.com)
 *  Copyright (C) 2004  Microtronix Datacom Ltd.
 *
 *  Based on:
 *
 *  linux/arch/m68k/mm/memory.c
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

#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/module.h>

#include <asm/setup.h>
#include <asm/segment.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/io.h>

unsigned long kernel_map(unsigned long paddr, unsigned long size,
			 int nocacheflag, unsigned long *memavailp )
{
	return paddr;
}
