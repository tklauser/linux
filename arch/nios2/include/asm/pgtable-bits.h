/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994 - 2002 by Ralf Baechle
 * Copyright (C) 1999, 2000, 2001 Silicon Graphics, Inc.
 * Copyright (C) 2002  Maciej W. Rozycki
 */
#ifndef _ASM_NIOS2_PGTABLE_BITS_H
#define _ASM_NIOS2_PGTABLE_BITS_H

/* These are actual HW defined protection bits (unshifted) in TLBACC
 */
#define _PAGE_GLOBAL                (1<<0)
#define _PAGE_EXEC                  (1<<1)
#define _PAGE_WRITE                 (1<<2)
#define _PAGE_READ                  (1<<3)
#define _PAGE_CACHED                (1<<4)

/* TLBACC also has 7 IGNORE bits to use for SW defined attributes
 */
#define _PAGE_PRESENT               (1<<5)
#define _PAGE_ACCESSED              (1<<6)  
#define _PAGE_MODIFIED              (1<<7)  
#define _PAGE_FILE                  (1<<8)  
#define _PAGE_VALID                 (1<<9)
#define _PAGE_OLD                   (1<<10)

#endif /* _ASM_NIOS2_PGTABLE_BITS_H */
