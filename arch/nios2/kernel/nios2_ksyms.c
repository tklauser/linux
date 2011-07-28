/*--------------------------------------------------------------------
 *
 * arch/nios2nommu/kernel/nios_ksyms.c
 *
 * Derived from Nios1
 *
 * Copyright (C) 2004   Microtronix Datacom Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * vic - copied from v850
 * Jan/20/2004		dgt	    NiosII
 *
 ---------------------------------------------------------------------*/

#include <linux/module.h>

/* string functions */

EXPORT_SYMBOL(memcpy);
EXPORT_SYMBOL(memset);
EXPORT_SYMBOL(memmove);

/*
 * libgcc functions - functions that are used internally by the
 * compiler...  (prototypes are not correct though, but that
 * doesn't really matter since they're not versioned).
 */
extern void __gcc_bcmp(void);
EXPORT_SYMBOL(__gcc_bcmp);
extern void __ashldi3(void);
EXPORT_SYMBOL(__ashldi3);
extern void __ashrdi3(void);
EXPORT_SYMBOL(__ashrdi3);
extern void __cmpdi2(void);
EXPORT_SYMBOL(__cmpdi2);
extern void __divdi3(void);
EXPORT_SYMBOL(__divdi3);
extern void __divsi3(void);
EXPORT_SYMBOL(__divsi3);
extern void __lshrdi3(void);
EXPORT_SYMBOL(__lshrdi3);
extern void __moddi3(void);
EXPORT_SYMBOL(__moddi3);
extern void __modsi3(void);
EXPORT_SYMBOL(__modsi3);
extern void __muldi3(void);
EXPORT_SYMBOL(__muldi3);
extern void __mulsi3(void);
EXPORT_SYMBOL(__mulsi3);
extern void __negdi2(void);
EXPORT_SYMBOL(__negdi2);
extern void __ucmpdi2(void);
EXPORT_SYMBOL(__ucmpdi2);
extern void __udivdi3(void);
EXPORT_SYMBOL(__udivdi3);
extern void __udivmoddi4(void);
EXPORT_SYMBOL(__udivmoddi4);
extern void __udivsi3(void);
EXPORT_SYMBOL(__udivsi3);
extern void __umoddi3(void);
EXPORT_SYMBOL(__umoddi3);
extern void __umodsi3(void);
EXPORT_SYMBOL(__umodsi3);
