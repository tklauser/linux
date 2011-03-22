#ifndef _ASM_NIOS2_NIOS_H
#define _ASM_NIOS2_NIOS_H

/* Add some defines wich should REALLY be extracted from fdt */
/* ==> REMOVE */
#define DDR2_TOP_BASE	0x0
#define DDR2_TOP_SPAN	33554432
#define FAST_TLB_MISS_EXCEPTION_ADDR 0xc9000000
#define EXCEPTION_ADDR 0xc0000020

/* <== END REMOVE */

/*
 * Maximum possible cache sizes on the Nios II. Only used in head.S where
 * the information from cpuinfo is not yet available.
 */
#define NIOS2_DCACHE_SIZE	65536
#define NIOS2_ICACHE_SIZE	65536

/*
 * Minumum possible cache line sizes on the Nios II. Only used in head.S
 * where the information from cpuinfo is not yet available.
 */
#define NIOS2_DCACHE_LINE_SIZE	4

/* Nios II instruction cache line size is always 32 bytes */
#define NIOS2_ICACHE_LINE_SIZE	32
#define NIOS2_ICACHE_LINE_SHIFT	5

#endif /* _ASM_NIOS2_NIOS_H */
