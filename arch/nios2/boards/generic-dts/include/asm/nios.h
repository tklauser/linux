#ifndef _ASM_NIOS2_NIOS_H
#define _ASM_NIOS2_NIOS_H

/* Add some defines wich should REALLY be extracted from fdt */
/* ==> REMOVE */
#define DDR2_TOP_BASE	0x0
#define DDR2_TOP_SPAN	33554432
#define FAST_TLB_MISS_EXCEPTION_ADDR 0xc9000000
#define EXCEPTION_ADDR 0xc0000020
#define RESET_ADDR 0xc4000000

/* <== END REMOVE */

#define EXC_RESET		0	/* System reset		*/
#define EXC_CPU_RESET		1	/* CPU reset		*/
#define EXC_EXTERNAL_INTERRUPT	2	/* Interrupt		*/
#define EXC_TRAP		3	/* Trap instruction	*/
#define EXC_UNIMPLEMTED_INSN	4	/* Unimplemented instruction */
#define EXC_ILLEGAL_INSN	5	/* Illegal instruction	   */
#define EXC_DATA_UNALIGNED	6	/* Misaligned data access   */
#define EXC_CODE_UNALIGNED	7	/* Misaligned destination address   */
#define EXC_DIVISION_ERROR	8	/* Division error  */
#define EXC_SUPERV_INSN_ACCESS	9	/* Supervisor only instr. access       */
#define EXC_UNUSED		10	/* Unused     */
#define EXC_SUPERV_DATA_ACCESS	11	/* Supervisor only data address	      */
#define EXC_TLB_DOUBLEFAULT	12	/* TLB double fault.	   */
#define EXC_X_PROTECTION_FAULT	13	/* TLB permission violation (x) */
#define EXC_R_PROTECTION_FAULT	14	/* TLB permission violation (r) */
#define EXC_W_PROT_FAULT	15	/* TLB permission violation (w) */
#define EXC_MPU_REGION_VIOLATION 16	/* MPU region violation */

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
