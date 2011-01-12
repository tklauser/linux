#ifndef _ASM_NIOS2_H__
#define _ASM_NIOS2_H__

/* Add some defines wich should REALLY be extracted from fdt */
/* ==> REMOVE */
#define DDR2_TOP_BASE	0x0
#define DDR2_TOP_SPAN	33554432
#define ICACHE_LINE_SIZE 32
#define ICACHE_LINE_SIZE_LOG2 5
#define ICACHE_SIZE 8192
#define DCACHE_LINE_SIZE 32
#define DCACHE_LINE_SIZE_LOG2 5
#define DCACHE_SIZE 4096
#define KERNEL_REGION_BASE 0xc0000000
#define IO_REGION_BASE 0xe0000000
#define KERNEL_MMU_REGION_BASE 0x80000000
#define FAST_TLB_MISS_EXCEPTION_ADDR 0xc9000000
#define EXCEPTION_ADDR 0xc0000020
#define TIMER_1MS_FREQ 60000000u
#define TIMER_1MS_BASE 0x8001240
#define TIMER_1MS_SPAN 64u
#define TIMER_1MS_IRQ 0
#define RESET_ADDR 0xc4000000
#define TLB_NUM_WAYS 16
#define TLB_NUM_WAYS_LOG2 4
#define TLB_PTR_SZ 8
#define TLB_NUM_ENTRIES 256
#define CPU_IMPLEMENTATION "fast"
#define HARDWARE_DIVIDE_PRESENT 0
#define HARDWARE_MULTIPLY_PRESENT 1
#define HARDWARE_MULX_PRESENT 0
#define PROCESS_ID_NUM_BITS 10
#define JTAG_UART_BASE 0x8001410

/* <== END REMOVE */

/* Fake CPU frequency if not defined by the design header file */
#ifndef CPU_FREQ
# define CPU_FREQ	0
#endif

/* Nios II Constants */
#define NIOS2_STATUS_PIE_MSK  0x1
#define NIOS2_STATUS_PIE_OFST 0
#define NIOS2_STATUS_U_MSK    0x2
#define NIOS2_STATUS_U_OFST   1


#define RDCTL(r) __builtin_rdctl(r)
#define WRCTL(r,v) __builtin_wrctl(r,v)

#define CTL_STATUS    0
#define CTL_ESTATUS   1
#define CTL_BSTATUS   2
#define CTL_IENABLE   3
#define CTL_IPENDING  4
#define CTL_CPUID     5
#define CTL_RSV1      6
#define CTL_EXCEPTION 7
#define CTL_PTEADDR   8
#define CTL_TLBACC    9
#define CTL_TLBMISC   10
#define CTL_RSV2      11
#define CTL_BADADDR   12
#define CTL_CONFIG    13
#define CTL_MPUBASE   14
#define CTL_MPUACC    15
#define CTL_SIM	   6


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

#endif

