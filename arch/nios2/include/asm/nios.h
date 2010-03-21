#ifndef _ASM_NIOS2_H__
#define _ASM_NIOS2_H__

#if defined(CONFIG_NIOS2_DEFAULT_MMU)
#include <asm/default_mmu.h>

/* Added compability mode with macro names from "old" design...
 * FIXME: we should really fix drivers instead. but this makes it
 * easier to switch between old and new design...
 */
#define CONFIG_ALTERA_CYCLONE_III
#define DDR2_TOP_BASE DDR2_LO_LATENCY_128M_BASE
#define DDR2_TOP_SPAN DDR2_LO_LATENCY_128M_SPAN
#define EXT_FLASH_BASE CFI_FLASH_64M_BASE
#define EXT_FLASH_SPAN CFI_FLASH_64M_SPAN
#define TIMER_1MS_FREQ LINUX_TIMER_1MS_FREQ
#define TIMER_1MS_BASE LINUX_TIMER_1MS_BASE
#define TIMER_1MS_SPAN LINUX_TIMER_1MS_SPAN
#define TIMER_1MS_IRQ LINUX_TIMER_1MS_IRQ

#elif defined(CONFIG_NIOS2_MAXIMUM_MMU)
#include <asm/maximum_mmu.h>

#define CONFIG_ALTERA_CYCLONE_III
#define DDR2_TOP_BASE DDR2_LO_LATENCY_128M_BASE
#define DDR2_TOP_SPAN DDR2_LO_LATENCY_128M_SPAN
#define EXT_FLASH_BASE CFI_FLASH_64M_BASE
#define EXT_FLASH_SPAN CFI_FLASH_64M_SPAN
#define TIMER_1MS_FREQ LINUX_TIMER_1MS_FREQ
#define TIMER_1MS_BASE LINUX_TIMER_1MS_BASE
#define TIMER_1MS_SPAN LINUX_TIMER_1MS_SPAN
#define TIMER_1MS_IRQ LINUX_TIMER_1MS_IRQ

#elif defined(CONFIG_NIOS2_CUSTOM_FPGA)
#include <asm/custom_fpga.h>

#define CONFIG_ALTERA_NEEK_C3
#define DDR2_TOP_BASE DDR_SDRAM_BASE
#define DDR2_TOP_SPAN DDR_SDRAM_SPAN
#define TIMER_1MS_FREQ SYS_CLK_TIMER_FREQ
#define TIMER_1MS_BASE SYS_CLK_TIMER_BASE
#define TIMER_1MS_SPAN SYS_CLK_TIMER_SPAN
#define TIMER_1MS_IRQ SYS_CLK_TIMER_IRQ

#elif defined(CONFIG_NIOS2_NEEK_OCM)
#include <asm/neek_ocm_fpga.h>

#define CONFIG_ALTERA_NEEK_C3
#define DDR2_TOP_BASE DDR_SDRAM_BASE
#define DDR2_TOP_SPAN DDR_SDRAM_SPAN
#define TIMER_1MS_FREQ SYS_CLK_TIMER_FREQ
#define TIMER_1MS_BASE SYS_CLK_TIMER_BASE
#define TIMER_1MS_SPAN SYS_CLK_TIMER_SPAN
#define TIMER_1MS_IRQ SYS_CLK_TIMER_IRQ
#define GPIO_I2C_0_SDA 7
#define GPIO_I2C_0_SCL 6
#define GPIO_I2C_1_SDA 1
#define GPIO_I2C_1_SCL 0
#define GPIO_I2C_2_SDA 5
#define GPIO_I2C_2_SCL 4
#define GPIO_SCEN 3 /* LCD i2c en */
#define GPIO_LED1 2

#else
#error "No FPGA configuration selected
#endif

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
#define CTL_SIM    6


#define EXC_RESET               0       /* System reset         */
#define EXC_CPU_RESET           1       /* CPU reset            */
#define EXC_EXTERNAL_INTERRUPT  2       /* Interrupt            */
#define EXC_TRAP                3       /* Trap instruction     */
#define EXC_UNIMPLEMTED_INSN    4       /* Unimplemented instruction */
#define EXC_ILLEGAL_INSN        5       /* Illegal instruction     */
#define EXC_DATA_UNALIGNED      6       /* Misaligned data access   */
#define EXC_CODE_UNALIGNED      7       /* Misaligned destination address   */
#define EXC_DIVISION_ERROR      8       /* Division error  */
#define EXC_SUPERV_INSN_ACCESS  9       /* Supervisor only instr. access       */
#define EXC_UNUSED              10      /* Unused     */
#define EXC_SUPERV_DATA_ACCESS  11      /* Supervisor only data address       */
#define EXC_TLB_DOUBLEFAULT     12      /* TLB double fault.       */
#define EXC_X_PROTECTION_FAULT  13      /* TLB permission violation (x) */
#define EXC_R_PROTECTION_FAULT  14      /* TLB permission violation (r) */
#define EXC_W_PROT_FAULT        15      /* TLB permission violation (w) */
#define EXC_MPU_REGION_VIOLATION 16     /* MPU region violation */

#endif

