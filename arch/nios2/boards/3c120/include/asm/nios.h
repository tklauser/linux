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
#define GPIO_LED1 0

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
#define GPIO_LED1 0

#else
#error "No FPGA configuration selected"
#endif

/* Fake CPU frequency if not defined by the design header file */
#ifndef CPU_FREQ
# define CPU_FREQ	0
#endif

#endif
