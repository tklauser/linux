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
#error "No FPGA configuration selected"
#endif

/* Fake CPU frequency if not defined by the design header file */
#ifndef CPU_FREQ
# define CPU_FREQ	0
#endif

#endif
