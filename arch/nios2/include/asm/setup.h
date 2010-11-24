#ifndef _ASM_NIOS2_SETUP_H
#define _ASM_NIOS2_SETUP_H

#include <asm-generic/setup.h>

#ifndef __ASSEMBLY__
# ifdef __KERNEL__
extern char cmd_line[COMMAND_LINE_SIZE];
# endif/* __KERNEL__ */
#endif /* __ASSEMBLY__ */

#endif /* _ASM_NIOS2_SETUP_H */
