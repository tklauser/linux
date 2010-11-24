#ifndef _ASM_NIOS2_IRQ_H
#define _ASM_NIOS2_IRQ_H

#define SYS_IRQS	32
#define NR_IRQS		SYS_IRQS

static inline void irq_dispose_mapping(unsigned int virq)
{
	return;
}

#include <asm-generic/irq.h>

#endif
