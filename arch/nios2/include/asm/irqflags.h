#ifndef _ASM_IRQFLAGS_H
#define _ASM_IRQFLAGS_H

#include <asm/nios.h>

static inline unsigned long arch_local_save_flags(void)
{
	return RDCTL(CTL_STATUS);
}

/*
 * This will restore ALL status register flags, not only the interrupt
 * mask flag.
 */
static inline void arch_local_irq_restore(unsigned long flags)
{
	WRCTL(CTL_STATUS, flags);
}

static inline void arch_local_irq_disable(void)
{
	unsigned long flags;
	flags = arch_local_save_flags();
	arch_local_irq_restore(flags & ~NIOS2_STATUS_PIE_MSK);
}

static inline void arch_local_irq_enable(void)
{
	unsigned long flags;
	flags = arch_local_save_flags();
	arch_local_irq_restore(flags | NIOS2_STATUS_PIE_MSK);
}

static inline int arch_irqs_disabled_flags(unsigned long flags)
{
	return (flags & NIOS2_STATUS_PIE_MSK) == 0;
}

static inline int arch_irqs_disabled(void)
{
	return arch_irqs_disabled_flags(arch_local_save_flags());
}

static inline unsigned long arch_local_irq_save(void)
{
	unsigned long flags;
	flags = arch_local_save_flags();
	arch_local_irq_restore(flags & ~NIOS2_STATUS_PIE_MSK);
	return flags;
}

#endif /* _ASM_IRQFLAGS_H */
