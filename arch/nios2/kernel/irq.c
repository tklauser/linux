/*
 *	irq.c
 *
 *	(C) Copyright 2007, Greg Ungerer <gerg@snapgear.com>
 *	(C) Copyright 2008, Thomas Chou <thomas@wytron.com.tw>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/seq_file.h>
#include <asm/system.h>
#include <asm/traps.h>

#define IENABLE 3

asmlinkage void do_IRQ(int irq, struct pt_regs *regs)
{
	struct pt_regs *oldregs = set_irq_regs(regs);

	irq_enter();
	generic_handle_irq(irq);
	irq_exit();

	set_irq_regs(oldregs);
}

void ack_bad_irq(unsigned int irq)
{
	printk(KERN_ERR "IRQ: unexpected irq=%d\n", irq);
}

static void chip_unmask(unsigned int irq)
{
	unsigned ien;
	ien = __builtin_rdctl(IENABLE);
	ien |= (1 << irq);
	__builtin_wrctl(IENABLE, ien);
}

static void chip_mask(unsigned int irq)
{
	unsigned ien;
	ien = __builtin_rdctl(IENABLE);
	ien &= ~(1 << irq);
	__builtin_wrctl(IENABLE, ien);
}

static void chip_ack(unsigned int irq)
{
}

static int chip_set_type(unsigned int irq, unsigned int flow_type)
{
	return 0;
}

static struct irq_chip m_irq_chip = {
	.name = "NIOS2-INTC",
	.unmask = chip_unmask,
	.mask = chip_mask,
	.ack = chip_ack,
	.set_type = chip_set_type,
};

void __init init_IRQ(void)
{
	int irq;

	for (irq = 0; (irq < NR_IRQS); irq++) {
		irq_desc[irq].status = IRQ_DISABLED;
		irq_desc[irq].action = NULL;
		irq_desc[irq].depth = 1;
		irq_desc[irq].chip = &m_irq_chip;
		irq_desc[irq].handle_irq = handle_level_irq;
	}
}

int show_interrupts(struct seq_file *p, void *v)
{
	struct irqaction *ap;
	int irq = *((loff_t *) v);

	if (irq == 0)
		seq_puts(p, "           CPU0\n");

	if (irq < NR_IRQS) {
		ap = irq_desc[irq].action;
		if (ap) {
			seq_printf(p, "%3d: ", irq);
			seq_printf(p, "%10u ", kstat_irqs(irq));
			seq_printf(p, "%14s  ", irq_desc[irq].chip->name);

			seq_printf(p, "%s", ap->name);
			for (ap = ap->next; ap; ap = ap->next)
				seq_printf(p, ", %s", ap->name);
			seq_putc(p, '\n');
		}
	}

	return 0;
}
