/*
 *	altjuart.h -- Altera JTAG UART driver defines.
 */

#ifndef	altjuart_h
#define	altjuart_h

#include <linux/serial_core.h>
#include <linux/platform_device.h>

struct altera_jtaguart_platform_uart {
	unsigned long mapbase;	/* Physical address base */
	void __iomem *membase;	/* Virtual address if mapped */
	unsigned int irq;	/* Interrupt vector */
};

#endif /* altjuart_h */
