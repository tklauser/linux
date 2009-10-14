/*
 * altera_uart.h -- Altera UART driver defines.
 */

#ifndef	__ALTUART_H
#define	__ALTUART_H

#include <linux/serial_core.h>
#include <linux/platform_device.h>

struct altera_uart_platform_uart {
	unsigned long mapbase;	/* Physical address base */
	void __iomem *membase;	/* Virtual address if mapped */
	unsigned int irq;	/* Interrupt vector */
	unsigned int uartclk;	/* UART clock rate */
};

#endif /* __ALTUART_H */
