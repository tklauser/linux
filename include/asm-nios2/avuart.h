/*
 *	avuart.h -- Altera Avalon UART driver defines.
 *	based on : mcfuart.h
 */

#ifndef	avuart_h
#define	avuart_h

#include <linux/serial_core.h>
#include <linux/platform_device.h>

struct avalon_uart_platform_uart {
	unsigned long mapbase;	/* Physical address base */
	void __iomem *membase;	/* Virtual address if mapped */
	unsigned int irq;	/* Interrupt vector */
	unsigned int uartclk;	/* UART clock rate */
};

#define AVALON_UART_RXDATA_REG            0
#define AVALON_UART_TXDATA_REG            4
#define AVALON_UART_STATUS_REG            8
#define AVALON_UART_CONTROL_REG           12
#define AVALON_UART_DIVISOR_REG           16
#define AVALON_UART_EOP_REG               20

#define AVALON_UART_STATUS_PE_MSK         (0x1)
#define AVALON_UART_STATUS_FE_MSK         (0x2)
#define AVALON_UART_STATUS_BRK_MSK        (0x4)
#define AVALON_UART_STATUS_ROE_MSK        (0x8)
#define AVALON_UART_STATUS_TOE_MSK        (0x10)
#define AVALON_UART_STATUS_TMT_MSK        (0x20)
#define AVALON_UART_STATUS_TRDY_MSK       (0x40)
#define AVALON_UART_STATUS_RRDY_MSK       (0x80)
#define AVALON_UART_STATUS_E_MSK          (0x100)
#define AVALON_UART_STATUS_DCTS_MSK       (0x400)
#define AVALON_UART_STATUS_CTS_MSK        (0x800)
#define AVALON_UART_STATUS_EOP_MSK        (0x1000)

#define AVALON_UART_CONTROL_PE_MSK        (0x1)
#define AVALON_UART_CONTROL_FE_MSK        (0x2)
#define AVALON_UART_CONTROL_BRK_MSK       (0x4)
#define AVALON_UART_CONTROL_ROE_MSK       (0x8)
#define AVALON_UART_CONTROL_TOE_MSK       (0x10)
#define AVALON_UART_CONTROL_TMT_MSK       (0x20)
#define AVALON_UART_CONTROL_TRDY_MSK      (0x40)
#define AVALON_UART_CONTROL_RRDY_MSK      (0x80)
#define AVALON_UART_CONTROL_E_MSK         (0x100)
#define AVALON_UART_CONTROL_TRBK_MSK      (0x200)
#define AVALON_UART_CONTROL_DCTS_MSK      (0x400)
#define AVALON_UART_CONTROL_RTS_MSK       (0x800)
#define AVALON_UART_CONTROL_EOP_MSK       (0x1000)

#define AVALON_UART_EOP_MSK               (0xFF)
#define AVALON_UART_EOP_OFST              (0)

#endif /* avuart_h */
