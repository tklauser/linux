/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * Copyright (C) 2009, Wind River Systems Inc
 * Implemented by fredrik.markstrom@gmail.com and ivarholmqvist@gmail.com
 */
#include <linux/console.h>
#include <linux/init.h>
#include <asm/io.h>

#if defined(CONFIG_SERIAL_ALTERA_JTAGUART_CONSOLE)

#if JTAG_UART_BASE > 0x20000000
#error Cannot map JTAG uart directly to IO_REGION
#error please disable early console
#endif

#define ALTERA_JTAGUART_CONTROL_REG               4

#define ALTERA_JTAGUART_CONTROL_WSPACE_MSK        (0xFFFF0000)
#define ALTERA_JTAGUART_CONTROL_AC_MSK            (0x00000400)

static void early_console_write(struct console *con, const char *s, unsigned n)
{
  unsigned long base = JTAG_UART_BASE + IO_REGION_BASE;
  unsigned long status;

  while (n-- && *s) {
     status = __builtin_ldwio((void *)(base + ALTERA_JTAGUART_CONTROL_REG));
     while((status & ALTERA_JTAGUART_CONTROL_WSPACE_MSK) == 0) 
     {
#if defined(CONFIG_SERIAL_ALTERA_JTAGUART_CONSOLE_BYPASS)
        if ((status & ALTERA_JTAGUART_CONTROL_AC_MSK) == 0) {
           return;	/* no connection activity */
        }
#endif
        status = __builtin_ldwio((void *)(base + ALTERA_JTAGUART_CONTROL_REG));
     }
     __builtin_stwio((void *)base, *s++);
   }
}

#elif defined(CONFIG_SERIAL_ALTERA_UART_CONSOLE)

#if UART_BASE > 0x20000000
#error Cannot map UART directly to IO_REGION
#error please disable early console
#endif

#define ALTERA_UART_TXDATA_REG		4
#define ALTERA_UART_STATUS_REG		8
#define ALTERA_UART_STATUS_TRDY		0x0040

static void early_console_write(struct console *con, const char *s, unsigned n)
{
  unsigned long base = UART_BASE + IO_REGION_BASE;
  int crlf = 0;

  while (n-- && *s) {
     while (!(__builtin_ldwio((void *)(base + ALTERA_UART_STATUS_REG)) & ALTERA_UART_STATUS_TRDY))
       ;
     crlf = (*s == '\n' && !crlf);	/* '\n' -> '\r' + '\n' */
     __builtin_stwio((void *)(base + ALTERA_UART_TXDATA_REG), crlf ? (n++, '\r') : *s++);
   }
}

#else
#error Neither SERIAL_ALTERA_JTAGUART_CONSOLE nor SERIAL_ALTERA_UART_CONSOLE selected???
#endif

static struct console early_console = {
	.name	= "early",
	.write	= early_console_write,
	.flags	= CON_PRINTBUFFER | CON_BOOT,
	.index	= -1
};

void __init setup_early_printk(void)
{
#if defined(CONFIG_SERIAL_ALTERA_JTAGUART_CONSOLE_BYPASS)
  {
    unsigned long base = JTAG_UART_BASE + IO_REGION_BASE;
    unsigned long status;
    
    /* Clear activity bit so BYPASS doesn't stall if we've used jtag for 
     * downloading the kernel. This might cause early data to be lost even 
     * if the jtag terminal is running.
     */
    status = __builtin_ldwio((void *)(base + ALTERA_JTAGUART_CONTROL_REG));
    __builtin_stwio((void *)(base + ALTERA_JTAGUART_CONTROL_REG), 
		    status | ALTERA_JTAGUART_CONTROL_AC_MSK);
  }
#endif

   register_console(&early_console);
   early_console_write(0, "Early printk initialized\n", 80);
}

void __init disable_early_printk(void)
{
}
