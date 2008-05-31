#include <asm/nios2.h>
#include <asm/io.h>

#if defined(CONFIG_SERIAL_ALTERA_JTAGUART_CONSOLE)

#define ALTERA_JTAGUART_DATA_REG                  0
#define ALTERA_JTAGUART_CONTROL_REG               4
#define ALTERA_JTAGUART_CONTROL_AC_MSK            (0x00000400)
#define ALTERA_JTAGUART_CONTROL_WSPACE_MSK        (0xFFFF0000)
static unsigned uartbase;

#if defined(CONFIG_SERIAL_ALTERA_JTAGUART_CONSOLE_BYPASS)
static void jtag_putc(int ch)
{
	if (readl(uartbase + ALTERA_JTAGUART_CONTROL_REG) &
	    ALTERA_JTAGUART_CONTROL_WSPACE_MSK)
		writeb(ch, uartbase + ALTERA_JTAGUART_DATA_REG);
}
#else
static void jtag_putc(int ch)
{
	while ((readl(uartbase + ALTERA_JTAGUART_CONTROL_REG) &
		ALTERA_JTAGUART_CONTROL_WSPACE_MSK) == 0) ;
	writeb(ch, uartbase + ALTERA_JTAGUART_DATA_REG);
}
#endif

static int putchar(int ch)
{
	jtag_putc(ch);
	return ch;
}

static void console_init(void)
{
	uartbase = ioremap(na_jtag_uart, 8);
	writel(ALTERA_JTAGUART_CONTROL_AC_MSK,
	       uartbase + ALTERA_JTAGUART_CONTROL_REG);
}

#elif defined(CONFIG_SERIAL_ALTERA_UART_CONSOLE)

#define ALTERA_UART_TXDATA_REG            4
#define ALTERA_UART_STATUS_REG            8
#define ALTERA_UART_DIVISOR_REG           16
#define ALTERA_UART_STATUS_TRDY_MSK       (0x40)
static unsigned uartbase;

static void uart_putc(int ch)
{
	int i;

	for (i = 0; (i < 0x10000); i++) {
		if (readw(base + ALTERA_UART_STATUS_REG) &
		    ALTERA_UART_STATUS_TRDY_MSK)
			break;
	}
	writeb(ch, base + ALTERA_UART_TXDATA_REG);
}

static int putchar(int ch)
{
	uart_putc(ch);
	if (ch == '\n')
		uart_putc('\r');
	return ch;
}

static void console_init(void)
{
	unsigned base = (unsigned)na_uart0;
	unsigned int baud, baudclk;

	uart_base = ioremap(na_uart0, 32);
	baud = CONFIG_SERIAL_ALTERA_UART_BAUDRATE;
	baudclk = nasys_clock_freq / baud;
	writew(baudclk, base + ALTERA_UART_DIVISOR_REG);
}

#else

static int putchar(int ch)
{
	return ch;
}

static void console_init(void)
{
}

#endif

static int puts(const char *s)
{
	while (*s)
		putchar(*s++);
	return 0;
}
