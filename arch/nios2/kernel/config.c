#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <asm/avuart.h>

static struct avalon_uart_platform_uart nios2_uart_platform[] = {
#ifdef na_uart0
	{
		.mapbase	= na_uart0,
		.irq		= na_uart0_irq,
		.uartclk	= nasys_clock_freq,
	},
#endif
#ifdef na_uart1
	{
		.mapbase 	= na_uart1,
		.irq		= na_uart1_irq,
		.uartclk	= nasys_clock_freq,
	},
#endif
#ifdef na_uart2
	{
		.mapbase 	= na_uart2,
		.irq		= na_uart2_irq,
		.uartclk	= nasys_clock_freq,
	},
#endif
#ifdef na_uart3
	{
		.mapbase 	= na_uart3,
		.irq		= na_uart3_irq,
		.uartclk	= nasys_clock_freq,
	},
#endif
	{ },
};

static struct platform_device nios2_uart = {
	.name			= "avalon_uart",
	.id			= 0,
	.dev.platform_data	= nios2_uart_platform,
};

static struct platform_device *nios2_devices[] __initdata = {
	&nios2_uart,
};

static int __init init_BSP(void)
{
	platform_add_devices(nios2_devices, ARRAY_SIZE(nios2_devices));
	return 0;
}

arch_initcall(init_BSP);
