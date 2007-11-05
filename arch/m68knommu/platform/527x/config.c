/***************************************************************************/

/*
 *	linux/arch/m68knommu/platform/527x/config.c
 *
 *	Sub-architcture dependant initialization code for the Freescale
 *	5270/5271 CPUs.
 *
 *	Copyright (C) 1999-2004, Greg Ungerer (gerg@snapgear.com)
 *	Copyright (C) 2001-2004, SnapGear Inc. (www.snapgear.com)
 */

/***************************************************************************/

#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/interrupt.h>
#include <asm/machdep.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/coldfire.h>
#include <asm/mcfsim.h>
#include <asm/mcfuart.h>

/***************************************************************************/

void coldfire_reset(void);

/***************************************************************************/

/*
 *	DMA channel base address table.
 */
unsigned int   dma_base_addr[MAX_M68K_DMA_CHANNELS] = {
        MCF_MBAR + MCFDMA_BASE0,
};

unsigned int dma_device_address[MAX_M68K_DMA_CHANNELS];

/***************************************************************************/

static struct mcf_platform_uart m527x_uart_platform[] = {
	{
		.mapbase	= MCF_MBAR + MCFUART_BASE1,
		.irq		= MCFINT_VECBASE + MCFINT_UART0,
	},
	{
		.mapbase 	= MCF_MBAR + MCFUART_BASE2,
		.irq		= MCFINT_VECBASE + MCFINT_UART1,
	},
	{
		.mapbase 	= MCF_MBAR + MCFUART_BASE3,
		.irq		= MCFINT_VECBASE + MCFINT_UART2,
	},
	{ },
};

static struct platform_device m527x_uart = {
	.name			= "mcfuart",
	.id			= 0,
	.dev.platform_data	= m527x_uart_platform,
};

static struct platform_device *m527x_devices[] __initdata = {
	&m527x_uart,
};

/***************************************************************************/

#define	INTC0	(MCF_MBAR + MCFICM_INTC0)

static void m527x_uart_init_line(int line, int irq)
{
	u16 sepmask;
	u32 imr;

	if ((line < 0) || (line > 2))
		return;

	/* level 6, line based priority */
	writeb(0x30+line, INTC0 + MCFINTC_ICR0 + MCFINT_UART0 + line);

	imr = readl(INTC0 + MCFINTC_IMRL);
	imr &= ~((1 << (irq - MCFINT_VECBASE)) | 1);
	writel(imr, INTC0 + MCFINTC_IMRL);

	/*
	 * External Pin Mask Setting & Enable External Pin for Interface
       	 */
	sepmask = readw(MCF_IPSBAR + MCF_GPIO_PAR_UART);
	if (line == 0)
		sepmask |= UART0_ENABLE_MASK;
	else if (line == 1)
		sepmask |= UART1_ENABLE_MASK;
	else if (line == 2)
		sepmask |= UART2_ENABLE_MASK;
	writew(sepmask, MCF_IPSBAR + MCF_GPIO_PAR_UART);
}

void m527x_uarts_init(void)
{
	const int nrlines = ARRAY_SIZE(m527x_uart_platform);
	int line;

	for (line = 0; (line < nrlines); line++)
		m527x_uart_init_line(line, m527x_uart_platform[line].irq);
}

/***************************************************************************/

void mcf_disableall(void)
{
	*((volatile unsigned long *) (MCF_IPSBAR + MCFICM_INTC0 + MCFINTC_IMRH)) = 0xffffffff;
	*((volatile unsigned long *) (MCF_IPSBAR + MCFICM_INTC0 + MCFINTC_IMRL)) = 0xffffffff;
}

/***************************************************************************/

void mcf_autovector(unsigned int vec)
{
	/* Everything is auto-vectored on the 5272 */
}

/***************************************************************************/

void config_BSP(char *commandp, int size)
{
	mcf_disableall();
	mach_reset = coldfire_reset;
}

/***************************************************************************/

static int __init init_BSP(void)
{
	m527x_uarts_init();
	platform_add_devices(m527x_devices, ARRAY_SIZE(m527x_devices));
	return 0;
}

arch_initcall(init_BSP);

/***************************************************************************/
