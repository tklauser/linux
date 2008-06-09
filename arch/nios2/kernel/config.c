

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/spi/ads7846.h>
#include <linux/fb.h>
#if defined(CONFIG_USB_ISP1362_HCD) || defined(CONFIG_USB_ISP1362_HCD_MODULE)
#include <linux/usb/isp1362.h>
#endif
#include <linux/ata_platform.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/altjuart.h>
#include <linux/altuart.h>

/*
 *	Altera JTAG UART
 */

static struct altera_jtaguart_platform_uart nios2_jtaguart_platform[] = {
#ifdef na_jtag_uart
	{
	 .mapbase = (unsigned long)na_jtag_uart,
	 .irq = na_jtag_uart_irq,
	 },
#endif
	{},
};

static struct platform_device nios2_jtaguart = {
	.name = "altera_jtaguart",
	.id = 0,
	.dev.platform_data = nios2_jtaguart_platform,
};

/*
 *	Altera UART
 */

static struct altera_uart_platform_uart nios2_uart_platform[] = {
#ifdef na_uart0
	{
	 .mapbase = (unsigned long)na_uart0,
	 .irq = na_uart0_irq,
	 .uartclk = nasys_clock_freq,
	 },
#endif
#ifdef na_uart1
	{
	 .mapbase = (unsigned long)na_uart1,
	 .irq = na_uart1_irq,
	 .uartclk = nasys_clock_freq,
	 },
#endif
#ifdef na_uart2
	{
	 .mapbase = (unsigned long)na_uart2,
	 .irq = na_uart2_irq,
	 .uartclk = nasys_clock_freq,
	 },
#endif
#ifdef na_uart3
	{
	 .mapbase = (unsigned long)na_uart3,
	 .irq = na_uart3_irq,
	 .uartclk = nasys_clock_freq,
	 },
#endif
	{},
};

static struct platform_device nios2_uart = {
	.name = "altera_uart",
	.id = 0,
	.dev.platform_data = nios2_uart_platform,
};

#if defined(CONFIG_SPI_ALTERA) && defined(na_touch_panel_spi)
static struct resource na_touch_panel_spi_resource[] = {
	[0] = {
		.start = na_touch_panel_spi,
		.end   = na_touch_panel_spi + 31,
		.flags = IORESOURCE_MEM,
		},
	[1] = {
		.start = na_touch_panel_spi_irq,
		.end   = na_touch_panel_spi_irq,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device na_touch_panel_spi_device = {
	.name = "altspi",
	.id = 0, /* Bus number */
	.num_resources = ARRAY_SIZE(na_touch_panel_spi_resource),
	.resource = na_touch_panel_spi_resource,
};
#endif  /* spi master and devices */

#if defined(CONFIG_TOUCHSCREEN_ADS7846) || defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)

#define ALTERA_PIO_IO_EXTENT      16
#define ALTERA_PIO_DATA           0
#define ALTERA_PIO_DIRECTION      4
#define ALTERA_PIO_IRQ_MASK       8
#define ALTERA_PIO_EDGE_CAP       12

static unsigned long ads7843_pendown_base;
static void ads7843_pendown_init(void)
{
	ads7843_pendown_base = ioremap(na_touch_panel_pen_irq_n, ALTERA_PIO_IO_EXTENT);
	writel(0, ads7843_pendown_base + ALTERA_PIO_EDGE_CAP); /* clear edge */
	writel(1, ads7843_pendown_base + ALTERA_PIO_IRQ_MASK); /* enable irq */
}

static int ads7843_pendown_state(void)
{
	unsigned d;
	d = readl(ads7843_pendown_base + ALTERA_PIO_DATA); /* read pen */
	writel(0, ads7843_pendown_base + ALTERA_PIO_EDGE_CAP); /* clear edge */
	return ~d & 1;	/* Touchscreen PENIRQ */
}

static struct ads7846_platform_data ads_info = {
	.model			= 7843,
	.x_min			= 150,
	.x_max			= 3830,
	.y_min			= 190,
	.y_max			= 3830,
	.vref_delay_usecs	= 100,
	.x_plate_ohms		= 450,
	.y_plate_ohms		= 250,
	.pressure_max		= 15000,
	.debounce_max		= 1,
	.debounce_rep		= 0,
	.debounce_tol		= (~0),
	.get_pendown_state	= ads7843_pendown_state,
};
#endif


static struct spi_board_info nios2_spi_devices[] = {
#if defined(CONFIG_TOUCHSCREEN_ADS7846) || defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
	{
		.modalias	= "ads7846",
		.chip_select	= 0,
		.max_speed_hz	= 125000 * 26,	/* (max sample rate @ 3V) * (cmd + data + overhead) */
		.bus_num	= 0,
		.platform_data	= &ads_info,
		.irq		= na_touch_panel_pen_irq_n_irq,
	},
#endif
};


/*
 *	Nios2 platform devices
 */

static struct platform_device *nios2_devices[] __initdata = {
	&nios2_jtaguart,

	&nios2_uart,

#if defined(CONFIG_SPI_ALTERA) && defined(na_touch_panel_spi)
	&na_touch_panel_spi_device,
#endif
};

static int __init init_BSP(void)
{
	platform_add_devices(nios2_devices, ARRAY_SIZE(nios2_devices));

#if defined(CONFIG_SPI_ALTERA)

#if defined(CONFIG_TOUCHSCREEN_ADS7846) || defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
	ads7843_pendown_init();
#endif

	spi_register_board_info(nios2_spi_devices,
				ARRAY_SIZE(nios2_spi_devices));
#endif
	return 0;
}

arch_initcall(init_BSP);
