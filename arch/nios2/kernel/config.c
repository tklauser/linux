

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

#if defined(CONFIG_MTD_PHYSMAP) || defined(CONFIG_MTD_PHYSMAP_MODULE)
static struct mtd_partition nios2_partitions[] = {
#ifdef CONFIG_ALTERA_STRATIX_II
	{
		.name =		"romfs/jffs2",
		.size =		0x600000,
		.offset =	0x200000,
	},{
		.name =		"loader/kernel",
		.size =		0x200000,
		.offset =	0,
	}, {
		.name =		"User configuration",
		.size =		0x400000,
		.offset =	0x800000,
	}, {
		.name =		"safe configuration",
		.size =		0x400000,
		.offset =	0xc00000,
		.mask_flags =	MTD_WRITEABLE,  /* force read-only */
	}
#elif defined(CONFIG_ALTERA_STRATIX_PRO)
	{
		.name =		"romfs/jffs2",
		.size =		0x200000,
		.offset =	0x200000,
	},{
		.name =		"loader/kernel",
		.size =		0x200000,
		.offset =	0,
	}, {
		.name =		"User configuration",
		.size =		0x200000,
		.offset =	0x400000,
	}, {
		.name =		"safe configuration",
		.size =		0x200000,
		.offset =	0x600000,
		.mask_flags =	MTD_WRITEABLE,  /* force read-only */
	}
#elif defined(CONFIG_ALTERA_DE2)
	{
		.name =		"romfs/jffs2",
		.size =		0x200000,
		.offset =	0x200000,
	},{
		.name =		"loader/kernel",
		.size =		0x200000,
		.offset =	0,
	}
#elif defined(CONFIG_ALTERA_NEEK_C3)
	{
		.name =		"romfs/jffs2",
		.size =		0x300000,
		.offset =	0xd00000,
	},{
		.name =		"catalog",
		.size =		0x020000,
		.offset =	0,
	}, {
		.name =		"application",
		.size =		0xb80000,
		.offset =	0x180000,
	}, {
		.name =		"selector",
		.size =		0x160000,
		.offset =	0x020000,
		.mask_flags =	MTD_WRITEABLE,  /* force read-only */
	}
#else
	{
		.name =		"romfs/jffs2",
		.size =		0x400000,
		.offset =	0x200000,
	},{
		.name =		"loader/kernel",
		.size =		0x200000,
		.offset =	0,
	}, {
		.name =		"User configuration",
		.size =		0x100000,
		.offset =	0x600000,
	}, {
		.name =		"safe configuration",
		.size =		0x100000,
		.offset =	0x700000,
		.mask_flags =	MTD_WRITEABLE,  /* force read-only */
	}
#endif
};

static struct physmap_flash_data nios2_flash_data = {
#if defined(CONFIG_ALTERA_NEEK_C3)
	.width      = 2,
#else
	.width      = 1,
#endif
	.parts      = nios2_partitions,
	.nr_parts   = ARRAY_SIZE(nios2_partitions),
};

static struct resource nios2_flash_resource = {
	.start = na_cfi_flash_0,
	.end   = na_cfi_flash_0_end - 1, 
	.flags = IORESOURCE_MEM,
};

static struct platform_device nios2_flash_device = {
	.name          = "physmap-flash",
	.id            = 0,
	.dev = {
		.platform_data = &nios2_flash_data,
	},
	.num_resources = 1,
	.resource      = &nios2_flash_resource,
};
#endif

#if (defined(CONFIG_SPI_ALTERA)  || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(na_epcs_controller)
#define EPCS_SPI_OFFSET 0x200 /* FIXME */
static struct resource na_epcs_controller_resource[] = {
	[0] = {
		.start = na_epcs_controller + EPCS_SPI_OFFSET,
		.end   = na_epcs_controller + EPCS_SPI_OFFSET + 31,
		.flags = IORESOURCE_MEM,
		},
	[1] = {
		.start = na_epcs_controller_irq,
		.end   = na_epcs_controller_irq,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device na_epcs_controller_device = {
	.name = "altspi",
	.id = 0, /* Bus number */
	.num_resources = ARRAY_SIZE(na_epcs_controller_resource),
	.resource = na_epcs_controller_resource,
};
#endif  /* spi master and devices */

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition nios2_spi_flash_partitions[] = {
	{
		.name =		"romfs/jffs2",
		.size =		0x180000,
		.offset =	0x80000,
	},{
		.name =		"fpga configuration",
		.size =		0x80000,
		.offset =	0,
	}
};

static struct flash_platform_data nios2_spi_flash_data = {
	.name = "m25p80",
	.parts = nios2_spi_flash_partitions,
	.nr_parts = ARRAY_SIZE(nios2_spi_flash_partitions),
	.type = "m25p16",
};
#endif

#if (defined(CONFIG_SPI_ALTERA)  || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(na_touch_panel_spi)
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
	.id = 1, /* Bus number */
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


#if defined(CONFIG_SPI_ALTERA) || defined(CONFIG_SPI_ALTERA_MODULE)
static struct spi_board_info nios2_spi_devices[] = {

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "m25p80", /* Name of spi_driver for this device */
		.max_speed_hz = 25000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 0, /* bus number */
		.chip_select = 0,
		.platform_data = &nios2_spi_flash_data,
	},
#endif

#if defined(CONFIG_TOUCHSCREEN_ADS7846) || defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
	{
		.modalias	= "ads7846",
		.chip_select	= 0,
		.max_speed_hz	= 125000 * 26,	/* (max sample rate @ 3V) * (cmd + data + overhead) */
		.bus_num	= 1, /* must match spi host bus number of touch panel spi  */
		.platform_data	= &ads_info,
		.irq		= na_touch_panel_pen_irq_n_irq,
	},
#endif
};
#endif


/*
 *	Nios2 platform devices
 */

static struct platform_device *nios2_devices[] __initdata = {
	&nios2_jtaguart,

	&nios2_uart,

#if defined(CONFIG_MTD_PHYSMAP) || defined(CONFIG_MTD_PHYSMAP_MODULE)
	&nios2_flash_device,
#endif

#if (defined(CONFIG_SPI_ALTERA)  || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(na_epcs_controller)
	&na_epcs_controller_device,
#endif

#if (defined(CONFIG_SPI_ALTERA)  || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(na_touch_panel_spi)
	&na_touch_panel_spi_device,
#endif
};

static int __init init_BSP(void)
{
	platform_add_devices(nios2_devices, ARRAY_SIZE(nios2_devices));

#if defined(CONFIG_SPI_ALTERA) || defined(CONFIG_SPI_ALTERA_MODULE)

#if defined(CONFIG_TOUCHSCREEN_ADS7846) || defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
	ads7843_pendown_init();
#endif

	spi_register_board_info(nios2_spi_devices,
				ARRAY_SIZE(nios2_spi_devices));
#endif
	return 0;
}

arch_initcall(init_BSP);
