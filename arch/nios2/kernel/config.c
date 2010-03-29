
#include <linux/types.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
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
#include <linux/i2c-ocores.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/altjuart.h>
#include <linux/altuart.h>
#include <linux/nios_mmc.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/gpio_mouse.h>

/*
 *	Altera JTAG UART
 */

static struct altera_jtaguart_platform_uart nios2_jtaguart_platform[] = {
#ifdef JTAG_UART_BASE
	{
	 .mapbase = JTAG_UART_BASE,
	 .irq = JTAG_UART_IRQ,
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
#ifdef UART0_BASE
	{
	 .mapbase = UART0_BASE,
	 .irq = UART0_IRQ,
	 .uartclk = UART0_FREQ,
	 },
#endif
#ifdef UART1_BASE
	{
	 .mapbase = UART1_BASE,
	 .irq = UART1_IRQ,
	 .uartclk = UART1_FREQ,
	 },
#endif
#ifdef UART2_BASE
	{
	 .mapbase = UART2_BASE,
	 .irq = UART2_IRQ,
	 .uartclk = UART2_FREQ,
	 },
#endif
#ifdef UART3_BASE
	{
	 .mapbase = UART3_BASE,
	 .irq = UART3_IRQ,
	 .uartclk = UART3_FREQ,
	 },
#endif
	{},
};

static struct platform_device nios2_uart = {
	.name = "altera_uart",
	.id = 0,
	.dev.platform_data = nios2_uart_platform,
};

/*
 *	openip gpio
 */

#ifdef CONFIG_GENERIC_GPIO
resource_size_t nios2_gpio_mapbase;

static void nios2_gpio_init(void)
{
	nios2_gpio_mapbase = (resource_size_t)ioremap(GPIO_0_BASE, 32);
#if defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
	gpio_direction_output(6,1); /* output only I2C SCLK on NEEK */
	gpio_direction_output(0,1);
	gpio_direction_output(4,1);
	gpio_direction_output(3,0);
#endif
}
#else
static void nios2_gpio_init(void) {}
#endif /* CONFIG_GENERIC_GPIO */

#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
static struct gpio_led nios2_led_pins[] = {
	{
		.name		= "led0",
		.default_trigger = "heartbeat", /* optional */
		.gpio		= 2, /* FIXME: gpio pin assignment */
		.active_low	= 1,
	},
};

static struct gpio_led_platform_data nios2_led_data = {
	.num_leds		= ARRAY_SIZE(nios2_led_pins),
	.leds			= nios2_led_pins,
};

static struct platform_device gpio_leds_device = {
	.name			= "leds-gpio",
	.id			= -1,
	.dev.platform_data	= &nios2_led_data,
};
#endif

#if defined(CONFIG_MOUSE_GPIO) || defined(CONFIG_MOUSE_GPIO_MODULE)
static struct gpio_mouse_platform_data nios2_gpio_mouse_data = {
	.polarity	= GPIO_MOUSE_POLARITY_ACT_LOW,
	{
		{
			.up		= 8, /* FIXME: gpio pin assignment */
			.down		= 9,
			.left		= 10,
			.right		= 11,
			.bleft		= 12,
			.bmiddle	= 13,
			.bright		= 14,
		},
	},
	.scan_ms	= 10,
};

static struct platform_device gpio_mouse_device = {
	.name		= "gpio_mouse",
	.id		= 0,
	.dev		= {
		.platform_data = &nios2_gpio_mouse_data,
	},
};
#endif

/*
 *	MTD map, CFI flash, NAND flash, SPI/EPCS flash
 */

#if defined(CONFIG_MTD_PHYSMAP) || defined(CONFIG_MTD_PHYSMAP_MODULE)
static struct mtd_partition nios2_partitions[] = {
#if defined(CONFIG_ALTERA_STRATIX_II) || defined(CONFIG_ALTERA_CYCLONE_II)
	{
	 .name = "romfs/jffs2",
	 .size = 0x600000,
	 .offset = 0x200000,
	 },
	{
	 .name = "loader/kernel",
	 .size = 0x200000,
	 .offset = 0,
	 },
	{
	 .name = "User configuration",
	 .size = 0x400000,
	 .offset = 0x800000,
	 },
	{
	 .name = "safe configuration",
	 .size = 0x400000,
	 .offset = 0xc00000,
	 .mask_flags = MTD_WRITEABLE,	/* force read-only */
	 }
#elif defined(CONFIG_ALTERA_STRATIX_PRO)
	{
	 .name = "romfs/jffs2",
	 .size = 0x200000,
	 .offset = 0x200000,
	 },
	{
	 .name = "loader/kernel",
	 .size = 0x200000,
	 .offset = 0,
	 },
	{
	 .name = "User configuration",
	 .size = 0x200000,
	 .offset = 0x400000,
	 },
	{
	 .name = "safe configuration",
	 .size = 0x200000,
	 .offset = 0x600000,
	 .mask_flags = MTD_WRITEABLE,	/* force read-only */
	 }
#elif defined(CONFIG_ALTERA_DE2)
	{
	 .name = "romfs/jffs2",
	 .size = 0x200000,
	 .offset = 0x200000,
	 },
	{
	 .name = "loader/kernel",
	 .size = 0x200000,
	 .offset = 0,
	 }
#elif defined(CONFIG_ALTERA_NEEK_C3)
	{
	 .name = "romfs/jffs2",
	 .size = 0x300000,
	 .offset = 0xd00000,
	 },
	{
	 .name = "catalog",
	 .size = 0x020000,
	 .offset = 0,
	 },
	{
	 .name = "application",
	 .size = 0xb80000,
	 .offset = 0x180000,
	 },
	{
	 .name = "selector",
	 .size = 0x160000,
	 .offset = 0x020000,
	 .mask_flags = MTD_WRITEABLE,	/* force read-only */
	 }
#elif defined(CONFIG_ALTERA_CYCLONE_III)
	{
	 .name = "userspace",
	 .size =   0x02800000, /* 40Mb */
	 .offset = 0x00000000,
	},
	{
	 .name = "U-Boot",
	 .size =   0x00100000, 
	 .offset = 0x02800000,
	},
	{
	 .name = "uImage1",
	 .size =   0x00400000, 
	 .offset = 0x02900000,
	},
	{
	 .name = "uImage2",
	 .size =   0x00400000, 
	 .offset = 0x02d00000,
	},
	{
	 .name = "uImage3",
	 .size =   0x00400000, 
	 .offset = 0x03100000,
	},
	
	{
	 .name = "DEFAULT_MMU",
	 .size =   0x00380000,
	 .offset = 0x03500000,
	 },
	{
	 .name = "MAXIMUM_MMU",
	 .size =   0x00380000,
	 .offset = 0x03880000,
	 },
	{
	 .name = "USER_IMAGE",
	 .size =   0x00380000,
	 .offset = 0x03c00000,
	 },
	{
	  .name = "options-bits",
	  .size =   0x00020000,
	  .offset = 0x03f80000,
	}
#else
	{
	 .name = "romfs/jffs2",
	 .size = 0x400000,
	 .offset = 0x200000,
	 },
	{
	 .name = "loader/kernel",
	 .size = 0x200000,
	 .offset = 0,
	 },
	{
	 .name = "User configuration",
	 .size = 0x100000,
	 .offset = 0x600000,
	 },
	{
	 .name = "safe configuration",
	 .size = 0x100000,
	 .offset = 0x700000,
	 .mask_flags = MTD_WRITEABLE,	/* force read-only */
	 }
#endif
};

static struct physmap_flash_data nios2_flash_data = {
#if defined(CONFIG_ALTERA_NEEK_C3) || defined(CONFIG_ALTERA_CYCLONE_III)
	.width = 2,		/* 16 bits data bus */
#else
	.width = 1,		/* 8 bits data bus */
#endif
	.parts = nios2_partitions,
	.nr_parts = ARRAY_SIZE(nios2_partitions),
};

static struct resource nios2_flash_resource = {
#if defined(CFI_FLASH_0_BASE)
	.start = CFI_FLASH_0_BASE,
	.end = CFI_FLASH_0_BASE + CFI_FLASH_0_SPAN - 1,
#else
	.start = EXT_FLASH_BASE,
	.end = EXT_FLASH_BASE + EXT_FLASH_SPAN - 1,
#endif
	.flags = IORESOURCE_MEM,
};

static struct platform_device cfi_flash_device = {
	.name = "physmap-flash",
	.id = 0,
	.dev = {
		.platform_data = &nios2_flash_data,
		},
	.num_resources = 1,
	.resource = &nios2_flash_resource,
};
#endif

#if defined(CONFIG_MTD_NAND_PLATFORM) || defined(CONFIG_MTD_NAND_PLATFORM_MODULE)
#ifdef CONFIG_MTD_PARTITIONS
static struct mtd_partition nios2_plat_nand_partitions[] = {
	{
		.name   = "linux kernel(nand)",
		.size   = 0x400000,
		.offset = 0,
	}, {
		.name   = "file system(nand)",
		.size   = MTDPART_SIZ_FULL,
		.offset = MTDPART_OFS_APPEND,
	},
};
#endif

#define NIOS2_NAND_PLAT_CLE 2
#define NIOS2_NAND_PLAT_ALE 3
static void nios2_plat_nand_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *this = mtd->priv;

	if (cmd == NAND_CMD_NONE)
		return;

	if (ctrl & NAND_CLE)
		writeb(cmd, this->IO_ADDR_W + (1 << NIOS2_NAND_PLAT_CLE));
	else
		writeb(cmd, this->IO_ADDR_W + (1 << NIOS2_NAND_PLAT_ALE));
}

#undef NIOS2_NAND_PLAT_READY    /* FIXME: define gpio pin assignment to R/B NAND */
#ifdef NIOS2_NAND_PLAT_READY 
static int nios2_plat_nand_dev_ready(struct mtd_info *mtd)
{
	return gpio_get_value(NIOS2_NAND_PLAT_READY);
}
#endif

static struct platform_nand_data nios2_plat_nand_data = {
	.chip = {
		.chip_delay = 30, /* FIXME: tR of your nand chip */
#ifdef CONFIG_MTD_PARTITIONS
		.partitions = nios2_plat_nand_partitions,
		.nr_partitions = ARRAY_SIZE(nios2_plat_nand_partitions),
#endif
	},
	.ctrl = {
		.cmd_ctrl  = nios2_plat_nand_cmd_ctrl,
#ifdef NIOS2_NAND_PLAT_READY
		.dev_ready = nios2_plat_nand_dev_ready,
#endif
	},
};

static struct resource nios2_plat_nand_resources = {
	.start = NAND_FLASH_0_BASE,
	.end   = NAND_FLASH_0_BASE + 12,
	.flags = IORESOURCE_MEM,
};

static struct platform_device async_nand_device = {
	.name = "gen_nand",
	.id = -1,
	.num_resources = 1,
	.resource = &nios2_plat_nand_resources,
	.dev = {
		.platform_data = &nios2_plat_nand_data,
	},
};

static void nios2_plat_nand_init(void)
{
#ifdef NIOS2_NAND_PLAT_READY
	gpio_request(NIOS2_NAND_PLAT_READY, "nios2_nand_plat");
#endif
}
#else
static void nios2_plat_nand_init(void) {}
#endif

#if (defined(CONFIG_SPI_ALTERA)  || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(EPCS_CONTROLLER_BASE)
#define EPCS_SPI_OFFSET 0x200	/* FIXME */
static struct resource epcs_controller_resource[] = {
	[0] = {
	       .start = EPCS_CONTROLLER_BASE + EPCS_SPI_OFFSET,
	       .end = EPCS_CONTROLLER_BASE + EPCS_SPI_OFFSET + 31,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = EPCS_CONTROLLER_IRQ,
	       .end = EPCS_CONTROLLER_IRQ,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct platform_device epcs_controller_device = {
	.name = "altspi",
	.id = 0,		/* Bus number */
	.num_resources = ARRAY_SIZE(epcs_controller_resource),
	.resource = epcs_controller_resource,
};
#endif /* spi master and devices */

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition nios2_spi_flash_partitions[] = {
#if defined(CONFIG_ALTERA_STRATIX_II) || defined(CONFIG_ALTERA_CYCLONE_II)
	{
	 .name = "romfs/jffs2",
	 .size = 0x400000,
	 .offset = 0x400000,
	 },
	{
	 .name = "fpga configuration",
	 .size = 0x400000,
	 .offset = 0,
	 }
#else
	{
	 .name = "romfs/jffs2",
	 .size = 0x180000,
	 .offset = 0x80000,
	 },
	{
	 .name = "fpga configuration",
	 .size = 0x80000,
	 .offset = 0,
	 }
#endif
};

static struct flash_platform_data nios2_spi_flash_data = {
	.name = "m25p80",
	.parts = nios2_spi_flash_partitions,
	.nr_parts = ARRAY_SIZE(nios2_spi_flash_partitions),
#if defined(CONFIG_ALTERA_STRATIX_II) || defined(CONFIG_ALTERA_CYCLONE_II)
	.type = "m25p64",	/* depend on the actual size of spi flash */
#else
	.type = "m25p16",	/* depend on the actual size of spi flash */
#endif
};
#endif

/*
 *	Altera SPI, MMC
 */

#if (defined(CONFIG_SPI_ALTERA)  || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(TOUCH_PANEL_SPI_BASE)
static struct resource touch_panel_spi_resource[] = {
	[0] = {
	       .start = TOUCH_PANEL_SPI_BASE,
	       .end = TOUCH_PANEL_SPI_BASE + 31,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = TOUCH_PANEL_SPI_IRQ,
	       .end = TOUCH_PANEL_SPI_IRQ,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct platform_device touch_panel_spi_device = {
	.name = "altspi",
	.id = 1,		/* Bus number */
	.num_resources = ARRAY_SIZE(touch_panel_spi_resource),
	.resource = touch_panel_spi_resource,
};
#endif /* spi master and devices */

#if (defined(CONFIG_TOUCHSCREEN_ADS7846) || defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)) && \
	defined(TOUCH_PANEL_PEN_IRQ_N_BASE)

#define ALTERA_PIO_IO_EXTENT      16
#define ALTERA_PIO_DATA           0
#define ALTERA_PIO_DIRECTION      4
#define ALTERA_PIO_IRQ_MASK       8
#define ALTERA_PIO_EDGE_CAP       12

static unsigned long ads7843_pendown_base;
static void ads7843_pendown_init(void)
{
	ads7843_pendown_base =
	    ioremap(TOUCH_PANEL_PEN_IRQ_N_BASE, ALTERA_PIO_IO_EXTENT);
	writel(0, ads7843_pendown_base + ALTERA_PIO_EDGE_CAP);	/* clear edge */
	writel(1, ads7843_pendown_base + ALTERA_PIO_IRQ_MASK);	/* enable irq */
}

static int ads7843_pendown_state(void)
{
	unsigned d;
	d = readl(ads7843_pendown_base + ALTERA_PIO_DATA);	/* read pen */
	writel(0, ads7843_pendown_base + ALTERA_PIO_EDGE_CAP);	/* clear edge */
	return ~d & 1;		/* Touchscreen PENIRQ */
}

static struct ads7846_platform_data ads_info = {
	.model = 7843,
	.x_min = 150,
	.x_max = 3830,
	.y_min = 190,
	.y_max = 3830,
	.vref_delay_usecs = 100,
	.x_plate_ohms = 450,
	.y_plate_ohms = 250,
	.pressure_max = 15000,
	.debounce_max = 1,
	.debounce_rep = 0,
	.debounce_tol = (~0),
	.get_pendown_state = ads7843_pendown_state,
};
#endif

#if (defined(CONFIG_SPI_ALTERA)  || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(MMC_SPI_BASE)
static struct resource mmc_spi_resource[] = {
	[0] = {
	       .start = MMC_SPI_BASE,
	       .end = MMC_SPI_BASE + MMC_SPI_SPAN - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MMC_SPI_IRQ,
	       .end = MMC_SPI_IRQ,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct platform_device mmc_spi_device = {
	.name = "altspi",
	.id = 2,		/* Bus number */
	.num_resources = ARRAY_SIZE(mmc_spi_resource),
	.resource = mmc_spi_resource,
};
#endif /* spi master and devices */

#if defined(CONFIG_SPI_GPIO) || defined(CONFIG_SPI_GPIO_MODULE)
#include <linux/spi/spi_gpio.h>

struct spi_gpio_platform_data nios2_spi_gpio_3_platform_data = {
	.sck = 8, /* FIXME: gpio pin assignment */
	.mosi = 9,
	.miso = 10,
	.num_chipselect = 1,
};

static struct platform_device nios2_spi_gpio_3_device = {
	.name		= "spi_gpio",
	.id = 3,		/* Bus number */
	.dev	= {
		.platform_data	= &nios2_spi_gpio_3_platform_data,
	},
};
#endif

#if defined(CONFIG_SPI) || defined(CONFIG_SPI_MODULE)
static struct spi_board_info nios2_spi_devices[] = {

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
	{
	 /* the modalias must be the same as spi device driver name */
	 .modalias = "m25p80",	/* Name of spi_driver for this device */
	 .max_speed_hz = 25000000,	/* max spi clock (SCK) speed in HZ */
	 .bus_num = 0,		/* bus number */
	 .chip_select = 0,
	 .platform_data = &nios2_spi_flash_data,
	 },
#endif

#if (defined(CONFIG_TOUCHSCREEN_ADS7846) || defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)) && \
	defined(TOUCH_PANEL_PEN_IRQ_N_BASE)
	{
	 .modalias = "ads7846",
	 .chip_select = 0,
	 .max_speed_hz = 125000 * 26,	/* (max sample rate @ 3V) * (cmd + data + overhead) */
	 .bus_num = 1,		/* must match spi host bus number of touch panel spi  */
	 .platform_data = &ads_info,
	 .irq = TOUCH_PANEL_PEN_IRQ_N_IRQ
	 },
#endif

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	{
	 .modalias = "mmc_spi",
	 .max_speed_hz = 25000000,
	 .bus_num = 2,		/* must match spi host bus number of mmc spi  */
	 .chip_select = 0,
	 },
#endif

#if defined(CONFIG_EEPROM_AT25) || defined(CONFIG_EEPROM_AT25_MODULE)
	{
	 .modalias = "at25",
	 .max_speed_hz = 25000000,
	 .bus_num = 3,		/* must match spi host bus*/
	 .chip_select = 0,
	 .controller_data = (void *) 11, /* FIXME: gpio pin assignment of CS */
	 },
#endif

};
#endif

/*
 *	PFS Tech SD/SDIO/MMC Host
 */

/* Map sdio_host to sdio if it exists */
#if defined(SDIO_HOST_BASE)
#define SDIO_BASE SDIO_HOST_BASE
#define SDIO_IRQ SDIO_HOST_IRQ
#define SDIO_FREQ SDIO_HOST_FREQ
#endif

#if (defined(CONFIG_MMC_NIOS) || defined(CONFIG_MMC_NIOS_MODULE)) && defined(SDIO_BASE)

static struct nios_mmc_platform_mmc nios2_mmc_platform[] = {
	{
	 .mapbase = SDIO_BASE,
	 .irq = SDIO_IRQ,
	 .clk_src = SDIO_FREQ,
	 },
};

static struct resource nios_mmc_resources[] = {
	[0] = {
		.start = SDIO_BASE,
		.end = SDIO_BASE + (16*4-1),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = SDIO_IRQ,
		.end = SDIO_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};
static struct platform_device nios_mmc_device = {
	.name = "nios_mmc",
	.id = 0,
	.num_resources = ARRAY_SIZE(nios_mmc_resources),
	.resource = nios_mmc_resources,
	.dev.platform_data = nios2_mmc_platform,
};

#endif

/*
 *	Altera CF IDE
 */

#if defined(CF_IDE_BASE)
/* Use PATA_ALTERA_CF in preference to PATA_PLATFORM */
#if defined(CONFIG_PATA_ALTERA_CF) || defined(CONFIG_PATA_ALTERA_CF_MODULE)
#define USE_PATA_ALTERA_CF
#elif defined(CONFIG_PATA_PLATFORM) || defined(CONFIG_PATA_PLATFORM_MODULE)
#define USE_PATA_PLATFORM
#endif
#endif

#ifdef USE_PATA_ALTERA_CF
static struct resource cf_resources[] = {
	{
	 .start = CF_IDE_BASE,
	 .end = CF_IDE_BASE + 63,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = CF_IDE_IRQ,
	 .end = CF_IDE_IRQ,
	 .flags = IORESOURCE_IRQ,
	 },
	{
	 .start = CF_CTL_BASE,
	 .end = CF_CTL_BASE + 15,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = CF_CTL_IRQ,
	 .end = CF_CTL_IRQ,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct platform_device cf_device = {
	.name = "pata_altera_cf",
	.id = -1,
	.num_resources = ARRAY_SIZE(cf_resources),
	.resource = cf_resources,
};
#endif

#ifdef USE_PATA_PLATFORM
static struct pata_platform_info cf_platform_data = {
	.ioport_shift = 2,
};

static struct resource cf_resources[] = {
	{
	 .start = CF_IDE_BASE,
	 .end = CF_IDE_BASE + 31,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = CF_IDE_BASE + 56,
	 .end = CF_IDE_BASE + 56 + 3,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = CF_IDE_IRQ,
	 .end = CF_IDE_IRQ,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct platform_device cf_device = {
	.name = "pata_platform",
	.id = -1,
	.num_resources = ARRAY_SIZE(cf_resources),
	.resource = cf_resources,
	.dev = {
		.platform_data = &cf_platform_data,
		}
};

#define ALTERA_CF_CTL_STATUS            	0
#define ALTERA_CF_IDE_CTL               	4
#define ALTERA_CF_CTL_STATUS_PRESENT_MSK       (0x1)
#define ALTERA_CF_CTL_STATUS_POWER_MSK         (0x2)
#define ALTERA_CF_CTL_STATUS_RESET_MSK         (0x4)
#define ALTERA_CF_CTL_STATUS_IRQ_EN_MSK        (0x8)
#define ALTERA_CF_IDE_CTL_IRQ_EN_MSK           (0x1)

static void __init cf_init(unsigned ctl_base)
{
	unsigned ctl = ioremap(ctl_base, 16);
	writel(0, ctl + ALTERA_CF_IDE_CTL);	/* disable ide irq */
	writel(ALTERA_CF_CTL_STATUS_RESET_MSK, ctl + ALTERA_CF_CTL_STATUS);	/* power down */
	msleep(500);		/* 0.5 sec delay */
	writel(ALTERA_CF_CTL_STATUS_POWER_MSK, ctl + ALTERA_CF_CTL_STATUS);	/* power up */
	msleep(500);		/* 0.5 sec delay */
	writel(ALTERA_CF_IDE_CTL_IRQ_EN_MSK, ctl + ALTERA_CF_IDE_CTL);	/* enable ide irq */
}
#endif

/*
 *	Opencore I2C , gpio I2C
 */

/* if you have multiple i2c buses, you may have to register different board info for each of them */
static struct i2c_board_info __initdata nios2_i2c_0_board_info[] = {
#if defined(CONFIG_RTC_DRV_DS1307) || defined(CONFIG_RTC_DRV_DS1307_MODULE)
	{
		I2C_BOARD_INFO("ds1307", 0x68),
	},
#endif
};

static struct i2c_board_info __initdata nios2_i2c_1_board_info[] = {
#if defined(CONFIG_EEPROM_AT24) || defined(CONFIG_EEPROM_AT24_MODULE)
	{
		I2C_BOARD_INFO("24c00", 0x50), /* microchip 24lc00 */
	},
#endif
};

#if (defined(CONFIG_I2C_OCORES) || defined(CONFIG_I2C_OCORES_MODULE)) && defined(I2C_0_BASE)
static struct resource i2c_oc_0_resources[] = {
	[0] = {
	       .start = I2C_0_BASE,
	       .end = I2C_0_BASE + 31,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = I2C_0_IRQ,
	       .end = I2C_0_IRQ,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct ocores_i2c_platform_data i2c_oc_0_platform_data = {
	.regstep = 4,		/* four bytes between registers */
	.clock_khz = I2C_0_FREQ / 1000,	/* input clock */
};

static struct platform_device i2c_oc_0_device = {
	.name = "ocores-i2c",
	.id = 0,
	.dev = {
		.platform_data = &i2c_oc_0_platform_data,
		},
	.num_resources = ARRAY_SIZE(i2c_oc_0_resources),
	.resource = i2c_oc_0_resources,
};
#endif

#if (defined(CONFIG_I2C_OCORES) || defined(CONFIG_I2C_OCORES_MODULE)) && defined(I2C_1_BASE)
static struct resource i2c_oc_1_resources[] = {
	[0] = {
	       .start = I2C_1_BASE,
	       .end = I2C_1_BASE + 31,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = I2C_1_IRQ,
	       .end = I2C_1_IRQ,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct ocores_i2c_platform_data i2c_oc_1_platform_data = {
	.regstep = 4,		/* four bytes between registers */
	.clock_khz = I2C_1_FREQ / 1000,	/* input clock */
};

static struct platform_device i2c_oc_1_device = {
	.name = "ocores-i2c",
	.id = 1,
	.dev = {
		.platform_data = &i2c_oc_1_platform_data,
		},
	.num_resources = ARRAY_SIZE(i2c_oc_1_resources),
	.resource = i2c_oc_1_resources,
};
#endif

#if defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
#include <linux/i2c-gpio.h>

static struct i2c_gpio_platform_data i2c_gpio_0_data = {
	.sda_pin		= 7, /* FIXME: gpio pin assignment */
	.scl_pin		= 6, /* FIXME: gpio pin assignment */
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 1,
	.udelay			= 40,
};

static struct platform_device i2c_gpio_0_device = {
	.name		= "i2c-gpio",
	.id		= 0,
	.dev		= {
		.platform_data	= &i2c_gpio_0_data,
	},
};

static struct i2c_gpio_platform_data i2c_gpio_1_data = {
	.sda_pin		= 1, /* FIXME: gpio pin assignment */
	.scl_pin		= 0, /* FIXME: gpio pin assignment */
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 1,
	.udelay			= 40,
};

static struct platform_device i2c_gpio_1_device = {
	.name		= "i2c-gpio",
	.id		= 1,
	.dev		= {
		.platform_data	= &i2c_gpio_1_data,
	},
};

static struct i2c_gpio_platform_data i2c_gpio_2_data = {
	.sda_pin		= 5, /* FIXME: gpio pin assignment */
	.scl_pin		= 4, /* FIXME: gpio pin assignment */
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 1,
	.udelay			= 40,
};

static struct platform_device i2c_gpio_2_device = {
	.name		= "i2c-gpio",
	.id		= 2,
	.dev		= {
		.platform_data	= &i2c_gpio_2_data,
	},
};
#endif

/*
 *	One wire
 */

#if defined(CONFIG_W1_MASTER_GPIO) || defined(CONFIG_W1_MASTER_GPIO_MODULE)
#include <linux/w1-gpio.h>

static struct w1_gpio_platform_data nios2_w1_gpio_0_data = {
	.pin		= 7, /* FIXME: gpio pin assignment */
	.is_open_drain	= 0,
};

static struct platform_device nios2_w1_gpio_0_device = {
	.name			= "w1-gpio",
	.id			= -1,
	.dev.platform_data	= &nios2_w1_gpio_0_data,
};
#endif

/*
 *	Altera PS2
 */

#if defined(CONFIG_SERIO_ALTPS2) && defined(PS2_0_BASE)
static struct resource altps2_0_resources[] = {
	[0] = {
		.start		= PS2_0_BASE,
		.end		= PS2_0_BASE + 0x8 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= PS2_0_IRQ,
		.end		= PS2_0_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};
static struct platform_device altps2_0_device = {
	.name		= "altps2",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(altps2_0_resources),
	.resource	= altps2_0_resources,
};

#if defined(PS2_1_BASE)
static struct resource altps2_1_resources[] = {
	[0] = {
		.start		= PS2_1_BASE,
		.end		= PS2_1_BASE + 0x8 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= PS2_1_IRQ,
		.end		= PS2_1_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};
static struct platform_device altps2_1_device = {
	.name		= "altps2",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(altps2_1_resources),
	.resource	= altps2_1_resources,
};
#endif
#endif

/*
 *	Altera Remote update
 */

#if defined(CONFIG_ALTERA_REMOTE_UPDATE) && defined(ALTREMOTE_BASE)
static struct resource altremote_resources[] = {
  [0] = {
    .start    = ALTREMOTE_BASE,
    .end    = ALTREMOTE_BASE + 0x200 - 1,
    .flags    = IORESOURCE_MEM,
  },
};
static struct platform_device altremote_device = {
  .name   = "altremote",
  .id   = 0,
  .num_resources  = ARRAY_SIZE(altremote_resources),
  .resource = altremote_resources,
};
#endif

/*
 *	Ethernet, Altera TSE
 */

#if defined(CONFIG_SMC91X) && defined(ENET_BASE)
#ifndef LAN91C111_REGISTERS_OFFSET
#define LAN91C111_REGISTERS_OFFSET 0x300
#endif

static struct resource smc91x_resources[] = {
	[0] = {
		.start		= ENET_BASE + LAN91C111_REGISTERS_OFFSET,
		.end		= ENET_BASE + LAN91C111_REGISTERS_OFFSET + 0x100 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= ENET_IRQ,
		.end		= ENET_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};
static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};
#endif

#if defined(CONFIG_DM9000) && defined(DM9000_BASE)
#include <linux/dm9000.h>
static struct resource dm9k_resource[] = {
	[0] = {
		.start = DM9000_BASE,
		.end   = DM9000_BASE + 3,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = DM9000_BASE + 4,
		.end   = DM9000_BASE + 4 + 3,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.start = DM9000_IRQ,
		.end   = DM9000_IRQ,
		.flags = IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}

};
static struct dm9000_plat_data dm9k_platdata = {
	.flags		= DM9000_PLATF_16BITONLY,
};
static struct platform_device dm9k_device = {
	.name		= "dm9000",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(dm9k_resource),
	.resource	= dm9k_resource,
	.dev		= {
		.platform_data = &dm9k_platdata,
	}
};
#endif

#if defined (CONFIG_ATSE)
/* Altera Triple Speed Ethernet */
#include         "../drivers/net/atse.h"
static struct resource atse_resource[] = {
	[0] = {
		.start = na_descriptor_memory_s1,
		.end   = na_descriptor_memory_s1 + 0x2000 - 1,   /* code from sopc file */
		.name  = ATSE_RESOURCE_NAME_STR_DESC_MEM,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = na_sgdma_rx_csr,
		.end   = na_sgdma_rx_csr + 0x400 - 1,  /* code from sopc file */
		.name  = ATSE_RESOURCE_NAME_STR_SGDMA_RX_MEM,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.start = na_sgdma_tx,
		.end   = na_sgdma_tx + 0x400 - 1,  /* code from sopc file */
		.name  = ATSE_RESOURCE_NAME_STR_SGDMA_TX_MEM,
		.flags = IORESOURCE_MEM,
	},
	[3] = {
		.start = na_sgdma_rx_csr_irq,
		.end   = na_sgdma_rx_csr_irq,
		.name  = ATSE_RESOURCE_NAME_STR_SGDMA_RX_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	[4] = {
		.start = na_sgdma_tx_irq,
		.end   = na_sgdma_tx_irq,
		.name  = ATSE_RESOURCE_NAME_STR_SGDMA_TX_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

static struct atse_plat_data atse_platdata = {
	.flags		= 1,
};

static struct platform_device atse_device = {
	/* the name string must be the same as in struct patform_driver */
	.name		= ATSE_CARDNAME,
	.id		= 0,
	.num_resources	= ARRAY_SIZE(atse_resource),
	.resource	= atse_resource,
	.dev		= {
		.platform_data = &atse_platdata,
	}
};
static void atse_device_init(void)
{
	/* write eth hardware address to MAC registers */
	unsigned int mac_reg_0;
	unsigned int mac_reg_1 = 0x0;
	/*
	mac_reg_0 = excalibur_enet_hwaddr[0]   | 
		excalibur_enet_hwaddr[1] << 8  |
		excalibur_enet_hwaddr[2] << 16 |
		excalibur_enet_hwaddr[3] << 24;

	mac_reg_1 = excalibur_enet_hwaddr[4] | excalibur_enet_hwaddr[5] << 8;
	*/
	mac_reg_0 = 0x00   |
		0x07 << 8  |
		0xED << 16 |
		0x0D << 24;

	mac_reg_1 = 0x09   |
		0x19 << 8  ;

	writel(mac_reg_0, ATSE_MAC_REG_MAC_ADDR_0);
	writel(mac_reg_1, ATSE_MAC_REG_MAC_ADDR_1);
}
#else
static void atse_device_init(void) {}
#endif /* CONFIG_ATSE */

/* Altera Triple Speed Ethernet (SLS) */

#if defined (CONFIG_ALT_TSE)

#include         "../drivers/net/altera_tse.h"

#define na_tse_mac_control_port TSE_MAC_BASE

#define na_sgdma_rx_csr         SGDMA_RX_BASE
#define na_sgdma_tx             SGDMA_TX_BASE

#ifdef DESCRIPTOR_MEMORY_BASE
#define na_descriptor_memory DESCRIPTOR_MEMORY_BASE
#define na_descriptor_memory_size DESCRIPTOR_MEMORY_SPAN
#endif

#define na_sgdma_rx_csr_irq SGDMA_RX_IRQ
#define na_sgdma_tx_irq SGDMA_TX_IRQ

static struct resource alt_tse_resource[] = {

  [0] = {
    .start = na_tse_mac_control_port,
    .end   = na_tse_mac_control_port + 0x400 - 1, /* code from sopc file */
    .name  = TSE_RESOURCE_MAC_DEV,
    .flags = IORESOURCE_MEM,
  },
  [1] = {
    .start = na_sgdma_rx_csr,
    .end   = na_sgdma_rx_csr + 0x400 - 1,         /* code from sopc file */
    .name  = TSE_RESOURCE_SGDMA_RX_DEV,
    .flags = IORESOURCE_MEM,
  },
  [2] = {
    .start = na_sgdma_tx,
    .end   = na_sgdma_tx + 0x400 - 1,             /* code from sopc file */
    .name  = TSE_RESOURCE_SGDMA_TX_DEV,
    .flags = IORESOURCE_MEM,
  },
  [3] = {
    .start = na_sgdma_rx_csr_irq,
    .end   = na_sgdma_rx_csr_irq,
    .name  = TSE_RESOURCE_SGDMA_RX_IRQ,
    .flags = IORESOURCE_IRQ,
  },
  [4] = {
    .start = na_sgdma_tx_irq,
    .end   = na_sgdma_tx_irq,
    .name  = TSE_RESOURCE_SGDMA_TX_IRQ,
    .flags = IORESOURCE_IRQ,
  },

#ifdef na_descriptor_memory
  [5] = {
      .start = na_descriptor_memory,
      .end   = na_descriptor_memory + na_descriptor_memory_size - 1,      /* code from sopc file */
      .name  = TSE_RESOURCE_SGDMA_DES_DEV,
      .flags = IORESOURCE_MEM,
  },
#else
  [5] = {
      .start = 0,
      .end   = 0,                                 /* code from sopc file */
      .name  = TSE_RESOURCE_SGDMA_DES_DEV,
      .flags = IORESOURCE_MEM,
  },
#endif

#ifdef CONFIG_PHY_IRQ_PRESENCE
  [6] = {
      .start = 0,
      .end   = 0,                                 /* code from sopc file */
      .name  = TSE_RESOURCE_SGDMA_PHY_DEV,
      .flags = IORESOURCE_MEM,
  },
  [7] = {
      .start = 0,
      .end   = 0,                                 /* code from sopc file */
      .name  = TSE_RESOURCE_SGDMA_PHY_IRQ,
      .flags = IORESOURCE_IRQ,
  },
#endif

};

    
static struct alt_tse_mdio_private alt_tse_mdio_private = {
	.irq = {
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
		PHY_POLL,
	}
};

     
static struct platform_device alt_tse_mdio_device = {
	.name	= ALT_TSE_MDIO_NAME,
	.id	= 0,
	.num_resources  = ARRAY_SIZE(alt_tse_resource),
	.resource = alt_tse_resource,
	.dev	= {
	  .platform_data = &alt_tse_mdio_private,
	}
};

/* all of this, except mii_id can be changed with ethtool */
static struct alt_tse_config tsemac0_config = {
	.mii_id = 0, /* should match alt_tse_mdio_device->id from above */
	.phy_addr = 18,
	.tse_supported_modes =  PHY_GBIT_FEATURES,
/*
	supported modes can be 
		SUPPORTED_10baseT_Half 
		SUPPORTED_10baseT_Full 
		SUPPORTED_100baseT_Half 
		SUPPORTED_100baseT_Full 
		SUPPORTED_Autoneg 
		SUPPORTED_TP 
		SUPPORTED_MII  ----------  Up to here is PHY_BASIC_FEATURES
		SUPPORTED_1000baseT_Half
		SUPPORTED_1000baseT_Full -- here PHY_GBIT_FEATURES
*/
	.interface = PHY_INTERFACE_MODE_RGMII_ID,
/*	Interfaces can be
		PHY_INTERFACE_MODE_MII
		PHY_INTERFACE_MODE_GMII
		PHY_INTERFACE_MODE_SGMII
		PHY_INTERFACE_MODE_TBI
		PHY_INTERFACE_MODE_RMII
		PHY_INTERFACE_MODE_RGMII
		PHY_INTERFACE_MODE_RGMII_ID
		PHY_INTERFACE_MODE_RGMII_RXID
		PHY_INTERFACE_MODE_RGMII_TXID
		PHY_INTERFACE_MODE_RTBI
*/
	.flags = 0,  /* these are apparently phy specific... */
	
	.autoneg = AUTONEG_ENABLE,
	/* speed and duplex only valid if autoneg is AUTONED_DISABLE */
	.speed = SPEED_100, /* SPEED_10, SPEED_100, SPEED_1000 */
	.duplex = DUPLEX_HALF, /* DUPLEX_HALF, DUPLEX_FULL */

	.rx_fifo_depth = ALT_TSE_TX_RX_FIFO_DEPTH,
        .tx_fifo_depth = ALT_TSE_TX_RX_FIFO_DEPTH,
        .ethaddr = {0x00 , 0x70 , 0xed , 0x11 , 0x12 , 0x12},
};

static struct platform_device alt_tse_device = {
  /* the name string must be the same as in struct patform_driver */
  .name   = ALT_TSE_NAME,
  .id   = 0,
  .num_resources  = ARRAY_SIZE(alt_tse_resource),
  .resource = alt_tse_resource,
  .dev    = {
    .platform_data = &tsemac0_config,
  }
};   

static void __init parse_mac_addr(struct alt_tse_config *tse_config, char *macstr)
{
        int i, j;
        unsigned char result, value;

        for (i = 0; i < 6; i++) {
                result = 0;

                if (i != 5 && *(macstr + 2) != ':')
                        return;

                for (j = 0; j < 2; j++) {
                        if (isxdigit(*macstr)
                            && (value =
                                isdigit(*macstr) ? *macstr -
                                '0' : toupper(*macstr) - 'A' + 10) < 16) {
                                result = result * 16 + value;
                                macstr++;
                        } else
                                return;
                }

                macstr++;
//                tse_config->ethaddr[i] = result;
                tse_config->ethaddr[i] = result;
       }

}


static int __init setup_tsemac0(char *s)
{
        printk(KERN_INFO "Altera TSE MAC 0 ethaddr = %s\n", s);
        parse_mac_addr((struct alt_tse_config*) &tsemac0_config, s);
        return 0;
}

__setup("tsemac0=", setup_tsemac0);


static void tse_device_init(void)
{
#ifndef na_descriptor_memory
	alt_tse_resource[5].start = kmalloc(ALT_TSE_TOTAL_SGDMA_DESC_SIZE, GFP_KERNEL);
	if (!alt_tse_resource[5].start)
		return -ENOMEM;
	alt_tse_resource[5].end   = alt_tse_resource[5].start + ALT_TSE_TOTAL_SGDMA_DESC_SIZE;
#endif
}
#else
static void tse_device_init(void) {}
#endif 

#if defined(CONFIG_ETHOC) || defined(CONFIG_ETHOC_MODULE)
#include <linux/etherdevice.h>
#include <net/ethoc.h>

#define ETHOC_USE_SRAM_BUFFER 0 /* 0 for dma_alloc from system memory */
#define na_igor_mac IGOR_MAC_BASE
#define na_igor_mac_irq IGOR_MAC_IRQ
#define na_ssram SSRAM_BASE
#define na_ssram_end (SSRAM_BASE + SSRAM_SPAN)

static struct ethoc_platform_data ethoc_platdata = {
	.hwaddr = { 0x00,0x07,0xed,0x0a,0x03,0x29 },
	.phy_id = -1,
};

static struct resource ethoc_resources[] = {
	{
	 .start = na_igor_mac,
	 .end = na_igor_mac + 0xfff,
	 .flags = IORESOURCE_MEM,
	 },
#if ETHOC_USE_SRAM_BUFFER
	{
	 .start = na_ssram,
	 .end = na_ssram_end -1,
	 .flags = IORESOURCE_MEM,
	 },
#endif
	{
	 .start = na_igor_mac_irq,
	 .end = na_igor_mac_irq,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct platform_device ethoc_device = {
	.name = "ethoc",
	.id = -1,
	.dev = {
		.platform_data = &ethoc_platdata,
		},
	.num_resources = ARRAY_SIZE(ethoc_resources),
	.resource = ethoc_resources,
};
#endif

#if defined(CONFIG_OPEN_ETH)
#include <linux/etherdevice.h>
#include <net/open_eth.h>

static struct oeth_platform_data oeth_platdata = {
      .hwaddr = { 0x00, 0x07, 0xed, 0x0a, 0x03, 0x29 },
      .phy_id = -1,
};

static struct resource oeth_resources[] = {
      {
       .start = IGOR_MAC_BASE,
       .end = IGOR_MAC_BASE + IGOR_MAC_SPAN - 1,
       .flags = IORESOURCE_MEM,
       },
      {
       .start = IGOR_MAC_IRQ,
       .end = IGOR_MAC_IRQ,
       .flags = IORESOURCE_IRQ,
       },
};

static struct platform_device oeth_device = {
      .name = "oeth",
      .id = -1,
      .dev = {
              .platform_data = &oeth_platdata,
              },
      .num_resources = ARRAY_SIZE(oeth_resources),
      .resource = oeth_resources,
};
#endif

/*
 *	Nios2 platform devices
 */

static struct platform_device *nios2_devices[] __initdata = {
	&nios2_jtaguart,

	&nios2_uart,

#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
	&gpio_leds_device,
#endif

#if defined(CONFIG_MOUSE_GPIO) || defined(CONFIG_MOUSE_GPIO_MODULE)
	&gpio_mouse_device,
#endif

#if defined(CONFIG_MTD_PHYSMAP) || defined(CONFIG_MTD_PHYSMAP_MODULE)
	&cfi_flash_device,
#endif

#if defined(CONFIG_MTD_NAND_PLATFORM) || defined(CONFIG_MTD_NAND_PLATFORM_MODULE)
	&async_nand_device,
#endif

#if (defined(CONFIG_SPI_ALTERA)  || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(EPCS_CONTROLLER_BASE)
	&epcs_controller_device,
#endif

#if (defined(CONFIG_SPI_ALTERA)  || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(TOUCH_PANEL_SPI_BASE)
	&touch_panel_spi_device,
#endif

#if (defined(CONFIG_SPI_ALTERA)  || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(MMC_SPI_BASE)
	&mmc_spi_device,
#endif

#if defined(CONFIG_SPI_GPIO) || defined(CONFIG_SPI_GPIO_MODULE)
	&nios2_spi_gpio_3_device,
#endif

#if (defined(CONFIG_MMC_NIOS) || defined(CONFIG_MMC_NIOS_MODULE)) && defined(SDIO_BASE)
	&nios_mmc_device,
#endif

#if defined(USE_PATA_PLATFORM) || defined(USE_PATA_ALTERA_CF)
	&cf_device,
#endif

#if (defined(CONFIG_I2C_OCORES) || defined(CONFIG_I2C_OCORES_MODULE)) && defined(I2C_0_BASE)
	&i2c_oc_0_device,
#endif
#if (defined(CONFIG_I2C_OCORES) || defined(CONFIG_I2C_OCORES_MODULE)) && defined(I2C_1_BASE)
	&i2c_oc_1_device,
#endif

#if defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
	&i2c_gpio_0_device,
	&i2c_gpio_1_device,
	&i2c_gpio_2_device,
#endif

#if defined(CONFIG_W1_MASTER_GPIO) || defined(CONFIG_W1_MASTER_GPIO_MODULE)
	&nios2_w1_gpio_0_device,
#endif

#if defined(CONFIG_SERIO_ALTPS2) && defined(PS2_0_BASE)
	&altps2_0_device,
#endif
#if defined(CONFIG_SERIO_ALTPS2) && defined(PS2_1_BASE)
	&altps2_1_device,
#endif

#if defined(CONFIG_ALTERA_REMOTE_UPDATE) && defined(ALTREMOTE_BASE)
	&altremote_device,
#endif

#if defined(CONFIG_SMC91X) && defined(ENET_BASE)
	&smc91x_device,
#endif

#if defined(CONFIG_DM9000) && defined(DM9000_BASE)
	&dm9k_device,
#endif

#if defined (CONFIG_ATSE)
	&atse_device,
#endif

#if defined (CONFIG_ALT_TSE)
	&alt_tse_mdio_device,
	&alt_tse_device,
#endif

#if defined(CONFIG_ETHOC) || defined(CONFIG_ETHOC_MODULE)
	&ethoc_device,
#endif
#if defined(CONFIG_OPEN_ETH)
    &oeth_device,
#endif
};

static int __init init_BSP(void)
{
	printk(KERN_INFO "%s(): registering device resources\n", __func__);
	nios2_gpio_init();
#ifdef USE_PATA_PLATFORM
	cf_init(CF_CTL_BASE);
#endif
	nios2_plat_nand_init();
	atse_device_init();
	tse_device_init();
	i2c_register_board_info(0, nios2_i2c_0_board_info,
				ARRAY_SIZE(nios2_i2c_0_board_info));
	i2c_register_board_info(1, nios2_i2c_1_board_info,
				ARRAY_SIZE(nios2_i2c_1_board_info));
	platform_add_devices(nios2_devices, ARRAY_SIZE(nios2_devices));

#if defined(CONFIG_SPI) || defined(CONFIG_SPI_MODULE)

#if (defined(CONFIG_TOUCHSCREEN_ADS7846) || defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)) && \
	defined(TOUCH_PANEL_PEN_IRQ_N_BASE)
	ads7843_pendown_init();
#endif

	spi_register_board_info(nios2_spi_devices,
				ARRAY_SIZE(nios2_spi_devices));
#endif
	return 0;
}

arch_initcall(init_BSP);
