
#include <linux/types.h>
#include <linux/init.h>
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
#if defined(na_cfi_flash_0)
	.start = na_cfi_flash_0,
	.end = na_cfi_flash_0_end - 1,
#else
	.start = na_ext_flash,
	.end = na_ext_flash_end - 1,
#endif
	.flags = IORESOURCE_MEM,
};

static struct platform_device nios2_flash_device = {
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

#undef NIOS2_NAND_PLAT_READY    /* def gpio to read R/B status from NAND chip */
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

#define MAX(x, y) (x > y ? x : y)
static struct resource nios2_plat_nand_resources = {
	.start = na_nand_flash_0,
	.end   = na_nand_flash_0 + 12,
	.flags = IORESOURCE_MEM,
};

static struct platform_device nios2_async_nand_device = {
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

#if (defined(CONFIG_SPI_ALTERA)  || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(na_epcs_controller)
#define EPCS_SPI_OFFSET 0x200	/* FIXME */
static struct resource na_epcs_controller_resource[] = {
	[0] = {
	       .start = na_epcs_controller + EPCS_SPI_OFFSET,
	       .end = na_epcs_controller + EPCS_SPI_OFFSET + 31,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = na_epcs_controller_irq,
	       .end = na_epcs_controller_irq,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct platform_device na_epcs_controller_device = {
	.name = "altspi",
	.id = 0,		/* Bus number */
	.num_resources = ARRAY_SIZE(na_epcs_controller_resource),
	.resource = na_epcs_controller_resource,
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

#if (defined(CONFIG_SPI_ALTERA)  || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(na_touch_panel_spi)
static struct resource na_touch_panel_spi_resource[] = {
	[0] = {
	       .start = na_touch_panel_spi,
	       .end = na_touch_panel_spi + 31,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = na_touch_panel_spi_irq,
	       .end = na_touch_panel_spi_irq,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct platform_device na_touch_panel_spi_device = {
	.name = "altspi",
	.id = 1,		/* Bus number */
	.num_resources = ARRAY_SIZE(na_touch_panel_spi_resource),
	.resource = na_touch_panel_spi_resource,
};
#endif /* spi master and devices */

#if (defined(CONFIG_TOUCHSCREEN_ADS7846) || defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)) && \
	defined(na_touch_panel_pen_irq_n)

#define ALTERA_PIO_IO_EXTENT      16
#define ALTERA_PIO_DATA           0
#define ALTERA_PIO_DIRECTION      4
#define ALTERA_PIO_IRQ_MASK       8
#define ALTERA_PIO_EDGE_CAP       12

static unsigned long ads7843_pendown_base;
static void ads7843_pendown_init(void)
{
	ads7843_pendown_base =
	    ioremap(na_touch_panel_pen_irq_n, ALTERA_PIO_IO_EXTENT);
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

#if (defined(CONFIG_SPI_ALTERA)  || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(na_mmc_spi)
static struct resource na_mmc_spi_resource[] = {
	[0] = {
	       .start = na_mmc_spi,
	       .end = na_mmc_spi + 31,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = na_mmc_spi_irq,
	       .end = na_mmc_spi_irq,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct platform_device na_mmc_spi_device = {
	.name = "altspi",
	.id = 2,		/* Bus number */
	.num_resources = ARRAY_SIZE(na_mmc_spi_resource),
	.resource = na_mmc_spi_resource,
};
#endif /* spi master and devices */

#if defined(CONFIG_SPI_ALTERA) || defined(CONFIG_SPI_ALTERA_MODULE)
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
	defined(na_touch_panel_pen_irq_n)
	{
	 .modalias = "ads7846",
	 .chip_select = 0,
	 .max_speed_hz = 125000 * 26,	/* (max sample rate @ 3V) * (cmd + data + overhead) */
	 .bus_num = 1,		/* must match spi host bus number of touch panel spi  */
	 .platform_data = &ads_info,
	 .irq = na_touch_panel_pen_irq_n_irq,
	 },
#endif

#if defined(CONFIG_MMC_SPI) || defined(CONFIG_MMC_SPI_MODULE)
	{
	 .modalias = "mmc_spi",
	 .max_speed_hz = 25000000,
	 .bus_num = 2,		/* must match spi host bus number of mmc spi  */
	 .chip_select = 0,
	 }
#endif

};
#endif

#if defined(na_cf_ide)
/* Use PATA_ALTERA_CF in preference to PATA_PLATFORM */
#if defined(CONFIG_PATA_ALTERA_CF) || defined(CONFIG_PATA_ALTERA_CF_MODULE)
#define USE_PATA_ALTERA_CF
#elif defined(CONFIG_PATA_PLATFORM) || defined(CONFIG_PATA_PLATFORM_MODULE)
#define USE_PATA_PLATFORM
#endif
#endif

#ifdef USE_PATA_ALTERA_CF
static struct resource na_cf_resources[] = {
	{
	 .start = na_cf_ide,
	 .end = na_cf_ide + 63,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = na_cf_ide_irq,
	 .end = na_cf_ide_irq,
	 .flags = IORESOURCE_IRQ,
	 },
	{
	 .start = na_cf_ctl,
	 .end = na_cf_ctl + 15,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = na_cf_ctl_irq,
	 .end = na_cf_ctl_irq,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct platform_device na_cf_device = {
	.name = "pata_altera_cf",
	.id = -1,
	.num_resources = ARRAY_SIZE(na_cf_resources),
	.resource = na_cf_resources,
};
#endif

#ifdef USE_PATA_PLATFORM
static struct pata_platform_info na_cf_platform_data = {
	.ioport_shift = 2,
};

static struct resource na_cf_resources[] = {
	{
	 .start = na_cf_ide,
	 .end = na_cf_ide + 31,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = na_cf_ide + 56,
	 .end = na_cf_ide + 56 + 3,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = na_cf_ide_irq,
	 .end = na_cf_ide_irq,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct platform_device na_cf_device = {
	.name = "pata_platform",
	.id = -1,
	.num_resources = ARRAY_SIZE(na_cf_resources),
	.resource = na_cf_resources,
	.dev = {
		.platform_data = &na_cf_platform_data,
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

/* SD/SDIO/MMC Host Platform Device */
/* Map na_sdio_host to na_sdio if it exists */
#if defined(na_sdio_host)
#define na_sdio na_sdio_host
#define na_sdio_irq na_sdio_host_irq
#define na_sdio_clock_freq na_sdio_host_clock_freq
#endif

#if (defined(CONFIG_MMC_NIOS) || defined(CONFIG_MMC_NIOS_MODULE)) && defined(na_sdio)

static struct nios_mmc_platform_mmc nios2_mmc_platform[] = {
	{
	 .mapbase = (unsigned long)na_sdio,
	 .irq = na_sdio_irq,
	 .clk_src = na_sdio_clock_freq,
	 },
};

static struct resource nios_mmc_resources[] = {
	[0] = {
		.start = na_sdio,
		.end = na_sdio + (16*4-1),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = na_sdio_irq,
		.end = na_sdio_irq,
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

#if (defined(CONFIG_I2C_OCORES) || defined(CONFIG_I2C_OCORES_MODULE)) && defined(na_i2c_0)
static struct resource na_i2c_0_resources[] = {
	[0] = {
	       .start = na_i2c_0,
	       .end = na_i2c_0 + 31,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = na_i2c_0_irq,
	       .end = na_i2c_0_irq,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct ocores_i2c_platform_data na_i2c_0_platform_data = {
	.regstep = 4,		/* four bytes between registers */
	.clock_khz = na_i2c_0_clock_freq / 1000,	/* input clock */
};

static struct platform_device na_i2c_0_device = {
	.name = "ocores-i2c",
	.id = 0,
	.dev = {
		.platform_data = &na_i2c_0_platform_data,
		},
	.num_resources = ARRAY_SIZE(na_i2c_0_resources),
	.resource = na_i2c_0_resources,
};
#endif

#if (defined(CONFIG_I2C_OCORES) || defined(CONFIG_I2C_OCORES_MODULE)) && defined(na_i2c_1)
static struct resource na_i2c_1_resources[] = {
	[0] = {
	       .start = na_i2c_1,
	       .end = na_i2c_1 + 31,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = na_i2c_1_irq,
	       .end = na_i2c_1_irq,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct ocores_i2c_platform_data na_i2c_1_platform_data = {
	.regstep = 4,		/* four bytes between registers */
	.clock_khz = na_i2c_1_clock_freq / 1000,	/* input clock */
};

static struct platform_device na_i2c_1_device = {
	.name = "ocores-i2c",
	.id = 1,
	.dev = {
		.platform_data = &na_i2c_1_platform_data,
		},
	.num_resources = ARRAY_SIZE(na_i2c_1_resources),
	.resource = na_i2c_1_resources,
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

#if defined(CONFIG_MTD_NAND_PLATFORM) || defined(CONFIG_MTD_NAND_PLATFORM_MODULE)
	&nios2_async_nand_device,
#endif

#if (defined(CONFIG_SPI_ALTERA)  || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(na_epcs_controller)
	&na_epcs_controller_device,
#endif

#if (defined(CONFIG_SPI_ALTERA)  || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(na_touch_panel_spi)
	&na_touch_panel_spi_device,
#endif

#if (defined(CONFIG_SPI_ALTERA)  || defined(CONFIG_SPI_ALTERA_MODULE)) && defined(na_mmc_spi)
	&na_mmc_spi_device,
#endif

#if defined(USE_PATA_PLATFORM) || defined(USE_PATA_ALTERA_CF)
	&na_cf_device,
#endif

#if (defined(CONFIG_MMC_NIOS) || defined(CONFIG_MMC_NIOS_MODULE)) && defined(na_sdio)
	&nios_mmc_device,
#endif

#if (defined(CONFIG_I2C_OCORES) || defined(CONFIG_I2C_OCORES_MODULE)) && defined(na_i2c_0)
	&na_i2c_0_device,
#endif
#if (defined(CONFIG_I2C_OCORES) || defined(CONFIG_I2C_OCORES_MODULE)) && defined(na_i2c_1)
	&na_i2c_1_device,
#endif

};

static int __init init_BSP(void)
{
#ifdef USE_PATA_PLATFORM
	cf_init(na_cf_ctl);
#endif
	nios2_plat_nand_init();
	platform_add_devices(nios2_devices, ARRAY_SIZE(nios2_devices));

#if defined(CONFIG_SPI_ALTERA) || defined(CONFIG_SPI_ALTERA_MODULE)

#if (defined(CONFIG_TOUCHSCREEN_ADS7846) || defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)) && \
	defined(na_touch_panel_pen_irq_n)
	ads7843_pendown_init();
#endif

	spi_register_board_info(nios2_spi_devices,
				ARRAY_SIZE(nios2_spi_devices));
#endif
	return 0;
}

arch_initcall(init_BSP);
