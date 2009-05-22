
/* stripped down version of platform device registration, 
 */

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
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/altjuart.h>
#include <linux/altuart.h>
#include <asm/nios.h>

#ifdef CONFIG_SERIAL_ALTERA_JTAGUART

/*
 *	Altera JTAG UART
 */

static struct altera_jtaguart_platform_uart nios2_jtaguart_platform[] = {
#ifdef JTAG_UART_BASE
	{
	 .mapbase = (unsigned long)JTAG_UART_BASE,
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

#endif


#ifdef CONFIG_SERIAL_ALTERA_UART
/*
 *	Altera UART
 */

static struct altera_uart_platform_uart nios2_uart_platform[] = {
	{
	 .mapbase = (unsigned long)UART_BASE,
	 .irq = UART_IRQ,
	 .uartclk = UART_FREQ
	},
	{},
};


static struct platform_device nios2_uart = {
	.name = "altera_uart",
	.id = 0,
	.dev.platform_data = nios2_uart_platform,
};
#endif

#if defined(CONFIG_MTD_PHYSMAP) || defined(CONFIG_MTD_PHYSMAP_MODULE)
static struct mtd_partition nios2_partitions[] = {
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
	 .name = "Altera Image 0",
	 .size =   0x00380000,
	 .offset = 0x03500000,
	 },
	{
	 .name = "FPGA Image 1",
	 .size =   0x00380000,
	 .offset = 0x03880000,
	 },
	{
	 .name = "FPGA Image 2",
	 .size =   0x00380000,
	 .offset = 0x03c00000,
	 },
	{
	  .name = "options-bits",
	  .size =   0x00020000,
	  .offset = 0x03f80000,
	}
};

static struct physmap_flash_data nios2_flash_data = {
	.width = 2,		/* 16 bits data bus */
	.parts = nios2_partitions,
	.nr_parts = ARRAY_SIZE(nios2_partitions),
};

static struct resource nios2_flash_resource = {
	.start = EXT_FLASH_BASE,
	.end = EXT_FLASH_BASE + EXT_FLASH_SPAN - 1,
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


#if defined (CONFIG_ALT_TSE)
#include         "../drivers/net/altera_tse.h"

#define na_tse_mac_control_port TSE_MAC_BASE

#define na_sgdma_rx_csr         SGDMA_RX_BASE
#define na_sgdma_tx             SGDMA_TX_BASE


#define na_sgdma_rx_csr_irq SGDMA_RX_IRQ
#define na_sgdma_tx_irq SGDMA_TX_IRQ


/* the tse mdio driver must define it's own resource to
 * avoid loops in the resource tree.
 */
static struct resource alt_tse_mdio_resource[] = {
  [0] = {
    .start = na_tse_mac_control_port,
    .end   = na_tse_mac_control_port + 0x400 - 1, /* hard number as per system sopc file */
    .name  = TSE_RESOURCE_MAC_DEV,
    .flags = IORESOURCE_MEM,
  }
};

static struct resource alt_tse_resource[] = {

  [0] = {
    .start = na_tse_mac_control_port,
    .end   = na_tse_mac_control_port + 0x400 - 1, /* hard number as per system sopc file */
    .name  = TSE_RESOURCE_MAC_DEV,
    .flags = IORESOURCE_MEM,
  },
  [1] = {
    .start = na_sgdma_rx_csr,
    .end   = na_sgdma_rx_csr + 0x400 - 1,         /* hard number as per system sopc file */
    .name  = TSE_RESOURCE_SGDMA_RX_DEV,
    .flags = IORESOURCE_MEM,
  },
  [2] = {
    .start = na_sgdma_tx,
    .end   = na_sgdma_tx + 0x400 - 1,             /* hard number as per system sopc file */
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

  #ifdef  CONFIG_DECS_MEMORY_SELECT
  [5] = {
      .start = DECS_MEMORY_BASE_ADDR,
      .end   = DECS_MEMORY_BASE_ADDR+ALT_TSE_TOTAL_SGDMA_DESC_SIZE -1,      /* hard number as per system sopc file */
      .name  = TSE_RESOURCE_SGDMA_DES_DEV,
      .flags = IORESOURCE_MEM,
  },
  #else
  [5] = {
      .start = 0,
      .end   = 0,                                 /* hard number as per system sopc file */
      .name  = TSE_RESOURCE_SGDMA_DES_DEV,
      .flags = IORESOURCE_MEM,
  },
  #endif

  #ifdef CONFIG_PHY_IRQ_PRESENCE
  [6] = {
      .start = 0,
      .end   = 0,                                 /* hard number as per system sopc file */
      .name  = TSE_RESOURCE_SGDMA_PHY_DEV,
      .flags = IORESOURCE_MEM,
  },
  [7] = {
      .start = 0,
      .end   = 0,                                 /* hard number as per system sopc file */
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
	.num_resources  = ARRAY_SIZE(alt_tse_mdio_resource),
	.resource = alt_tse_mdio_resource,
	.dev	= {
	  .platform_data = &alt_tse_mdio_private,
	}
};

//all of this, except mii_id can be changed with ethtool & fifo depth
static struct alt_tse_config tsemac0_config = {
	.mii_id = 0, //should match alt_tse_mdio_device->id from above
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
	/* Note, this should really be PHY_INTERFACE_MODE_RGMII_ID
	 * but this is not present in include/linux/phy.h for 2.6.21
	 * I've moddified marvell.c instead fo changing generic headers
	 * to set  RX/TX DELAY  even for this mode
	 */ 
	.interface = 		PHY_INTERFACE_MODE_RGMII,
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
	.flags = 0,  //these are apparently phy specific...
	
	.autoneg = AUTONEG_ENABLE,
	//speed and duplex only valid if autoneg is AUTONED_DISABLE
	.speed = SPEED_100, //SPEED_10, SPEED_100, SPEED_1000
	.duplex = DUPLEX_FULL, //DUPLEX_HALF, DUPLEX_FULL

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

#endif

#if defined(CONFIG_DM9000)
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


/*
 *	Nios2 platform devices
 */

static struct platform_device *nios2_devices[] __initdata = {
#ifdef CONFIG_SERIAL_ALTERA_JTAGUART
	&nios2_jtaguart,
#endif

#ifdef CONFIG_SERIAL_ALTERA_UART
	&nios2_uart,
#endif

#if defined(CONFIG_MTD_PHYSMAP) || defined(CONFIG_MTD_PHYSMAP_MODULE)	
	&nios2_flash_device,
#endif

#if defined(CONFIG_DM9000)
	&dm9k_device,
#endif

#ifdef CONFIG_ALT_TSE
        &alt_tse_mdio_device,
        &alt_tse_device,
#endif
};

static int __init init_BSP(void)
{
	printk("adding nios2 pfm drivers\n");

#ifdef CONFIG_ALT_TSE
#ifndef  CONFIG_DECS_MEMORY_SELECT  
    alt_tse_resource[5].start = kmalloc(ALT_TSE_TOTAL_SGDMA_DESC_SIZE, GFP_KERNEL);
    alt_tse_resource[5].end   = alt_tse_resource[6].start + ALT_TSE_TOTAL_SGDMA_DESC_SIZE;
#endif
#endif

	platform_add_devices(nios2_devices, ARRAY_SIZE(nios2_devices));
	return 0;
}

arch_initcall(init_BSP);
