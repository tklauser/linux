/*
 *  linux/drivers/net/atse.h
 *
 *  Copyright (C) 2008       Joseph "Camel" Chen (joe4camel@gmail.com)
 *
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 * NON INFRINGEMENT.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef _ATSE_H_
#define _ATSE_H_

/* Stop compile if nios2.h is not generated with the Altera TSE-SGDMA FPGA design */
#ifndef na_tse_mac_control_port
#error "**** This build has not been configured with Alteras example design of TSE-SGDMA."
#error "**** You need to do: make vendor_hwselect SYSPTF=<your_path>/NiosII_stratixII_2s60_RoHS_TSE_SGDMA_sopc.ptf"
#error "**** Or, You need to unselect ATSE ethernet driver with make menuconfig"
#endif

#define NIOS2_DCACHE_LINE_SIZE  32

#define ATSE_CARDNAME "atse"


#define ATSE_RESOURCE_NAME_STR_DESC_MEM     "atse_resource_desc_mem"
#define ATSE_RESOURCE_NAME_STR_SGDMA_RX_MEM "atse_resource_sgdma_rx_mem"
#define ATSE_RESOURCE_NAME_STR_SGDMA_TX_MEM "atse_resource_sgdma_tx_mem"
#define ATSE_RESOURCE_NAME_STR_SGDMA_RX_IRQ "atse_resource_sgdma_rx_irq"
#define ATSE_RESOURCE_NAME_STR_SGDMA_TX_IRQ "atse_resource_sgdma_tx_irq"



/* 4 bytes in a register word */
#define BYTES_IN_WORD                            4

/*  macros starting with na_ are defined in nios2.h */
#define ATSE_MAC_BASE                     ioremap(na_tse_mac_control_port, 0x3FC + 1)
#define ATSE_MAC_REG_MAC_ADDR_0          (ATSE_MAC_BASE + 0x0C)
#define ATSE_MAC_REG_MAC_ADDR_1          (ATSE_MAC_BASE + 0x10)

/* the memory descriptore */
#define ATSE_SGDMA_DESC_MEM_BASE          na_descriptor_memory_s1

#define ATSE_SGDMA_TX_IRQ                 na_sgdma_tx_irq
#define ATSE_SGDMA_RX_IRQ                 na_sgdma_rx_csr_irq

/*
 * The hard coded numbers are from the following documents: Tables
 * 4-10 and 4-11
 *
 * 1.  Triple * Speed Ethernet, MegaCore Function user Guide, MegaCore
 * Version 7.2, * Document Date October 2007, Altera
 *
 */


/* number of bytes in one MDIO register */
#define ATSE_PHY_ID_1_OFFSET   (2 * BYTES_IN_WORD)
#define ATSE_PHY_ID_2_OFFSET   (3 * BYTES_IN_WORD)
#define ATSE_MAC_REG_MDIO_ADDR_0   (ATSE_MAC_BASE + 0x3C  )
#define ATSE_MAC_REG_MDIO_ADDR_1   (ATSE_MAC_BASE + 0x40  )
#define ATSE_MAC_REG_MDIO_SPACE_0  (ATSE_MAC_BASE + 0x0200)
#define ATSE_MAC_REG_MDIO_SPACE_1  (ATSE_MAC_BASE + 0x0280)

#define ATSE_SET_PHY_MDIO_CONTROL(n) writel((n), ATSE_MAC_REG_MDIO_SPACE_0)
#define ATSE_GET_PHY_MDIO_CONTROL()  readl(ATSE_MAC_REG_MDIO_SPACE_0)

#define ATSE_GET_PHY_MDIO_STATUS()   readl(ATSE_MAC_REG_MDIO_SPACE_0 + 1 * BYTES_IN_WORD)

#define ATSE_PHY_MDIO_CONTROL_RESET_BIT                  (0x01 << 15)

#define ATSE_PHY_MDIO_CONTROL_AUTO_NEGO_ENA_BIT          (0x01 << 12)
#define ATSE_PHY_MDIO_CONTROL_POWER_DOWN_BIT             (0x01 << 11)

#define ATSE_PHY_MDIO_CONTROL_START_AUTO_NEGO_BIT        (0x01 << 9)
#define ATSE_PHY_MDIO_CONTROL_FULL_DUP_MODE_BIT          (0x01 << 8)

#define ATSE_PHY_MDIO_CONTROL_AUTO_NEG_COMPLETE_BIT      (0x01 << 5)

/* PHY Speed Settings */
#define ATSE_SET_PHY_SPEED(the_speed)					\
	do {								\
		u32 i;							\
		i = ATSE_GET_PHY_MDIO_CONTROL();			\
		switch(the_speed) {					\
		case 1000:						\
			i |= (0x01 << 6);				\
                        i &= ~(0x01 << 13);				\
                        break;						\
		default:						\
			printk("ATSE: unkonw phy speed:%d\n", the_speed); \
			printk("ATSE: use 100 phy speed\n");		\
		case 100:						\
			i &= ~(0x01 << 6);				\
			i |= (0x01 << 13);				\
                        break;						\
		case 10:						\
			i &= ~(0x01 << 6);				\
			i &= ~(0x01 << 13);				\
                        break;						\
		}							\
		ATSE_SET_PHY_MDIO_CONTROL(i);                           \
	} while(0)

                 
/* PHY ID for Marvell 88E1111 */
#define ATSE_PHY_ID_MARVELL                 0x0141
#define ATSE_PHY_VENDOR_NAME_STR_MARVELL    "Marvell 88E1111 PHY"

/* Marvell Vendor Specific, according to Altera source code */
#define ATSE_READ_PHY_MDIO_REG(n)     readl(ATSE_MAC_REG_MDIO_SPACE_0 + (n) * BYTES_IN_WORD)

#define ATSE_GET_MVLPHY_LINK_STATUS() ATSE_READ_PHY_MDIO_REG(17)
#define ATSE_MVLPHY_IS_1000(dat)     (((dat) >> 14) & 0x03) == 2 ? 1: 0
#define ATSE_MVLPHY_IS_FULL_DUP(dat) ( (dat) >> 13) & 0x1

/* PHY ID for National DP83865 */
#define ATSE_PHY_ID_NATIONAL_1          0x20005c7a
#define ATSE_PHY_VENDOR_NAME_STR_NATIONAL_1   "National DP83865 PHY"

/* this PHY reportedly comes with Altera NEEK kit board */
#define ATSE_PHY_ID_NATIONAL_2          0x20005c90
#define ATSE_PHY_VENDOR_NAME_STR_NATIONAL_2   "National Semiconductor"

/* Microtronix Ethernet PHY Design Kit, added by Eintisy */
#define ATSE_PHY_ID_TDK_78Q2120C        0x000e70c5
#define ATSE_PHY_VENDOR_NAME_MICROTRONIX      "Microtronix Ethernet PHY"

/* make up an unknown phy ID */
#define ATSE_PHY_ID_UNKNOWN             0x12345678

/* na_sgdma_tx is parsed from the PTF file and defined in nios2_syste.h */
#define ATSE_SGDMA_TX_BASE    ioremap(na_sgdma_tx, 0x400)
/* na_sgdma_rx_csr is parsed from PTF file  and defined in nios2_syste.h */
#define ATSE_SGDMA_RX_BASE    ioremap(na_sgdma_rx_csr, 0x400)

/* Regsiter 0 offset */
#define ATSE_SGDMA_TX_STATUS_BASE  ATSE_SGDMA_TX_BASE
#define ATSE_SGDMA_RX_STATUS_BASE  ATSE_SGDMA_RX_BASE

/* Register 4 Offset, a 4-byte register */
#define ATSE_SGDMA_TX_CONTROL_BASE (ATSE_SGDMA_TX_BASE + 4*BYTES_IN_WORD)
#define ATSE_SGDMA_RX_CONTROL_BASE (ATSE_SGDMA_RX_BASE + 4*BYTES_IN_WORD)


/* sgdma Control Register Map  */
#define ATSE_SGDMA_CONTROL_IE_ERROR_BIT                      (0x01 << 0)
#define ATSE_SGDMA_CONTROL_IE_EOP_ENCOUNTERED_BIT            (0x01 << 1)
#define ATSE_SGDMA_CONTROL_IE_DESC_COMPLETED_BIT             (0x01 << 2)
#define ATSE_SGDMA_CONTROL_IE_CHAIN_COMPLETED_BIT            (0x01 << 3)
#define ATSE_SGDMA_CONTROL_IE_GLOBAL_BIT                     (0x01 << 4)
#define ATSE_SGDMA_CONTROL_RUN_BIT                           (0x01 << 5)
#define ATSE_SGDMA_CONTROL_STOP_DMA_ER_BIT                   (0x01 << 6)
#define ATSE_SGDMA_CONTROL_SW_RESET_BIT                      (0x01 << 16)
#define ATSE_SGDMA_CONTROL_PARK_BIT                          (0x01 << 17)
#define ATSE_SGDMA_CONTROL_CLEAR_INTERRUPT_BIT               (0x01 << 31)

/* sgdms status Register Map */
#define ATSE_SGDMA_STATUS_ERROR_BIT                          (0x01 << 0)
#define ATSE_SGDMA_STATUS_EOP_ENCOUNTERED_BIT                (0x01 << 1)
#define ATSE_SGDMA_STATUS_DESCRIPTOR_COMPLETED_BIT           (0x01 << 2)
#define ATSE_SGDMA_STATUS_CHAIN_COMPLETED_BIT                (0x01 << 3)
#define ATSE_SGDMA_STATUS_BUSY_BIT                           (0x01 << 4)


/*
 * sgdma buffer descriptor status bit map
 */
#define ATSE_SGDMA_DESC_STATUS_E_CRC_BIT                       (0x01 << 0)
#define ATSE_SGDMA_DESC_STATUS_E_PARITY_BIT                    (0x01 << 1)
#define ATSE_SGDMA_DESC_STATUS_E_OVERFLOW_BIT                  (0x01 << 2)
#define ATSE_SGDMA_DESC_STATUS_E_SYNC_BIT                      (0x01 << 3)
#define ATSE_SGDMA_DESC_STATUS_E_UEOP_BIT                      (0x01 << 4)
#define ATSE_SGDMA_DESC_STATUS_E_MEOP_BIT                      (0x01 << 5)
#define ATSE_SGDMA_DESC_STATUS_E_MSOP_BIT                      (0x01 << 6)
#define ATSE_SGDMA_DESC_STATUS_TERMINATED_BY_EOP_BIT           (0x01 << 7)

#define ATSE_ENABLE_SGDMA_RX_INT()                                       \
	do {                                                             \
	ATSE_SET_SGDMA_RX_CONTROL(                                       \
		ATSE_SGDMA_CONTROL_IE_ERROR_BIT           |              \
		ATSE_SGDMA_CONTROL_IE_EOP_ENCOUNTERED_BIT |              \
		ATSE_SGDMA_CONTROL_IE_DESC_COMPLETED_BIT  |              \
		ATSE_SGDMA_CONTROL_IE_CHAIN_COMPLETED_BIT |              \
		ATSE_SGDMA_CONTROL_IE_GLOBAL_BIT          |              \
		ATSE_GET_SGDMA_RX_CONTROL()                              \
		);                                                       \
	} while(0)
	


#define ATSE_SGDMA_BUSY_TIMEOUT_COUNT                         1000
#define ATSE_PHY_AUTONEG_TIMEOUT_COUNT                          10



/* set the control field bit 0 to 1 */
#define ATSE_SGDMA_DESC_CONTROL_GENERATE_EOP_BIT              0x01
/* set the control field bit 1 to 1 */
#define ATSE_SGDMA_DESC_CONTROL_READ_FIXED_ADDRESS_BIT        (0x1 << 1)
/* set the control field bit 2 to 1 */
#define ATSE_SGDMA_DESC_CONTROL_WRITE_FIXED_ADDRESS_BIT       (0x1 << 2)
/* set the control field bit 7 to 1 */
#define ATSE_SGDMA_DESC_CONTROL_OWNED_BY_HW_BIT               (0x1 << 7)



/* Altera TSE MAC Bit Maps */
#define ATSE_MAC_TX_CMD_STAT_OMIT_CRC_BIT     (1 << 17)
#define ATSE_MAC_TX_CMD_STAT_TXSHIFT16_BIT    (1 << 18)
#define ATSE_MAC_RX_CMD_STAT_RXSHIFT16_BIT    (1 << 25)


#define ATSE_GET_MAC_REV()                readl(ATSE_MAC_BASE + 0x0)

#define ATSE_SET_MAC_SCRATCH(n)           writel((n), ATSE_MAC_BASE + 0x04)
#define ATSE_GET_MAC_SCRATCH()            readl( ATSE_MAC_BASE + 0x04)

#define ATSE_SET_MAC_CMD_CONFIG(n)        writel((n), ATSE_MAC_BASE + 0x08)
#define ATSE_GET_MAC_CMD_CONFIG()         readl(ATSE_MAC_BASE + 0x08)

#define ATSE_SET_MAC_REG_STAT(n)          writel((n), ATSE_MAC_BASE + 0x58))
#define ATSE_GET_MAC_REG_STAT()           readl(ATSE_MAC_BASE + 0x58)

#define ATSE_SET_MAC_TX_CMD_STAT(n)       writel((n), ATSE_MAC_BASE + 0xE8)
#define ATSE_GET_MAC_TX_CMD_STAT()        readl(ATSE_MAC_BASE + 0xE8)

#define ATSE_SET_MAC_RX_CMD_STAT(n)       writel((n), ATSE_MAC_BASE + 0xEC)
#define ATSE_GET_MAC_RX_CMD_STAT()        readl(ATSE_MAC_BASE + 0xEC)



/* Altera TSE MAC Command-Config register Bit map */
#define ATSE_MAC_CC_TX_ENA_BIT        0x01
#define ATSE_MAC_CC_RX_ENA_BIT       (0x01 << 1 )
#define ATSE_MAC_CC_XON_GEN_BIT      (0x01 << 2 )
#define ATSE_MAC_CC_ETH_SPEED_BIT    (0x01 << 3 )
#define ATSE_MAC_CC_PROMIS_ENA_BIT   (0x01 << 4 )
#define ATSE_MAC_CC_PAD_ENA_BIT      (0x01 << 5 )
#define ATSE_MAC_CC_CRC_FWD_BIT      (0x01 << 6 )
#define ATSE_MAC_CC_PAUSE_FWD_BIT    (0x01 << 7 )
#define ATSE_MAC_CC_PAUSE_IGN_BIT    (0x01 << 8 )
#define ATSE_MAC_CC_TX_ADDR_INS_BIT  (0x01 << 9 )
#define ATSE_MAC_CC_HD_ENA_BIT       (0x01 << 10)
#define ATSE_MAC_CC_EXCESS_COL_BIT   (0x01 << 11)
#define ATSE_MAC_CC_LATE_COL_BIT     (0x01 << 12)
#define ATSE_MAC_CC_SW_RESET_BIT     (0x01 << 13)
#define ATSE_MAC_CC_MHASH_SEL_BIT    (0x01 << 14)
#define ATSE_MAC_CC_LOOP_ENA_BIT     (0x01 << 15)
#define ATSE_MAC_CC_TX_ADDR_SEL_MAC_0_MAC_1_BIT        (0x00 << 16)
#define ATSE_MAC_CC_TX_ADDR_SEL_SMAC_0_0_SMAC_0_1_BIT  (0x04 << 16)
#define ATSE_MAC_CC_TX_ADDR_SEL_SMAC_1_0_SMAC_1_1_BIT  (0x05 << 16)
#define ATSE_MAC_CC_TX_ADDR_SEL_SMAC_2_0_SMAC_2_1_BIT  (0x06 << 16)
#define ATSE_MAC_CC_TX_ADDR_SEL_SMAC_3_0_SMAC_3_1_BIT  (0x07 << 16)
#define ATSE_MAC_CC_MAGIC_ENA_BIT                      (0x01 << 19)
#define ATSE_MAC_CC_SLEEP_BIT                          (0x01 << 20)
#define ATSE_MAC_CC_WAKEUP_BIT                         (0x01 << 21)
#define ATSE_MAC_CC_XOFF_GEN_BIT                       (0x01 << 22)
#define ATSE_MAC_CC_CNTL_FRM_ENA_BIT                   (0x01 << 23)
#define ATSE_MAC_CC_NO_LEN_CK_BIT                      (0x01 << 24)
#define ATSE_MAC_CC_ENA_10_BIT                         (0x01 << 25)
#define ATSE_MAC_CC_RX_ERR_DISCD_BIT                    (0x01 << 26)
/* Bits 27 -- 30 are reserved */
#define ATSE_MAC_CC_CNT_RESET_BIT                      (0x01 << 31)




#define ATSE_SET_MAC_FRM_LEN(n)          writel((n), (ATSE_MAC_BASE + 0x14))
#define ATSE_SET_MAC_RX_SECTION_EMPTY(n) writel((n), (ATSE_MAC_BASE + 0x1C))
#define ATSE_SET_MAC_RX_SECTION_FULL(n)  writel((n), (ATSE_MAC_BASE + 0x20))
#define ATSE_SET_MAC_TX_SECTION_EMPTY(n) writel((n), (ATSE_MAC_BASE + 0x24))
#define ATSE_SET_MAC_TX_SECTION_FULL(n)  writel((n), (ATSE_MAC_BASE + 0x28))
#define ATSE_SET_MAC_RX_ALMOST_EMPTY(n)  writel((n), (ATSE_MAC_BASE + 0x2C))
#define ATSE_SET_MAC_RX_ALMOST_FULL(n)   writel((n), (ATSE_MAC_BASE + 0x30))
#define ATSE_SET_MAC_TX_ALMOST_EMPTY(n)  writel((n), (ATSE_MAC_BASE + 0x34))
#define ATSE_SET_MAC_TX_ALMOST_FULL(n)   writel((n), (ATSE_MAC_BASE + 0x38))






#define ATSE_SET_SGDMA_TX_CONTROL(n)        writel((n), ATSE_SGDMA_TX_CONTROL_BASE)
#define ATSE_GET_SGDMA_TX_CONTROL()         readl(ATSE_SGDMA_TX_CONTROL_BASE)
#define ATSE_SET_SGDMA_TX_WITH_DESC(n)      writel((n), ATSE_SGDMA_TX_BASE + 8 * BYTES_IN_WORD)

#define ATSE_SET_SGDMA_RX_CONTROL(n)        writel((n), ATSE_SGDMA_RX_CONTROL_BASE)
#define ATSE_GET_SGDMA_RX_CONTROL()         readl(ATSE_SGDMA_RX_CONTROL_BASE)
#define ATSE_SET_SGDMA_RX_WITH_DESC(n)      writel((n), ATSE_SGDMA_RX_BASE + 8 * BYTES_IN_WORD)



#define ATSE_MAC_INIT()   abcd						\
	do {								\
		int x=0;						\
		/* reset phy if neccessary */				\
                							\
                /* reset mac */						\
		while(ATSE_GET_MAC_CMD_CONFIG() & ATSE_MAC_CC_SW_RESET_BIT) { \
			if( x++ > 10000 ) {				\
				break;					\
			}						\
		}							\
									\
		if(x >= 10000) {					\
			printk("**** JoeCamel:%s:%d:MAC SW reset bit never cleared!\n", __FILE__, __LINE__); \
		}							\
									\
		if ((ATSE_GET_MAC_CMD_CONFIG() & (ATSE_MAC_CC_TX_ENA_BIT | ATSE_MAC_CC_RX_ENA_BIT) )  != 0) \
			printk("**** JoeCamel:%s:%d:WARN: RX/TX not disabled after reset... missing PHY clock?\n", __FILE__, __LINE__); \
									\
		ATSE_SET_MAC_CMD_CONFIG(				\
			(ATSE_MAC_CC_TX_ENA_BIT    |			\
			 ATSE_MAC_CC_RX_ENA_BIT    |			\
			 ATSE_MAC_CC_ETH_SPEED_BIT |			\
			 0x0) &						\
			~ATSE_MAC_CC_PROMIS_ENA_BIT			\
			);						\
									\
                							\
                /* Initialize MAC registers */				\
                							\
		ATSE_SET_MAC_FRM_LEN(1518);				\
                ATSE_SET_MAC_RX_ALMOST_EMPTY(8);			\
                ATSE_SET_MAC_RX_ALMOST_FULL(8);				\
                ATSE_SET_MAC_TX_ALMOST_EMPTY(8);			\
                ATSE_SET_MAC_TX_ALMOST_FULL(3);				\
                ATSE_SET_MAC_TX_SECTION_EMPTY(1024 - 16);		\
                ATSE_SET_MAC_TX_SECTION_FULL(0);			\
                ATSE_SET_MAC_RX_SECTION_EMPTY(1024 - 16);		\
                ATSE_SET_MAC_RX_SECTION_FULL(0);			\
        } while (0)



/* the following number is obtained by inspecting the PTF file:
 *  0x01401FFF - 0x01400000 + 1 = 0x2000 
 */

#define ATSE_TOTAL_SGDMA_DESC_MEM            0x2000



#define ATSE_SGDMA_TX_SW_RESET()   writel(ATSE_SGDMA_CONTROL_SWRESET_BIT, ATSE_SGDMA_TX_CONTROL_BASE)
#define ATSE_SGDMA_RX_SW_RESET()   writel(ATSE_SGDMA_CONTROL_SWRESET_BIT, ATSE_SGDMA_RX_CONTROL_BASE)

#define ATSE_GET_SGDMA_TX_STATUS() readl(ATSE_SGDMA_TX_STATUS_BASE)
#define ATSE_GET_SGDMA_RX_STATUS() readl(ATSE_SGDMA_RX_STATUS_BASE)

#define ATSE_CLEAR_SGDMA_TX_CONTROL() writel(0x0,  ATSE_SGDMA_TX_CONTROL_BASE)
#define ATSE_CLEAR_SGDMA_TX_STATUS()  writel(0x1F, ATSE_SGDMA_TX_STATUS_BASE)
#define ATSE_CLEAR_SGDMA_RX_CONTROL() writel(0x0,  ATSE_SGDMA_RX_CONTROL_BASE)
#define ATSE_CLEAR_SGDMA_RX_STATUS()  writel(0x1F, ATSE_SGDMA_RX_STATUS_BASE)

#define ATSE_GET_NEXT_SGDMA_DESC_PTR(desc_addr) readl(desc_addr + 4*BYTES_IN_WORD)

#define ATSE_SGDMA_TX_DESC_CHAIN_SIZE                                      1
#define ATSE_SGDMA_RX_DESC_CHAIN_SIZE                                      1

#define ATSE_SGDMA_TX_FIRST_DESC_OFFSET   0
#define ATSE_SGDMA_TX_SECOND_DESC_OFFSET  1
#define ATSE_SGDMA_RX_FIRST_DESC_OFFSET   2
#define ATSE_SGDMA_RX_SECOND_DESC_OFFSET  3


/* bits 0 through 15 of the offset 7 contain the acutal bytes transferred */
#define ATSE_GET_SGDMA_DESC_ACTUAL_BYTES_TRANSFERRED(desc_addr) (readl((desc_addr)+7*BYTES_IN_WORD)&0xFFFF) 
#define ATSE_GET_SGDMA_DESC_STATUS(desc_addr)  ((readl(desc_addr + 7*BYTES_IN_WORD) >> 16) & 0xFF)

#define ATSE_SET_PHY_POWER_DOWN()					\
	do {								\
		ATSE_SET_PHY_MDIO_CONTROL(ATSE_GET_PHY_MDIO_CONTROL() |	\
					  ATSE_PHY_MDIO_CONTROL_POWER_DOWN_BIT); \
	} while(0)


#define	ATSE_SET_MAC_TX_SHIFT16_ON()					\
	do {								\
		ATSE_SET_MAC_TX_CMD_STAT(ATSE_GET_MAC_TX_CMD_STAT() |   \
                                         ATSE_MAC_TX_CMD_STAT_TXSHIFT16_BIT); \
        }while(0)

#define	ATSE_SET_MAC_TX_SHIFT16_OFF()					\
	do {								\
		ATSE_SET_MAC_TX_CMD_STAT(ATSE_GET_MAC_TX_CMD_STAT() &   \
                                         ~ATSE_MAC_TX_CMD_STAT_TXSHIFT16_BIT); \
        }while(0)



#define ATSE_MVLPHY_IS_MDIX(dat)     (dat) & (1 << 6)
#define ATSE_MVLPHY_IS_1000(dat)     (((dat) >> 14) & 0x03) == 2 ? 1: 0
#define ATSE_MVLPHY_IS_FULL_DUP(dat) ( (dat) >> 13) & 0x1



struct atse_plat_data {
	unsigned int flags;
	/* allow replacement IO routines */
	
	void	(*inblk)(void __iomem *reg, void *data, int len);
	void	(*outblk)(void __iomem *reg, void *data, int len);
	void	(*dumpblk)(void __iomem *reg, int len);
	
};

#endif /*  _ATSE_H_ */

