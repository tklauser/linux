/*
 *  linux/drivers/net/atse.c
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

/*
 * Modified by: Eintisy Chuang (eintisy.chuang@gfec.com.tw) 
 * Updated: 1. Add TDK78Q2120CPHY infor for Microtronix Ethernet PHY Design Kit
 */

#include <linux/init.h>
#include <linux/module.h>
MODULE_LICENSE("GPL");

#include <linux/delay.h>

#include <linux/pm.h>  /* pm_message_t */
#include <linux/platform_device.h>

#include <linux/mii.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/interrupt.h>

#include <asm/cacheflush.h>

#include "atse.h"

#define ATSE_CAN_USE_DATACS 1


/* #define ATSE_SGDMA_DO_UNALIGNED_TRANSFER */
#undef ATSE_SGDMA_DO_UNALIGNED_TRANSFER


static const char atse_version_str[] =
	"atse.c: v1.1, June 3, 2008 by Joseph (Camel) Chen <joe4camel@gmail.com>";
static const char atse_version_str1[] =
	"atse.c: modified version by Eintisy Chuang <eintisy.chuang@gfec.com.tw>";


/* static struct sk_buff *g_skb_array[ATSE_SGDMA_RX_DESC_CHAIN_SIZE]; */
static unsigned char *g_rx_data_buffer = NULL;
static unsigned char *g_rx_data_buffer_ptr = NULL;

static int g_tx_bytes_transferred = 0;

/*
 * Buffer Descriptor data structure
 *
 * The SGDMA controller buffer descriptor allocates
 * 64 bits for each address. To support ANSI C, the
 * struct implementing a descriptor places 32-bits
 * of padding directly above each address; each pad must
 * be cleared when initializing a descriptor.
 */
struct atse_sgdma_desc {
	u32   *read_addr;
	u32   read_addr_pad;
	
	u32   *write_addr;
	u32   write_addr_pad;
	
	u32   *next_desc;
	u32   next_desc_pad;
	
	u16   bytes_to_transfer;
	u8    read_burst_size;
	u8    write_burst_size;
	
	u16   bytes_transferred;
	u8    status;
	u8    control;
};



static struct atse_sgdma_desc *g_desc = (struct atse_sgdma_desc *)ATSE_SGDMA_DESC_MEM_BASE;
static struct atse_sgdma_desc *g_rx_desc      = NULL;
static struct atse_sgdma_desc *g_rx_desc_next = NULL;

static struct atse_sgdma_desc *g_tx_desc      = NULL;
static struct atse_sgdma_desc *g_tx_desc_next = NULL;



struct atse_phy_device {
	char *name;
	u32 phy_id;
	u8  link_stat_reg_num;

	u32 phy_addr;
	int (*get_link_speed)  (void);
	int (*link_is_full_dup)(void);
	int (*link_is_established)(void);
};


/* #define ATSE_DEBUG_PHY_DEV_FLAG */
/* #define ATSE_DEBUG_FUNC_TRACE_ENTER */
/* #define ATSE_DEBUG_FUNC_TRACE_EXIT */

/*  #define ATSE_DEBUG_DESC */
/* #define ATSE_DEBUG_TX_DATA */
/* #define ATSE_DEBUG_RX_DATA */

#ifdef ATSE_DEBUG_PHY_DEV_FLAG
#define ATSE_DEBUG_PRINT_PHY_DEV(oui, mod_num, rev_num, phy_dev) atse_debug_pring_phy_dev(__FILE__, __LINE__, oui, mod_num, rev_num, phy_dev)
static void atse_debug_pring_phy_dev(char *file, int line, int oui, int mod_num, int rev_num, struct atse_phy_device *phy_dev)
{
	printk("%s:%d:oui = %x, mod_num = %x, rev_num = %x, phy_name = %s\n", file, line,
	       oui, mod_num, rev_num, phy_dev->name);
	       

	return;
}
#else
#define ATSE_DEBUG_PRINT_PHY_DEV(oui, model_num, rev_num, phy_dev)
#endif



#ifdef ATSE_DEBUG_DESC
#define ATSE_DEBUG_PRINT_DESC(this_desc) atse_debug_print_desc(this_desc, __FILE__, __LINE__, #this_desc)
static void atse_debug_print_desc(struct atse_sgdma_desc *desc, char *file, int line, char *s)
{
	int i;
	unsigned char *pc;

	if (desc == NULL)
		return;
	printk("----ATSE_DEBUG:%s:%d:printing %s contents:\n", file, line, s);
	printk("\t%s addr = 0x%p\n",             s, desc);

	printk("\t read_addr=\t 0x%p\n",         desc->read_addr);
	printk("\t read_addr_pad=\t %x\n",     desc->read_addr_pad);
	
	printk("\t write_addr=\t 0x%p\n",        desc->write_addr);
	printk("\t write_addr_pad=\t %x\n",    desc->write_addr_pad);
	
	printk("\t next_desc=\t 0x%p\n",         desc->next_desc);
	printk("\t next_desc_pad=\t %x\n",     desc->next_desc_pad);
	
	printk("\t bytes_to_transfer=\t %x\n", desc->bytes_to_transfer);
	printk("\t read_burst_size=\t %x\n",   desc->read_burst_size);
	printk("\t write_burst_size=\t %x\n",  desc->write_burst_size);
	
	printk("\t bytes_transferred=\t %x\n", desc->bytes_transferred);
	printk("\t status=\t %x\n",            desc->status);
	printk("\t control=\t %x\n",           desc->control);

	printk("\n");
	printk("\t data tied to read_addr: len = %d\n", desc->bytes_to_transfer);

	pc = (unsigned char *) desc->read_addr;
	printk("\t\t");
	for (i = 0; i < desc->bytes_to_transfer; i++) {
		printk(" %02x", *(pc + i) );
		if ( (i + 1) % 8 == 0)
			printk(" ");
		if ( (i + 1) % 16 == 0)
			printk("\n\t\t");
	}
	printk("\n");

	return;
}
#else
#define ATSE_DEBUG_PRINT_DESC(this_desc)
#endif /* #ifdef ATSE_DEBUG_DESC */



#ifdef ATSE_DEBUG_TX_DATA
#define ATSE_DEBUG_PRINT_TX_DATA(data, len, align_offset) atse_debug_print_tx_data(data, len, align_offset, __FILE__, __LINE__)
static int tx_data_num = 0;
static void atse_debug_print_tx_data(char *data, int len, int align_offset, char *file, int line)
{
	int i;

	++tx_data_num;
	printk("----ATSE_DEBUG:%s:%d:kernel tx_data: num = %d, addr = 0x%p, len = %d, align_offset = %d\n",
	       file, line, tx_data_num, data, len, align_offset);
	for (i = 0; i < len; i++) {
		printk(" %02x", ((unsigned char *)data)[i]);
		
		if ((i+1) % 8 == 0)
			printk(" ");

		if ((i+1) % 16 == 0)
			printk("\n");
	}
	printk("\n\n");
}

#else
#define ATSE_DEBUG_PRINT_TX_DATA(data, len, align_offset)
#endif /* #ifdef ATSE_DEBUG_TX_DATA */


#ifdef ATSE_DEBUG_RX_DATA
#define ATSE_DEBUG_PRINT_RX_DATA(this_desc, this_data) atse_debug_print_rx_data(this_desc, this_data, __FILE__, __LINE__)
static void atse_debug_print_rx_data(struct atse_sgdma_desc *rx_desc, unsigned char *rx_data, char *file, int line)
{
	int j;
	int rx_datalen;
	
	printk("----ATSE_DEBUG%s:%d:ATSE_GET_SGDMA_DESC_STATUS(rx_desc) = %0x\n", 
	       __FILE__, __LINE__, ATSE_GET_SGDMA_DESC_STATUS(rx_desc));

	printk("----ATSE_DEBUG:%s:%d:rx_desc = 0x%p\n", __FILE__, __LINE__, rx_desc);

	rx_datalen = rx_desc->bytes_transferred;

	printk("----ATSE_DEBUG:%s:%d:rx_datalen = %d\n", file, line, rx_datalen);
	
	printk("----ATSE_DEBUG:%s:%d:rx_data addr = 0x%p\n", __FILE__, __LINE__, rx_data);
	printk("----ATSE_DEBUG:%s:%d:rx_data = \n", __FILE__, __LINE__);
	for (j = 0; j < rx_datalen; j++) {
		printk(" %02x", rx_data[j]);
		if ((j+1) % 16  == 0)
			printk("\n");
		else if ((j+1) % 16 == 8)
			printk(" ");
		
	}
	printk("\n");


	return;
}
#else
#define ATSE_DEBUG_PRINT_RX_DATA(this_desc, this_data)
#endif /* #ifdef ATSE_DEBUG_RX_DATA */



#ifdef ATSE_DEBUG_FUNC_TRACE_ENTER
#define ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER() printk("----%s:%d: ENTER %s()\n", __FILE__, __LINE__, __FUNCTION__)
#else
#define ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER()
#endif

#ifdef ATSE_DEBUG_FUNC_TRACE_EXIT
#define ATSE_DEBUG_PRINT_FUNC_TRACE_EXIT()  printk("----%s:%d: EXIT  %s()\n", __FILE__, __LINE__, __FUNCTION__)
#else
#define ATSE_DEBUG_PRINT_FUNC_TRACE_EXIT()
#endif




/* store this information for the driver.. */
struct atse_board_priv {
	spinlock_t tx_lock;
	spinlock_t rx_lock;
	struct atse_phy_device *phy_dev;
//	int      link_is_full_dup;
//	int      link_is_1000;
	int      link_is_mdix; /* crossover cable */
	int        tx_pkt_cnt;
	int        queue_pkt_len;
 	struct net_device_stats stats;
 	struct sk_buff *pending_tx_skb;
	struct resource *res_desc_mem;
	struct resource *res_sgdma_rx_mem;
	struct resource *res_sgdma_tx_mem;
	struct resource *res_sgdma_rx_irq;
	struct resource *res_sgdma_tx_irq;
};


static irqreturn_t atse_sgdma_tx_irq_handler(int irq, void *dev_id)
{
	struct net_device *dev;
	struct atse_board_priv  *bd_priv;

	dev = (struct net_device *)dev_id;
	bd_priv = netdev_priv(dev);
	spin_lock(&bd_priv->tx_lock);

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

	ATSE_DEBUG_PRINT_DESC(tx_desc);

	ATSE_SET_SGDMA_TX_CONTROL(ATSE_SGDMA_CONTROL_CLEAR_INTERRUPT_BIT);

	ATSE_CLEAR_SGDMA_TX_CONTROL();
	ATSE_CLEAR_SGDMA_TX_STATUS();

	/* a transmission is over: free the skb */
	bd_priv->stats.tx_packets++;
	bd_priv->stats.tx_bytes += g_tx_bytes_transferred;
	dev_kfree_skb_irq(bd_priv->pending_tx_skb);
	bd_priv->pending_tx_skb = NULL;

	spin_unlock(&bd_priv->tx_lock);

	return IRQ_HANDLED;
}


static int atse_sgdma_tx_init(struct net_device *dev)
{
	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

	g_tx_desc->status &= ~ATSE_SGDMA_DESC_STATUS_TERMINATED_BY_EOP_BIT;

	ATSE_SET_SGDMA_TX_CONTROL(ATSE_SGDMA_CONTROL_SW_RESET_BIT);
	ATSE_SET_SGDMA_TX_CONTROL(ATSE_SGDMA_CONTROL_CLEAR_INTERRUPT_BIT);
	ATSE_CLEAR_SGDMA_TX_STATUS();
	ATSE_CLEAR_SGDMA_TX_CONTROL();

	return 0;
}

/*
 * atse_mac_rcv() function assumes that the data passed in do not have
 * any issues with cached or uncached data.
 *
 */

static void atse_mac_rcv(struct net_device *ndev, struct atse_sgdma_desc *rx_desc, unsigned char *rx_data_buffer)
{
	struct sk_buff *skb;
	struct atse_board_priv *bd_priv = netdev_priv(ndev);
	int pktdatalen;

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();
	
	if (!(rx_desc->status & ATSE_SGDMA_DESC_STATUS_TERMINATED_BY_EOP_BIT)) {
		printk("ATSE driver:%s:%d: not ATSE_SGDMA_DESC_STATUS_TERMINATED_BY_EOP_BIT\n",
		       __FILE__, __LINE__);
		goto out_atse_mac_rcv;
	}

	rx_desc->status &= ~ ATSE_SGDMA_DESC_STATUS_TERMINATED_BY_EOP_BIT;

	if ( (rx_desc->status & ( ATSE_SGDMA_DESC_STATUS_E_CRC_BIT |
				ATSE_SGDMA_DESC_STATUS_E_PARITY_BIT |
				ATSE_SGDMA_DESC_STATUS_E_OVERFLOW_BIT |
				ATSE_SGDMA_DESC_STATUS_E_SYNC_BIT |
				ATSE_SGDMA_DESC_STATUS_E_UEOP_BIT |
				ATSE_SGDMA_DESC_STATUS_E_MEOP_BIT |
				ATSE_SGDMA_DESC_STATUS_E_MSOP_BIT
		      ) ) != 0) {
		printk("ATSE driver:%s:%d: rx_desc->status ERROR\n", __FILE__, __LINE__);
		goto out_atse_mac_rcv;
	}
    
	pktdatalen = rx_desc->bytes_transferred;
	/* saw adding extra 2 bytes from the device driver book, don't know why */
	skb = dev_alloc_skb(pktdatalen + 6);
	if(!skb) {
		if (printk_ratelimit())
			printk(KERN_NOTICE "ATSE driver:%s:%d:%s(): low on mem - packet dropped\n",
			       __FILE__, __LINE__, __FUNCTION__);
		bd_priv->stats.rx_dropped++;
		goto out_atse_mac_rcv;
	}
	/*
	 * Call skb_reserve() to align IP header to 32bits, don't konw
	 * why.  Without this call, kernel woun't work for the IP data.
	 */
	skb_reserve(skb, 2);

	/* sync the cache memory with the uncached memory ???? */
	memcpy(skb_put(skb, pktdatalen), rx_data_buffer, pktdatalen);
	
	/* write metadata, and then pass to the receive level */
	skb->dev = ndev;
	skb->protocol = eth_type_trans(skb, ndev);
	bd_priv->stats.rx_packets++;
	bd_priv->stats.rx_bytes += pktdatalen;
	netif_rx(skb);
	
out_atse_mac_rcv:

	ATSE_DEBUG_PRINT_FUNC_TRACE_EXIT();

	return;
}


static int atse_sgdma_rx_enable_async_read(struct atse_sgdma_desc *rx_desc)
{
	int timeout;

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

	/*  Make sure SGDMA controller is not busy from a former command */
	timeout = 0;

	while ( (ATSE_GET_SGDMA_RX_STATUS() & ATSE_SGDMA_STATUS_BUSY_BIT) ) {
		if(timeout++ == ATSE_SGDMA_BUSY_TIMEOUT_COUNT) {
			printk("ATSE driver:%s:%d:Timeout\n", __FILE__, __LINE__);
			return -EBUSY;
		}
	}

	ATSE_CLEAR_SGDMA_RX_STATUS();

	/* flush the descriptor out of the cache */
	dcache_push((unsigned long)rx_desc, sizeof(struct atse_sgdma_desc));

	/* Point the controller at the descriptor */
	ATSE_SET_SGDMA_RX_WITH_DESC((u32)rx_desc);

	/*
	 * Set up agdma controller to:
	 *  - Run
	 *  - Stop on an error with any particular descriptor
	 *  - Include any control register bits registered with along with
	 *    the callback routine (effectively, interrupts are controlled
	 *    via the control bits set during callback-register time).
	 */
	
	ATSE_SET_SGDMA_RX_CONTROL(
		ATSE_SGDMA_CONTROL_IE_CHAIN_COMPLETED_BIT |
		ATSE_SGDMA_CONTROL_IE_GLOBAL_BIT          |
		ATSE_SGDMA_CONTROL_RUN_BIT                |
		ATSE_SGDMA_CONTROL_STOP_DMA_ER_BIT        |
		ATSE_GET_SGDMA_RX_CONTROL()
		);
	
	return 0;
}


static int atse_sgdma_rx_prepare_desc(void)
{
	struct atse_sgdma_desc *cur_desc = g_rx_desc;
	struct atse_sgdma_desc *next_desc = g_rx_desc_next;
	u32 * clear_uncached_data_addr;

	/*
	 * this is a starting write_address, and make sure to
	 * clear the uncache-bit (bit 31) before passing to
	 * SGDMA controller
	 */
	/* ioremap() requires buffer_size as the 2nd argument, but it is not being used inside anyway, so put 1234 */
	clear_uncached_data_addr = ioremap_fullcache((unsigned long)g_rx_data_buffer_ptr, 1234);

	/* initialize the dscriptor structure data */
	
	memset(cur_desc, 0, sizeof(struct atse_sgdma_desc));
	memset(next_desc, 0, sizeof(struct atse_sgdma_desc));
	
	/* cur_desc->read_addr = 0; */
	cur_desc->write_addr = clear_uncached_data_addr;
	cur_desc->next_desc = (u32 *) next_desc;
	/* cur_desc->read_addr_pad  = 0x0; */
	/* cur_desc->next_desc_pad  = 0x0; */
	/* cur_desc->bytes_to_transfer = 0; */
 	/* cur_desc->bytes_transferred = 0; */
	/* cur_desc->status = 0x0; */
	/* SGDMA burst not currently supported */
	/* cur_desc->read_burst_size = 0; */
	/* cur_desc->write_burst_size = 0; */
	/* set the descriptor control as follows */
	cur_desc->control = ATSE_SGDMA_DESC_CONTROL_OWNED_BY_HW_BIT;
	
	return 0;
}


static int atse_sgdma_rx_init(struct net_device *ndev)
{
	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

	
	ATSE_SET_SGDMA_RX_CONTROL(ATSE_SGDMA_CONTROL_SW_RESET_BIT);
	ATSE_SET_SGDMA_RX_CONTROL(ATSE_SGDMA_CONTROL_CLEAR_INTERRUPT_BIT);
	ATSE_CLEAR_SGDMA_RX_STATUS();
	ATSE_CLEAR_SGDMA_RX_CONTROL();
	
	atse_sgdma_rx_prepare_desc();

	return 0;
}


static int atse_mac_init(struct net_device *ndev)
{
	u32 mac_tx_cmd_stat;
	u32 mac_rx_cmd_stat;
	u32 not_bits_group;
	int x;
	volatile u32 dat;
	struct atse_board_priv  *bd_priv = netdev_priv(ndev);


	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();


	ATSE_SET_MAC_CMD_CONFIG(ATSE_MAC_CC_SW_RESET_BIT |
				ATSE_MAC_CC_TX_ENA_BIT   |
				ATSE_MAC_CC_RX_ENA_BIT
		);

	x = 0;
	while(ATSE_GET_MAC_CMD_CONFIG() & ATSE_MAC_CC_SW_RESET_BIT) {
		if( x++ > 10000 )
			break;
	}

	if(x >= 10000)
		printk("ATSE driver:%s:%d:MAC SW reset bit never cleared!\n", __FILE__, __LINE__); \


	if ((ATSE_GET_MAC_CMD_CONFIG() & (ATSE_MAC_CC_TX_ENA_BIT | ATSE_MAC_CC_RX_ENA_BIT) )  != 0)
		printk("ATSE driver:%s:%d:WARN: RX/TX not disabled after reset... missing PHY clock?\n", __FILE__, __LINE__);


	/* Initialize MAC registers */
	ATSE_SET_MAC_FRM_LEN(1518);
	ATSE_SET_MAC_RX_ALMOST_EMPTY(8);
	ATSE_SET_MAC_RX_ALMOST_FULL(8);
	ATSE_SET_MAC_TX_ALMOST_EMPTY(8);
	ATSE_SET_MAC_TX_ALMOST_FULL(3);
	ATSE_SET_MAC_TX_SECTION_EMPTY(1024 - 16);
	ATSE_SET_MAC_TX_SECTION_FULL(0);
	ATSE_SET_MAC_RX_SECTION_EMPTY(1024 - 16);
	ATSE_SET_MAC_RX_SECTION_FULL(0);


	 /* 
	  * Some required settings:
	  *
	  * 1. Do not let MAC to use 16-bit shift in TX, since the
	  *    data provided by the kernel seems aligned already
	  *
	  * 2. Do not let MAC to omit CRC, since the kernel does not
	  *    provide it in this case.
	  */
	mac_tx_cmd_stat = ATSE_GET_MAC_TX_CMD_STAT();
	not_bits_group = ~(
		ATSE_MAC_TX_CMD_STAT_TXSHIFT16_BIT |
		ATSE_MAC_TX_CMD_STAT_OMIT_CRC_BIT
		);
	ATSE_SET_MAC_TX_CMD_STAT(mac_tx_cmd_stat & not_bits_group);

	mac_rx_cmd_stat = ATSE_GET_MAC_RX_CMD_STAT();
	not_bits_group = ~(
		ATSE_MAC_RX_CMD_STAT_RXSHIFT16_BIT |
		0x0
		);
	ATSE_SET_MAC_RX_CMD_STAT(mac_tx_cmd_stat & not_bits_group);
	/* enable mac */
	dat = ATSE_MAC_CC_TX_ENA_BIT                |
		ATSE_MAC_CC_RX_ENA_BIT              |
		ATSE_MAC_CC_RX_ERR_DISCD_BIT        |
		0x0;

	if (bd_priv->phy_dev->get_link_speed() == 1000)
		dat |= ATSE_MAC_CC_ETH_SPEED_BIT;


	if (!bd_priv->phy_dev->link_is_full_dup())
		dat |= ATSE_MAC_CC_HD_ENA_BIT;

	ATSE_SET_MAC_CMD_CONFIG(dat);
	
	return 0;
}



static void atse_reset_and_enable(struct net_device *ndev)
{
	int i;

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

	/* 
	 * Reset is to do the folowing 4 steps:
	 *
	 * 1. reset SGDMA_TX
	 * 2. reset SGDMA_RX
	 * 3. reset PHY
	 * 4. reset MAC
	 *
	 */
	/* clear SGDMA descriptors */
	for (i = 0; i < ATSE_TOTAL_SGDMA_DESC_MEM / BYTES_IN_WORD; i++)
		writel(0x0, ATSE_SGDMA_DESC_MEM_BASE + i *BYTES_IN_WORD);

	/* reset sgdma */
	atse_sgdma_tx_init(ndev);
	atse_sgdma_rx_init(ndev);
	atse_sgdma_rx_enable_async_read(g_rx_desc);
			
	/* reset mac */
	atse_mac_init(ndev);

	return;
}



static irqreturn_t atse_sgdma_rx_irq_handler(int irq, void *dev_id)
{
	struct net_device *ndev;
	struct atse_board_priv  *bd_priv;
	struct atse_sgdma_desc *desc;
	struct atse_sgdma_desc *rx_desc;
	unsigned char *uncached_rx_data_buffer_ptr = g_rx_data_buffer;

	ndev = (struct net_device *)dev_id;
	bd_priv = netdev_priv(ndev);
	spin_lock(&bd_priv->rx_lock);

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

	/* First the firs to clear the pending interrupts */
	ATSE_SET_SGDMA_RX_CONTROL(ATSE_SGDMA_CONTROL_CLEAR_INTERRUPT_BIT);

	desc = (struct atse_sgdma_desc *)ATSE_SGDMA_DESC_MEM_BASE;
	rx_desc = &desc[ATSE_SGDMA_RX_FIRST_DESC_OFFSET];

	/* 
	 * since both rx_desc and rx_data were just written by the hw controller, they may
	 * not be in the cache yet, so use the uncacched access
	 *
	 */
	uncached_rx_data_buffer_ptr = (unsigned char *)ioremap_nocache((unsigned long)uncached_rx_data_buffer_ptr, 1234);
	rx_desc = (struct atse_sgdma_desc *) ioremap_nocache((unsigned long)rx_desc, 1234);
	ATSE_DEBUG_PRINT_RX_DATA(rx_desc,uncached_rx_data_buffer_ptr);

	atse_mac_rcv(ndev, rx_desc, uncached_rx_data_buffer_ptr);
	/* done with received packet, prepare for the next receiving enevent */
	ATSE_CLEAR_SGDMA_RX_CONTROL();
	ATSE_ENABLE_SGDMA_RX_INT();

	if (!(ATSE_GET_SGDMA_RX_STATUS() & ATSE_SGDMA_STATUS_CHAIN_COMPLETED_BIT)) {
		printk("ATSE driver:%s:%d:CHAINE_COMPLETED is false\n", __FILE__, __LINE__);
	}

	atse_sgdma_rx_prepare_desc();

	
	atse_sgdma_rx_enable_async_read(rx_desc);

	spin_unlock(&bd_priv->rx_lock);

	return IRQ_HANDLED;
} /* end of atse_sgdma_rx_irq_handler() */

static int atse_phy_init(struct net_device *ndev)
{
	unsigned int stat;
	int count = 0;

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

	/* see if need reset PHY */
	if ((ATSE_GET_PHY_MDIO_STATUS() &
	     ATSE_PHY_MDIO_CONTROL_AUTO_NEG_COMPLETE_BIT) > 0)
	{
		/* auto negotiation is complete, so no reset needed */
		return 0;
	} else {
		/* reset now */
		ATSE_SET_PHY_MDIO_CONTROL(
			ATSE_PHY_MDIO_CONTROL_RESET_BIT           |
			ATSE_PHY_MDIO_CONTROL_AUTO_NEGO_ENA_BIT   |
			0x0
			);
	
		stat = ATSE_GET_PHY_MDIO_STATUS();
		if ((stat & ATSE_PHY_MDIO_CONTROL_AUTO_NEG_COMPLETE_BIT) == 0) {
			printk("ATSE: Waiting on PHY link ......\n");
			while ((ATSE_GET_PHY_MDIO_STATUS() & 
				ATSE_PHY_MDIO_CONTROL_AUTO_NEG_COMPLETE_BIT) == 0) {
				if (count > ATSE_PHY_AUTONEG_TIMEOUT_COUNT) {
					printk("ATSE:PHY Auto negotiation faild, continue anyway ... \n");
					break;
				}
				mdelay(1000);
				++count;
			} /* end while() */
		}
	}

	return 0;
}


static int atse_open(struct net_device *ndev)
{
	struct atse_board_priv *bd_priv = netdev_priv(ndev);
	int ret;
	int n;
	DECLARE_MAC_BUF(mac_buf);
	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

	atse_phy_init(ndev);

	/* initialize globle data */
	g_rx_desc      = &g_desc[ATSE_SGDMA_RX_FIRST_DESC_OFFSET];
	g_rx_desc_next = &g_desc[ATSE_SGDMA_RX_SECOND_DESC_OFFSET];

	g_tx_desc      = &g_desc[ATSE_SGDMA_TX_FIRST_DESC_OFFSET];
	g_tx_desc_next = &g_desc[ATSE_SGDMA_TX_SECOND_DESC_OFFSET];


	/* allocate memory for each of rx_data_buffer[i] */
	if (g_rx_data_buffer == NULL) {
		g_rx_data_buffer = kmalloc(ETH_DATA_LEN + 2 + 3, GFP_KERNEL);
		if (!g_rx_data_buffer) {
			if (printk_ratelimit( ))
				printk(KERN_NOTICE "ATSE driver:%s:%d:rx: low on mem - packet dropped\n", __FILE__, __LINE__);
			goto out_atse_open_error;
		}
	}

	g_rx_data_buffer_ptr = g_rx_data_buffer;
	/* make sure the addr is 32-bit aligned: JoeCamel, FIXME */
	n = (int)g_rx_data_buffer_ptr % 4;
	if (n  != 0) {
		g_rx_data_buffer_ptr += (4 - n);
		printk("++++++++ATSE driver:%s:%d:rx_data_buffer manually aligned here: remainder = %d\n", __FILE__, __LINE__, n);
	}


	/* register interrupt callback function for rx_sgdma controller */
	ret = request_irq(ATSE_SGDMA_RX_IRQ, atse_sgdma_rx_irq_handler, IRQF_DISABLED, "atse ether driver rx", ndev);
	if(ret) {
		printk("********ATSE driver:%s:%d:can't request assigned irq=%d\n", __FILE__, __LINE__, ATSE_SGDMA_RX_IRQ);
	}

	/* register interrupt callback function for tx_sgdma controller */
	ret = request_irq(ATSE_SGDMA_TX_IRQ, atse_sgdma_tx_irq_handler, IRQF_DISABLED, "atse ether dirver tx", ndev);
	if(ret) {
		printk("********ATSE driver:%s:%d:can't request assigned irq=%d\n", __FILE__, __LINE__, ATSE_SGDMA_TX_IRQ);
	}


	/* reset the hardware */
	atse_reset_and_enable(ndev);
	netif_start_queue(ndev);

	printk("%s up, link speed %d, %s duplex",
	       ndev->name, bd_priv->phy_dev->get_link_speed(),
	       bd_priv->phy_dev->link_is_full_dup() == 1? "full":"half");
	if (bd_priv->link_is_mdix)
		printk(", crossover\n");
	printk(", eth hw addr %s", print_mac(mac_buf, ndev->dev_addr));
	
	printk("\n");

 
	return 0;
out_atse_open_error:

	return -1;
}


static int atse_close(struct net_device *ndev)
{

	struct atse_board_priv  *bd_priv = netdev_priv(ndev);

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

	netif_stop_queue(ndev);

	ATSE_SET_SGDMA_RX_CONTROL(ATSE_GET_SGDMA_RX_CONTROL() &
				  ~ATSE_SGDMA_CONTROL_CLEAR_INTERRUPT_BIT);

	ATSE_CLEAR_SGDMA_RX_CONTROL();


	ATSE_SET_SGDMA_TX_CONTROL(ATSE_GET_SGDMA_TX_CONTROL() &
				  ~ATSE_SGDMA_CONTROL_CLEAR_INTERRUPT_BIT);

	ATSE_CLEAR_SGDMA_TX_CONTROL();

	ATSE_SET_PHY_POWER_DOWN();

	if (bd_priv->pending_tx_skb) {
		dev_kfree_skb(bd_priv->pending_tx_skb);
		bd_priv->pending_tx_skb = NULL;
	}

	/* de-register irq for both SGDMA-TX and SGDMA-RX controllers */
	free_irq(ATSE_SGDMA_TX_IRQ, ndev);
	free_irq(ATSE_SGDMA_RX_IRQ, ndev);

	if (g_rx_data_buffer) {
		kfree(g_rx_data_buffer);
		g_rx_data_buffer     = NULL;
		g_rx_data_buffer_ptr = NULL;
	}

	printk("%s down\n", ndev->name);

	return 0;
}

#define ATSE_MARVELL_PHY_ID_88E1111 0x01410cc2
#define ATSE_MARVELL_PHY_ID_88E1145 0x01410dc2
#define ATSE_NATIONAL_PHY_ID_83848  0x20005c90  /* National 83848, 10/100 */
#define ATSE_NATIONAL_PHY_ID_83865  0x20005c7a  /* National DP83865 */
#define ATSE_TDK_PHY_ID_78Q2120C    0x000e70c5  /* TDK 78Q2120 10/100 PHY */



/* PHY ID */
/* Marvell PHY on PHYWORKX board */
enum {
    MV88E1111_OUI       = 0x005043,
    MV88E1111_MODEL     = 0x0c,
    MV88E1111_REV       = 0x02
};

/* Marvell Quad PHY on PHYWORKX board */
enum {
    MV88E1145_OUI       = 0x005043,
    MV88E1145_MODEL     = 0x0d,
    MV88E1145_REV       = 0x02
};

/* National PHY on PHYWORKX board */
enum {
    DP83865_OUI       = 0x080017,
    DP83865_MODEL     = 0x07,
    DP83865_REV       = 0x10
};

/* National 10/100 PHY on PHYWORKX board */
enum {
    DP83848C_OUI       = 0x080017,
    DP83848C_MODEL     = 0x09,
    DP83848C_REV       = 0x00
};



/* Marvell PHY 88E1111 */

static int atse_phy_marvell_88E1111_get_link_speed(void)
{
	unsigned int stat;

	stat = ATSE_GET_MVLPHY_LINK_STATUS();
	if (ATSE_MVLPHY_IS_1000(stat)) {
		return 1000;
	}

	return 100;
}

static int atse_phy_marvell_88E1111_link_is_full_dup(void)
{
	unsigned int stat;
	stat = ATSE_GET_MVLPHY_LINK_STATUS();
	return (ATSE_MVLPHY_IS_FULL_DUP(stat));
}


/*  National 83848, 10/100 */

static int atse_phy_national_83848_get_link_speed(void)
{
	/* Reference: National DP83848C datasheet */
	unsigned int stat;

	/* register 0x10 is an Extended Register for PHY Status Register */
	stat = ATSE_READ_PHY_MDIO_REG(0x10);
	/* Bit 1 is for Speed Status */
	if ((stat & (0x1 << 1)) == 0)
		return 100;
	else
		return 100;
}

static int atse_phy_national_83848_link_is_full_dup(void)
{
	/* Reference: National DP83848C datasheet */
	unsigned int stat;

	/* register 0x10 is an Extended Register for PHY Status Register */
	stat = ATSE_READ_PHY_MDIO_REG(0x10);
	/* Bit 2 is for Duples Status  */
	if ((stat & (0x1 << 2)) == 0)
		return 0; /* Half duplex mode */
	
	/* Full duplex mode */
	return 1;
}

static int atse_phy_national_83848_link_is_established(void)
{
	/* Reference: National DP83848C datasheet */
	unsigned int stat;

	/* register 0x10 is an Extended Register for PHY Status Register */
	stat = ATSE_READ_PHY_MDIO_REG(0x10);
	/* Bit 0 is for Duples Status  */
	if ((stat & (0x1 << 0)) == 0)
		return 0; /* not established */
	
	/* established */
	return 1;
}


/*  National 83865 */

static int atse_phy_national_83865_get_link_speed(void)
{
	/* Reference: National DP83865 datasheet */
	unsigned int stat;

	/* register 0x10 is an Extended Register for PHY Status Register */
	stat = ATSE_READ_PHY_MDIO_REG(0x10);
	/* Bit 3 is for Speed Status */
	if ((stat & (0x1 << 3)) == 0)
		return 100;
	else
		return 100;
}

static int atse_phy_national_83865_link_is_full_dup(void)
{
	/* Reference: National DP83865 datasheet */
	unsigned int stat;

	/* register 0x10 is an Extended Register for PHY Status Register */
	stat = ATSE_READ_PHY_MDIO_REG(0x10);
	/* Bit 1 is for Duples Status  */
	if ((stat & (0x1 << 1)) == 0)
		return 0; /* Half duplex mode */
	
	/* Full duplex mode */
	return 1;
}

static int atse_phy_national_83865_link_is_established(void)
{
	/* Reference: National DP83865 datasheet */
	unsigned int stat;

	/* register 0x10 is an Extended Register for PHY Status Register */
	stat = ATSE_READ_PHY_MDIO_REG(0x10);
	/* Bit 2 is for Duples Status  */
	if ((stat & (0x1 << 2)) == 0)
		return 0; /* not established */
	
	/* established */
	return 1;
}

/*  TDK 78Q2120C PHY */

static int atse_phy_tdk_78Q2120_get_link_speed(void)
{
	/* Reference: TDK 78Q2120C datasheet */
	unsigned int stat;

	/* Using MR1 (Status Register) to check ANEGC */
	stat = ATSE_READ_PHY_MDIO_REG(0x01);
	/* MR1.5 ANEGC: Auto-Negotiation Complete */
	if((stat & (0x01 << 5)) == 1) {

	  /* Using MR0 (Control Register) */
	  stat = ATSE_READ_PHY_MDIO_REG(0x00);
	  /* Bit 13 is for Speed Selection */
	  if ((stat & (0x1 << 13)) == 0)
	    return 10;
	  else
	    return 100;

	}

	return 100; // Default to 100MB

}

static int atse_phy_tdk_78Q2120_link_is_full_dup(void)
{
	/* Reference: TDK 78Q2120C datasheet */
	unsigned int stat;

	/* Using MR1 (Status Register) to check ANEGC */
	stat = ATSE_READ_PHY_MDIO_REG(0x01);
	/* MR1.5 ANEGC: Auto-Negotiation Complete */
	if((stat & (0x01 << 5)) == 1) {

	  /* Using MR0 (Control Register) */
	  stat = ATSE_READ_PHY_MDIO_REG(0x00);
	  /* Bit 13 is for Speed Selection */
	  if ((stat & (0x1 << 8)) == 0)
	    return 0;
	  else
	    return 1;

	}

	return 1; // Default to Full-Duplex
}

static int atse_phy_tdk_78Q2120_link_is_established(void)
{
	/* Reference: TDK 78Q2120C datasheet */
	unsigned int stat;

	/* Using MR1 (Status Register) to check LINK */
	stat = ATSE_READ_PHY_MDIO_REG(0x01);
	/* MR1.5 ANEGC: Auto-Negotiation Complete */
	if((stat & (0x01 << 2)) == 0)
	    return 0; /* not established / link is not ready */
	else
	    return 1; /* ok */
	    
}


static struct atse_phy_device atse_phy_dev_list[] = {
	[0] = {
		.name = "Marvell 88E1111 PHY",
		.phy_id = ATSE_MARVELL_PHY_ID_88E1111,
		.link_stat_reg_num = 0x11,

		.get_link_speed   = atse_phy_marvell_88E1111_get_link_speed,
		.link_is_full_dup = atse_phy_marvell_88E1111_link_is_full_dup,
	},

	[1] = {
		.name = "Marvell 88E1145 PHY",
		.phy_id = ATSE_MARVELL_PHY_ID_88E1145,
		.link_stat_reg_num = 0x11,
		/* functions are compatible with 88E111, so reuse here */
		.get_link_speed   = atse_phy_marvell_88E1111_get_link_speed,
		.link_is_full_dup = atse_phy_marvell_88E1111_link_is_full_dup,
	},

	[2] = {
		.name = "National DP83848C PHY",
		.phy_id = ATSE_NATIONAL_PHY_ID_83848,
		.link_stat_reg_num = 0x10,

		.get_link_speed      = atse_phy_national_83848_get_link_speed,
		.link_is_full_dup    = atse_phy_national_83848_link_is_full_dup,
		.link_is_established = atse_phy_national_83848_link_is_established,

	},

	[3] = {
		.name = "National DP83865 PHY",
		.phy_id = ATSE_NATIONAL_PHY_ID_83865,
		.link_stat_reg_num = 0x10,

		.get_link_speed      = atse_phy_national_83865_get_link_speed,
		.link_is_full_dup    = atse_phy_national_83865_link_is_full_dup,
		.link_is_established = atse_phy_national_83865_link_is_established,

	},

	[4] = {
		.name = "TDK 78Q2120 PHY",
		.phy_id = ATSE_TDK_PHY_ID_78Q2120C,
		.link_stat_reg_num = 0x01,

		.get_link_speed      = atse_phy_tdk_78Q2120_get_link_speed,
		.link_is_full_dup    = atse_phy_tdk_78Q2120_link_is_full_dup,
		.link_is_established = atse_phy_tdk_78Q2120_link_is_established,

	}
};


static struct atse_phy_device *
get_phy_dev(u32 phy_id_1, u32 phy_id_2)
{
	int num_of_phys;
	int i;
	int oui;
	int model_num;
	int rev_num;

	struct atse_phy_device *phy_dev;

	/* calculate OUI, model number, and revision number from phy_id_1 and phy_id_2 */
	oui = (phy_id_1 << 6) | ((phy_id_2 >> 10) & 0x3F);
	model_num = ((phy_id_2 >> 4)) & 0x3F;
	rev_num = phy_id_2 & 0x0F;

		     

	num_of_phys = sizeof(atse_phy_dev_list) / sizeof(atse_phy_dev_list[0]);
	for (i = 0; i < num_of_phys; i++) {
		phy_dev = &(atse_phy_dev_list[i]);
		if ((phy_dev->phy_id >> 16) == phy_id_1 && 
		    ((phy_dev->phy_id & 0xFFFF) == phy_id_2)) {
			/* found a known phy */
			goto found_phy_dev;
		}
	}
	
	printk("ATSE Driver:%s:%d: failed to find phy dev: phy_id_1 = 0x%x, phy_id_2 = 0x%x\n", __FILE__, __LINE__, phy_id_1, phy_id_2);
	return NULL;

found_phy_dev:
	ATSE_DEBUG_PRINT_PHY_DEV(oui, model_num, rev_num, phy_dev);
	return phy_dev;
}


/*
 *------------------------------------------------------------------------------
 *  Side Effect:  Write detected PHY address to MAC register mdio_addr_0.
 *
 *------------------------------------------------------------------------------
 */
static struct atse_phy_device *
atse_detect_phy(void)
{
	int phy_addr;
	int phy_id_1;
	int phy_id_2;
	int found_phy;
	
	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

	/* Detect if a valid phy exists */
	/* probe Marvel PHY MDIO address by blindliy scanning all the
	 * addresses until a valid PHY ID is abtained.  When a valid
	 * PHY is detected, the MAC register for mdio_addr_0 will be
	 * written with the valid PHY address from which a valid PHY
	 * ID was detected.
	 */
	
	found_phy = 0;
	for (phy_addr = 0x0; phy_addr < 0xFF; phy_addr++) {
		/* set the phy address to the mac map data */
		writel(phy_addr, ATSE_MAC_REG_MDIO_ADDR_0);
		phy_id_1 = readl(ATSE_MAC_REG_MDIO_SPACE_0 + ATSE_PHY_ID_1_OFFSET);
		phy_id_2 = readl(ATSE_MAC_REG_MDIO_SPACE_0 + ATSE_PHY_ID_2_OFFSET);
		if (phy_id_1 != phy_id_2) {
			found_phy = 1;
			break;
		}
	}

	if (!found_phy) {
		/* failed to detect phy */
		return NULL;
	}

	return get_phy_dev(phy_id_1, phy_id_2);
}


static int atse_hw_send_data(char *data, int len, struct net_device *ndev)
{
	/* This function is to wirte data to TX FIFO */

	int timeout = 0;
	struct atse_sgdma_desc *desc = (struct atse_sgdma_desc *)ATSE_SGDMA_DESC_MEM_BASE;
	struct atse_sgdma_desc *cur_desc = &(desc[ATSE_SGDMA_TX_FIRST_DESC_OFFSET]);
	struct atse_sgdma_desc *next_desc = &(desc[ATSE_SGDMA_TX_SECOND_DESC_OFFSET]);

#ifndef ATSE_SGDMA_DO_UNALIGNED_TRANSFER
	int align_offset;
#endif /* ATSE_SGDMA_DO_UNALIGNED_TRANSFER */

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

#ifndef ATSE_SGDMA_DO_UNALIGNED_TRANSFER
	/* Since sgdma hardware can not do unaligned transfer,
	 * software needs to do extra for 32-bit alignment with the
	 * data pointer.
	 */

	/* Aligment check */
	/*check if the first 2 LSBs are all zeros */
	align_offset  = (u32)data & 0x02;
	if (align_offset != 0) {
		data -= 2;
		len += 2;
		ATSE_SET_MAC_TX_SHIFT16_ON();
	} else {
		ATSE_SET_MAC_TX_SHIFT16_OFF();
	}
#endif /* not ATSE_SGDMA_DO_UNALIGNED_TRANSFER */

 	ATSE_DEBUG_PRINT_TX_DATA(data, len, align_offset);

	/* clear bit 31, if any, to get rid of uncashed flag.  Not portable, FIXMEJOE*/
	data = (char *)ioremap_fullcache((unsigned long)data, 1234);
	/* flush data out of cache */
	dcache_push((unsigned long)data, len);

	/* prepare mem-to-stream memory buffer descriptores */
	/*
	 * Mark the "next" descriptor as "not" owned by hardware. This
	 * prevents The SGDMA controller from continuing to process the
	 * chain. This is done as a single IO write to bypass cache, without
	 * flushing the entire descriptor, since only the 8-bit descriptor
	 * status must be flushed.
	 */
	memset(cur_desc, 0, sizeof(struct atse_sgdma_desc));
	memset(next_desc, 0, sizeof(struct atse_sgdma_desc));
	next_desc->control &= ~ATSE_SGDMA_DESC_CONTROL_OWNED_BY_HW_BIT;

	/* now assemble the descriptor */
	cur_desc->read_addr = (u32 *) data;
	cur_desc->write_addr = NULL; /* N/A for mem-stream interface */
	cur_desc->next_desc = (u32 *)next_desc;
	/* cur_desc->read_addr_pad = 0x0; */
	/* cur_desc->write_addr_pad = 0x0; */
	/* cur_desc->next_desc_pad = 0x0; */
	cur_desc->bytes_to_transfer = len;
	/* cur_desc->bytes_transferred = 0; */
	/* cur_desc->read_burst_size = 0; */
	/* cur_desc->write_burst_size = 0; */

	/*
	 * Set the descriptor control block as follows:
	 * - Set "owned by hardware" bit
	 * - Set "generte EOP" bit
	 * - Unset the "read from fixed address" bit
	 * - Set the "write to fixed address bit (which serves
	 *   serves as a "generate SOP" control bit in memory-to-stream mode).
	 * - Unet the 4-bit atlantic channel
	 *
	 * Note that for the unset part, nothing to do because of
	 * memset() has been called
	 *
	 * Note that this step is performed after all other descriptor information
	 * has been filled out so that, if the controller already happens to be
	 * pointing at this descriptor, it will not run (via the "owned by hardware"
	 * bit) until all other descriptor information has been set up.
	 */
	cur_desc->control = (
		ATSE_SGDMA_DESC_CONTROL_OWNED_BY_HW_BIT             |
		ATSE_SGDMA_DESC_CONTROL_GENERATE_EOP_BIT            |
		ATSE_SGDMA_DESC_CONTROL_WRITE_FIXED_ADDRESS_BIT
		);
	
	/* flush the cached descriptos to the physical memory */
	dcache_push((unsigned long)cur_desc, sizeof(struct atse_sgdma_desc));
	dcache_push((unsigned long)next_desc, sizeof(struct atse_sgdma_desc));

	/*
	 * Now to do Synchronous SGDMA copy from buffer memory into
	 * transmit FIFO. Waits until SGDMA has completed.  Raw
	 * function without any error checks.
	 */

	/* Make sure DMA controller is not busy from a former command
	 *  and TX is able to accept data timeout = 0;
	 */
	while ( ATSE_GET_SGDMA_TX_STATUS() & ATSE_SGDMA_STATUS_BUSY_BIT ) {
		if(timeout++ == ATSE_SGDMA_BUSY_TIMEOUT_COUNT) {
			printk("ATSE driver sgdma timedout:%s:%d\n", __FILE__, __LINE__);
			return -1;  /*  avoid being stuck here */
		}
	}

	ATSE_CLEAR_SGDMA_TX_CONTROL();
	ATSE_CLEAR_SGDMA_TX_STATUS();

	/* Point the controller at the descriptor */
	ATSE_SET_SGDMA_TX_WITH_DESC((u32)cur_desc);

	ATSE_DEBUG_PRINT_DESC(cur_desc);
	ATSE_DEBUG_PRINT_DESC(next_desc);

	/* Set up and Start SGDMA (blocking call) */
	/*
	 * Set up SGDMA controller to:
	 * - Disable interrupt generation ????
	 * - Run once a valid descriptor is written to controller
	 * - Stop on an error with any particular descriptor
	 */
	ATSE_SET_SGDMA_TX_CONTROL(
		ATSE_SGDMA_CONTROL_IE_CHAIN_COMPLETED_BIT |
		ATSE_SGDMA_CONTROL_IE_GLOBAL_BIT          |
		ATSE_SGDMA_CONTROL_RUN_BIT                |
		ATSE_SGDMA_CONTROL_STOP_DMA_ER_BIT        |
		ATSE_GET_SGDMA_TX_CONTROL()
		);

	/* wait for descriptor chain to complete */
	while ( (ATSE_GET_SGDMA_TX_STATUS() & ATSE_SGDMA_STATUS_BUSY_BIT) );

	/* Clear the Run Bit */
	ATSE_SET_SGDMA_TX_CONTROL( ATSE_GET_SGDMA_TX_CONTROL() &
				~ATSE_SGDMA_CONTROL_RUN_BIT );

	ATSE_DEBUG_PRINT_DESC(cur_desc);
	cur_desc = (struct atse_sgdma_desc *) ((u32)cur_desc | (((u32) 1) << 31));
	ATSE_DEBUG_PRINT_DESC(cur_desc);

	g_tx_bytes_transferred = cur_desc->bytes_transferred;

	ATSE_CLEAR_SGDMA_TX_STATUS();

	return 0;

}


static int atse_hard_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	int len;
	char shortpkt[ETH_ZLEN];
	char *data;

	struct atse_board_priv *bd_priv = netdev_priv(ndev);

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

	data = skb->data;
	len = skb->len;

	if (len < ETH_ZLEN) {
		memset(shortpkt, 0, ETH_ZLEN);
		memcpy(shortpkt, skb->data, skb->len);
		len = ETH_ZLEN;
		data = shortpkt;
	}
	ndev->trans_start = jiffies;  /* save the timestamp */

	/* remember the skb, so we can free it at interrupt time */
	bd_priv->pending_tx_skb = skb;

	/* hardware transmit now */
	atse_hw_send_data(data, len, ndev);



	return 0;
}

static void atse_timeout(struct net_device *ndev)
{
	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

	return;
}

/*
 * Get the current statistics.
 * This may be called with the card open or closed.
 */
static struct net_device_stats *atse_query_stats(struct net_device *ndev)
{
	struct atse_board_priv *bd_priv = netdev_priv(ndev);

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();


	return &bd_priv->stats;
}

static void atse_set_multicast_list(struct net_device *ndev)
{
	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

}


static void
atse_shutdown(struct net_device *ndev)
{
	struct atse_board_priv *bd_priv;
	struct sk_buff *pending_tx_skb;

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();
	printk("++++JoeCamel:%s:%d:%s: FIXME\n", __FILE__, __LINE__, __FUNCTION__);


	bd_priv = netdev_priv(ndev);
	/* no more interrupts for me */
	spin_lock_irq(&bd_priv->tx_lock);
	pending_tx_skb = bd_priv->pending_tx_skb;
	bd_priv->pending_tx_skb = NULL;
	spin_unlock_irq(&bd_priv->tx_lock);
	if (pending_tx_skb)
		dev_kfree_skb(pending_tx_skb);

	ATSE_SET_PHY_POWER_DOWN();

	/* RESET device */
	
	/* PHY RESET */

	/* Power-Down PHY */

	/* Disable all interrupt */

	/* Disable RX */

}



static int atse_probe(struct net_device *ndev)
{
	struct atse_board_priv *bd_priv;
	union {
		unsigned char c[4];
		unsigned int i;
	} mac_addr_0, mac_addr_1;

	DECLARE_MAC_BUF(mac_buf);
	int ret;
	struct atse_phy_device *phy_dev;

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

	/* 
	 * PHY Hardware has to be there: When detected, the phy
	 * address will be written in MAC register, a side effect.
	 */
	phy_dev = atse_detect_phy();
	if (!phy_dev) {
		printk("ATSE driver failed to detect an ether phy:%s:%d:%s()\n",
		       __FILE__, __LINE__, __FUNCTION__);
		return -1;
	}

	/* So a valid PHY exists, we assume that we have found netowrk hardware. */
	/* then shut everything down to save power */
	ATSE_SET_PHY_POWER_DOWN();

	/* Set up network device data structure*/

	/* Set up the board priv data */
	bd_priv = (struct atse_board_priv *)ndev->priv;
	memset(bd_priv, 0, sizeof(*bd_priv));

	spin_lock_init(&bd_priv->tx_lock);
	spin_lock_init(&bd_priv->rx_lock);
	bd_priv->phy_dev = phy_dev;

	/* fill in data for net_device stucture */
	ether_setup(ndev);
	mac_addr_0.i = readl(ATSE_MAC_REG_MAC_ADDR_0);
	mac_addr_1.i = readl(ATSE_MAC_REG_MAC_ADDR_1);

	/* fill in ethernet HW address */
	ndev->dev_addr[0] = mac_addr_0.c[0];
	ndev->dev_addr[1] = mac_addr_0.c[1];
	ndev->dev_addr[2] = mac_addr_0.c[2];
	ndev->dev_addr[3] = mac_addr_0.c[3];
	ndev->dev_addr[4] = mac_addr_1.c[0];
	ndev->dev_addr[5] = mac_addr_1.c[1];

	ndev->open            = atse_open;
	ndev->stop            = atse_close;
	ndev->hard_start_xmit = atse_hard_start_xmit;
	ndev->tx_timeout = atse_timeout;
	ndev->get_stats = atse_query_stats;
	ndev->set_multicast_list = atse_set_multicast_list;
	

	ret = register_netdev(ndev);
	if (ret != 0) {
		printk("ATSE driver:%s:%d:register_netdev() failed\n", __FILE__, __LINE__);
		goto OUT_ERR;
	} else {
		/*
		 * now, print out the card info, in a short format:
		 * 1. Device name
		 * 2. Driver version
		 * 3. PHY revision number
		 * 4. Ethernet hardware address
		 * 
		 *
		 */
		printk("%s: %s \n", ndev->name, atse_version_str);
		printk("%s: %s \n", ndev->name, atse_version_str1);
		printk("%s: Altera Tripple Speed, ether hw addr %s",
		       ndev->name, print_mac(mac_buf, ndev->dev_addr));
		printk(", %s", bd_priv->phy_dev->name);
		printk("\n");


	}

	return 0;

OUT_ERR:
	free_netdev(ndev);
	return ret;
}


static inline void atse_request_datacs(struct platform_device *pdev, struct net_device *ndev)
{
	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

	return;
}


static void __exit atse_exit(void)
{
	/* platform_driver_unregister(&atse_driver); */
	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();
	printk("++++JoeCamel:%s:%d:%s: FIXME\n", __FILE__, __LINE__, __FUNCTION__);

}

static void atse_release_board(struct platform_device *pdev, struct atse_board_priv *bd_priv)
{
	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();
	printk("++++JoeCamel:%s:%d:%s: FIXME\n", __FILE__, __LINE__, __FUNCTION__);

}


static int atse_drv_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct atse_board_priv *bd_priv = ndev->priv;

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

	platform_set_drvdata(pdev, NULL);

	unregister_netdev(ndev);
	atse_release_board(pdev, bd_priv);
	free_netdev(ndev);

	return 0;
}

static int
atse_drv_suspend(struct platform_device *dev, pm_message_t state)
{
	struct net_device *ndev = platform_get_drvdata(dev);

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();
	printk("++++JoeCamel:%s:%d:%s: FIXME\n", __FILE__, __LINE__, __FUNCTION__);

	if (ndev) {
		if (netif_running(ndev)) {
			netif_device_detach(ndev);
			atse_shutdown(ndev);
		}
	}
	return 0;
}

/*
 * Initilize ATSE Whole Board
 */
static void atse_init_atse(struct net_device *ndev)
{
	struct atse_board_priv *bd_priv = (struct atse_board_priv *) ndev->priv;

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();
	printk("++++JoeCamel:%s:%d:%s: FIXME\n", __FILE__, __LINE__, __FUNCTION__);

	/* Init PHY  */

	/* Init Descriptors */

	/* Init SGDMA_TX Controller */

	/* Init SGDMA_RX Controller */

	/* Init Driver variable */
	bd_priv->tx_pkt_cnt = 0;
	bd_priv->queue_pkt_len = 0;
	ndev->trans_start = 0;
}


static void atse_reset(struct atse_board_priv *bd_priv)
{
	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();
	printk("++++JoeCamel:%s:%d:%s: FIXME\n", __FILE__, __LINE__, __FUNCTION__);

	/* RESET device */
	//writeb(DM9000_NCR, db->io_addr);
	//udelay(200);
	//writeb(NCR_RST, db->io_data);
	//udelay(200);
}



static int atse_drv_resume(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct atse_board_priv *bd_priv = (struct atse_board_priv *) ndev->priv;

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();
	printk("++++JoeCamel:%s:%d:%s: FIXME\n", __FILE__, __LINE__, __FUNCTION__);

	if (ndev) {
		if (netif_running(ndev)) {
			atse_reset(bd_priv);
			atse_init_atse(ndev);
			netif_device_attach(ndev);
		}
	}
	return 0;
}



static int atse_drv_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct atse_board_priv *bd_priv;
	struct resource *res_desc_mem;
	struct resource *res_sgdma_rx_mem;
	struct resource *res_sgdma_tx_mem;
	struct resource *res_sgdma_rx_irq;
	struct resource *res_sgdma_tx_irq;
	int ret;

	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();

	/* 1. Get descriptor mem resurce */
	res_desc_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM, ATSE_RESOURCE_NAME_STR_DESC_MEM);
	if (!res_desc_mem) {
		printk("++++ATSE:%s:%d:platform_get_resource_byname() failed\n", __FILE__, __LINE__);
		ret = -ENODEV;
		goto out;
	}

	if (!request_mem_region(res_desc_mem->start, res_desc_mem->end - res_desc_mem->start + 1, ATSE_CARDNAME)) {
		printk("******ATSE:%s:%d:request_mem_region() failed\n", __FILE__, __LINE__);
		ret = -EBUSY;
		goto out;
	}
	
	/* 2. Get sgdma_rx mem resurce */
	res_sgdma_rx_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM, ATSE_RESOURCE_NAME_STR_SGDMA_RX_MEM);
	if (!res_sgdma_rx_mem) {
		printk("++++ATSE:%s:%d:platform_get_resource_byname() failed\n", __FILE__, __LINE__);
		ret = -ENODEV;
		goto out;
	}
	if (!request_mem_region(res_sgdma_rx_mem->start, res_sgdma_rx_mem->end - res_sgdma_rx_mem->start + 1, ATSE_CARDNAME)) {
		printk("******ATSE:%s:%d:request_mem_region() failed\n", __FILE__, __LINE__);
		ret = -EBUSY;
		goto out;
	}
	
	/* 3. Get sgdma_tx mem resurce */
	res_sgdma_tx_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM, ATSE_RESOURCE_NAME_STR_SGDMA_TX_MEM);
	if (!res_sgdma_tx_mem) {
		printk("++++ATSE:%s:%d:platform_get_resource_byname() failed\n", __FILE__, __LINE__);
		ret = -ENODEV;
		goto out;
	}
	if (!request_mem_region(res_sgdma_tx_mem->start, res_sgdma_tx_mem->end - res_sgdma_tx_mem->start + 1, ATSE_CARDNAME)) {
		printk("******ATSE:%s:%d:request_mem_region() failed\n", __FILE__, __LINE__);
		ret = -EBUSY;
		goto out;
	}
	
	/* 4. Get sgdma_tx irq resurce */
	res_sgdma_tx_irq = platform_get_resource_byname(pdev, IORESOURCE_IRQ, ATSE_RESOURCE_NAME_STR_SGDMA_TX_IRQ);
	if (!res_sgdma_tx_irq) {
		printk("++++ATSE:%s:%d:platform_get_resource_byname() failed\n", __FILE__, __LINE__);
		ret = -ENODEV;
		goto out;
	}


	/* 5. Get sgdma_rx irq resurce */
	res_sgdma_rx_irq = platform_get_resource_byname(pdev, IORESOURCE_IRQ, ATSE_RESOURCE_NAME_STR_SGDMA_RX_IRQ);
	if (!res_sgdma_rx_irq) {
		printk("++++ATSE:%s:%d:platform_get_resource_byname() failed\n", __FILE__, __LINE__);
		ret = -ENODEV;
		goto out;
	}





	ndev = alloc_etherdev( sizeof(struct atse_board_priv) );
	if (!ndev) {
		printk("%s:%d:%s could not allocate device\n", __FILE__, __LINE__, ATSE_CARDNAME);
		ret = -ENOMEM;
		goto out;
	}
	
	bd_priv = netdev_priv(ndev);
	bd_priv->res_desc_mem     =     res_desc_mem;    
	bd_priv->res_sgdma_rx_mem = res_sgdma_rx_mem;
	bd_priv->res_sgdma_tx_mem = res_sgdma_tx_mem;
	bd_priv->res_sgdma_rx_irq = res_sgdma_rx_irq;
	bd_priv->res_sgdma_tx_irq = res_sgdma_tx_irq;


	SET_NETDEV_DEV(ndev, &pdev->dev);

	platform_set_drvdata(pdev, ndev);	
	atse_probe(ndev);
	
	
	return 0;
	
out:
	return ret;

}


static struct platform_driver atse_driver = {
	.driver	= {
		/* the name must be the same as in struct platform_device */
		.name    = ATSE_CARDNAME,
		.owner	 = THIS_MODULE,
	},
	.probe   = atse_drv_probe,
	.remove  = atse_drv_remove,
	.suspend = atse_drv_suspend,
	.resume  = atse_drv_resume,
};


static int __init
atse_init(void)
{
	ATSE_DEBUG_PRINT_FUNC_TRACE_ENTER();
	/* probe board and register */
	return platform_driver_register(&atse_driver);
}

module_init(atse_init);
module_exit(atse_exit);
