/*******************************************************************************
*  linux/drivers/net/altera_tse.c
*
* Copyright (C) 2008 Altera Corporation.
*
* History:
*    o  SLS  - Linux 2.6.27                                                            
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
*******************************************************************************/  

#include <linux/module.h>	/* for module-version */
#include <linux/kernel.h>	/* printk(), and other useful stuff */
#include <linux/sched.h>	/* for jiffies, HZ, etc. */
#include <linux/string.h>	/* inline memset(), etc. */
#include <linux/errno.h>	/* return codes */
#include <linux/ioport.h>	/* request_region(), release_region() */
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/netdevice.h>	/* struct net_device, and other headers */
#include <linux/etherdevice.h>	/* eth_type_trans */
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/init.h>		/* __init (when not using as a module) */
#include <linux/mii.h>
#include <linux/phy.h>

#include <linux/pm.h>		/* pm_message_t */
#include <linux/platform_device.h>

#include <asm/cacheflush.h>

#include <asm/processor.h>	/* Processor type for cache alignment. */
#include <asm/bitops.h>
#include <asm/io.h>		/* I/O functions */
#include <asm/uaccess.h>	/* User space memory access functions */

#include "altera_tse.h"

static const char version[] =
    "Altera Triple Speed MAC IP Driver(v8.0) "
    "developed by SLS,August-2008\n";

/* DEBUG flags */
//#define DEBUG_INFO     1  
//#define DEBUG_WARNING  1
//#define DEBUG_ERROR    1
//
//#if DEBUG_INFO == 1
//#define PRINTK1(args...) printk(args)
//#else                                
//#define PRINTK1(args...)
//#endif                                                     
//
//#if DEBUG_WARNING == 1
//#define PRINTK2(args...) printk(args)
//#else                                                          
//#define PRINTK2(args...)
//#endif                                                                  
//
//#if DEBUG_ERROR == 1
//#define PRINTK3(args...) printk(args)
//#else
//#define PRINTK3(args...)
//#endif

//netif_msg_rx_err
//netif_msg_rx_status
//netif_msg_intr
//netif_msg_tx_queued
//netif_msg_link
//netif_msg_drv
//netif_msg_hw
//netif_msg_ifdown

/* 1 -> print contents of all tx packtes on printk
 */
#define TX_DEEP_DEBUG 0

#if 1
#define my_flush_dcache_range(x,y) do{flush_dcache_range(x,y);}while(0)
#else
#define my_flush_dcache_range(x,y) flush_cache_all()
#endif

/*
 * MDIO specific functions
 */

static int altera_tse_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	alt_tse_mac *mac_dev;
	unsigned int *mdio_regs;
	unsigned int data;

	mac_dev = (alt_tse_mac *) bus->priv;

	/* set MDIO address */
	writel(mii_id, &mac_dev->mdio_phy0_addr);
	mdio_regs = (unsigned int *) &mac_dev->mdio_phy0;

	/* get the data */
	data = readl(&mdio_regs[regnum]);

	return data & 0xffff;
}

static int altera_tse_mdio_write(struct mii_bus *bus, int mii_id, int regnum, u16 value)
{
	alt_tse_mac *mac_dev;
	unsigned int *mdio_regs;
	unsigned int data;

	mac_dev = (alt_tse_mac *) bus->priv;

	/* set MDIO address */
	writel(mii_id, &mac_dev->mdio_phy0_addr);
	mdio_regs = (unsigned int *) &mac_dev->mdio_phy0;

	/* get the data */
	data = (unsigned int) value;

	writel(data, &mdio_regs[regnum]);

	return 0;
}

static int altera_tse_mdio_irqs[] = {
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
};

int altera_tse_mdio_register(struct alt_tse_private *tse_priv)
{
	struct mii_bus *mdio_bus;
	int ret;
	int i;

	mdio_bus = mdiobus_alloc();
	if (!mdio_bus)
		return -ENOMEM;

	mdio_bus->name = "Altera TSE MII Bus";
	mdio_bus->read = &altera_tse_mdio_read;
	mdio_bus->write = &altera_tse_mdio_write;
	snprintf(mdio_bus->id, MII_BUS_ID_SIZE, "%u", tse_priv->tse_config->mii_id);

	mdio_bus->irq = altera_tse_mdio_irqs;
	mdio_bus->priv = (void *) tse_priv->mac_dev;

	ret = mdiobus_register(mdio_bus);
	if (ret) {
		printk(KERN_ERR "%s: Cannot register as MDIO bus\n",
				mdio_bus->name);
		mdiobus_free(mdio_bus);
		return ret;
	}

	/* report available PHYs */
	for (i = 31; i >= 0; i--) {
		u32 phy_id;
		u32 phy_id_bottom;
		u32 phy_id_top;
		int r;

		r = get_phy_id(mdio_bus, i, &phy_id);
		phy_id_top = (phy_id >> 16) & 0xffff;
		phy_id_bottom = (phy_id) & 0xffff;
		if (r)
			return r;

		if (phy_id_top != phy_id_bottom)
			printk(KERN_INFO "Found PHY with ID=0x%x at address=0x%x\n",
				phy_id, i);
	}

	return ret;
}

/*******************************************************************************
*	SGDMA Control Stuff
*
*******************************************************************************/

/* Clear descriptor memory ,initialize SGDMA descriptor chain and reset SGDMA.
* arg1     :TSE private data structure
* @return  :1 on success
*           less than zero on error
*/
static void sgdma_config(struct alt_tse_private *tse_priv)
{
	unsigned int mem_off, *mem_ptr = (unsigned int *)tse_priv->desc_mem_base,
	    mem_size = ALT_TSE_TOTAL_SGDMA_DESC_SIZE;

	//Clearing SGDMA desc Memory
	for (mem_off = 0; mem_off < mem_size; mem_off += 4)
		*mem_ptr++ = 0x00000000;

	//reset rx_sgdma
	tse_priv->rx_sgdma_dev->control = ALT_SGDMA_CONTROL_SOFTWARERESET_MSK;
	tse_priv->rx_sgdma_dev->control = 0x0;

	//reset tx_sgdma
	tse_priv->tx_sgdma_dev->control = ALT_SGDMA_CONTROL_SOFTWARERESET_MSK;
	tse_priv->tx_sgdma_dev->control = 0x0;                       
}

/* This is a generic routine that the SGDMA mode-specific routines
* call to populate a descriptor.
* arg1     :pointer to first SGDMA descriptor.
* arg2     :pointer to next  SGDMA descriptor.
* arg3     :Address to where data to be written.
* arg4     :Address from where data to be read.
* arg5     :no of byte to transaction.
* arg6     :variable indicating to generate start of packet or not
* arg7     :read fixed
* arg8     :write fixed
* arg9     :read burst
* arg10    :write burst
* arg11    :atlantic_channel number
*/
static void alt_sgdma_construct_descriptor_burst(volatile struct
						 alt_sgdma_descriptor *desc,
						 volatile struct
						 alt_sgdma_descriptor *next,
						 unsigned int *read_addr,
						 unsigned int *write_addr,
						 unsigned short length_or_eop,
						 int generate_eop,
						 int read_fixed,
						 int write_fixed_or_sop,
						 int read_burst,
						 int write_burst,
						 unsigned char atlantic_channel)
{
	/*
	* Mark the "next" descriptor as "not" owned by hardware. This prevents
	* The SGDMA controller from continuing to process the chain. This is
	* done as a single IO write to bypass cache, without flushing
	* the entire descriptor, since only the 8-bit descriptor status must
	* be flushed.
	*/
	next->descriptor_control = (next->descriptor_control
				    &
				    ~ALT_SGDMA_DESCRIPTOR_CONTROL_OWNED_BY_HW_MSK);
	//	printk("read_addr %p\n", read_addr);
#if TX_DEEP_DEBUG
	if (read_addr != 0) {
	  unsigned char *buf = ((unsigned char *)read_addr) + 2;
	  int i;
	  int len  = length_or_eop;

	  /* first 2 bytes of the mac-address of my laptop...
	   */
	  if (buf[0] == 0 && buf[1] == 0x12) {

	    printk("incoming tx descriptor read_addr:%p len:%d", read_addr, len);
	    BUG_ON(write_addr != 0);	
	    for (i = 0; i < len-2; i++) {
	      if ((i % 16) == 0) {
		printk("\n%04x: ", i);
	      }
	      if ((i % 16) == 8) { // emulate wireshark output
		printk(" ");
	      }
	      printk("%02x ", buf[i]); //assume tx_shift
	    }
	    printk("\n -- end of packet data\n");
	  }
	}
#endif
	desc->source = read_addr;
	desc->destination = write_addr;
	desc->next = (unsigned int *)next;
	desc->source_pad = 0x0;
	desc->destination_pad = 0x0;
	desc->next_pad = 0x0;
	desc->bytes_to_transfer = length_or_eop;
	desc->actual_bytes_transferred = 0;
	desc->descriptor_status = 0x0;

	/* SGDMA burst not currently supported */
	desc->read_burst = 0;	//read_burst;  //TBD
	desc->write_burst = 0;	//write_burst; //TBD

	/*
	* Set the descriptor control block as follows:
	* - Set "owned by hardware" bit
	* - Optionally set "generate EOP" bit
	* - Optionally set the "read from fixed address" bit
	* - Optionally set the "write to fixed address bit (which serves
	*   serves as a "generate SOP" control bit in memory-to-stream mode).
	* - Set the 4-bit atlantic channel, if specified
	*
	* Note that this step is performed after all other descriptor information
	* has been filled out so that, if the controller already happens to be
	* pointing at this descriptor, it will not run (via the "owned by hardware"
	* bit) until all other descriptor information has been set up.
	*/

	desc->descriptor_control = ( 
		(ALT_SGDMA_DESCRIPTOR_CONTROL_OWNED_BY_HW_MSK) | 
		(generate_eop ? ALT_SGDMA_DESCRIPTOR_CONTROL_GENERATE_EOP_MSK : 0x0) | 
		(read_fixed ? ALT_SGDMA_DESCRIPTOR_CONTROL_READ_FIXED_ADDRESS_MSK : 0x0) | 
		(write_fixed_or_sop ? ALT_SGDMA_DESCRIPTOR_CONTROL_WRITE_FIXED_ADDRESS_MSK : 0x0) | 
		(atlantic_channel ? ((atlantic_channel & 0x0F) << 3) : 0)
	    );
}

/* Start to copy from rxFIFO into given buffer memory area with Asynchronous .so the
* function does not return the actual bytes transferred for current descriptor
* arg1     :TSE private data structure
* arg2     :Pointer to first descriptor structure of RX SGDMA chain
* return   SUCCESS on success
*          less than 0 on errors
*/
static int sgdma_async_read(struct alt_tse_private *tse_priv,
			    volatile struct alt_sgdma_descriptor *rx_desc)
{
	unsigned int timeout;
	unsigned int retval = 0;
	struct net_device *dev = tse_priv->dev;

	/* Make sure SGDMA controller is not busy from a former command */
	timeout = 0;

	if (netif_msg_rx_status(tse_priv))
		printk(KERN_WARNING "%s :Waiting while rx SGDMA is busy........\n",
			dev->name);

	tse_priv->rx_sgdma_dev->control = 0;
	tse_priv->rx_sgdma_dev->status = 0x1f;	//clear status

	/* Wait for the descriptor (chain) to complete */
	while (tse_priv->rx_sgdma_dev->status & ALT_SGDMA_STATUS_BUSY_MSK) {
		ndelay(100);
		if (timeout++ == ALT_TSE_SGDMA_BUSY_WATCHDOG_CNTR) {
			if (netif_msg_rx_status(tse_priv))
				printk(KERN_WARNING "%s :RX SGDMA Timeout\n", dev->name);
			return -EBUSY;
		}
	}

	tse_priv->rx_sgdma_dev->next_descriptor_pointer = 
           (int)(volatile struct alt_sgdma_descriptor *)rx_desc;
	
	//Don't just enable IRQs adhoc
	tse_priv->rx_sgdma_dev->control = tse_priv->rx_sgdma_imask | ALT_SGDMA_CONTROL_RUN_MSK;

	return retval;
}

static int sgdma_async_write(struct alt_tse_private *tse_priv,
			    volatile struct alt_sgdma_descriptor *tx_desc)
{
	unsigned int timeout;
	unsigned int retval = 0;
	struct net_device *dev = tse_priv->dev;

	/* Make sure SGDMA controller is not busy from a former command */
	timeout = 0;
//	if (netif_msg_tx_done(tse_priv))
//		printk(KERN_WARNING "%s :Waiting while tx SGDMA is busy.........\n",
//			dev->name);

	tse_priv->tx_sgdma_dev->control = 0;
	tse_priv->tx_sgdma_dev->status = 0x1f;	//clear status

	/* Wait for the descriptor (chain) to complete */
	while (tse_priv->tx_sgdma_dev->status & ALT_SGDMA_STATUS_BUSY_MSK) {
		ndelay(100);
		if (timeout++ == ALT_TSE_SGDMA_BUSY_WATCHDOG_CNTR) {
			if (netif_msg_rx_status(tse_priv))
				printk(KERN_WARNING "%s :TX SGDMA Timeout\n", dev->name);
			return -EBUSY;
		}
	}

	tse_priv->tx_sgdma_dev->next_descriptor_pointer = 
           (int)(volatile struct alt_sgdma_descriptor *)tx_desc;
	
	//Don't just enable IRQs adhoc
	tse_priv->tx_sgdma_dev->control = tse_priv->tx_sgdma_imask | ALT_SGDMA_CONTROL_RUN_MSK;

	return retval;
}

/* Create TSE descriptor for next buffer
* or error if no buffer available
*/
static int tse_sgdma_add_buffer(struct net_device *dev)
{

	struct alt_tse_private *tse_priv = netdev_priv(dev);
	int next_head;
	struct sk_buff *skb;
	
	next_head = ((tse_priv->rx_sgdma_descriptor_head +1) & (ALT_RX_RING_MOD_MASK));
	    
	if (next_head == tse_priv->rx_sgdma_descriptor_tail)
		return -EBUSY;

	//current MTU + 4 b/c input packet is aligned by 2;
	skb = dev_alloc_skb(tse_priv->current_mtu + 4);
	if (skb == NULL) {
		if (netif_msg_rx_err(tse_priv))
			printk(KERN_WARNING "%s :ENOMEM:::skb_size=%d\n", 
				dev->name, tse_priv->current_mtu + 4);
		return -ENOMEM;
	}
	flush_dcache_range((unsigned long)skb->data,
			   ((unsigned long)skb->data) + skb->len);
	skb->dev = dev;
	
	tse_priv->rx_skb[tse_priv->rx_sgdma_descriptor_head] = skb;

	alt_sgdma_construct_descriptor_burst(
		(volatile struct alt_sgdma_descriptor *)&tse_priv->sgdma_rx_desc[tse_priv->rx_sgdma_descriptor_head],
		(volatile struct alt_sgdma_descriptor *)&tse_priv->sgdma_rx_desc[next_head],
		NULL, //read addr
		(unsigned int *)tse_priv->rx_skb[tse_priv->rx_sgdma_descriptor_head]->data,
		0x0, //length or EOP
		0x0, //gen eop
		0x0, //read fixed
		0x0, //write fixed or sop
		0x0, //read burst
		0x0, //write burst
		0x0 //channel
	);

	tse_priv->rx_sgdma_descriptor_head = next_head;

	return SUCCESS;

}

/* Init and setup SGDMA Descriptor chain.
* arg1     :TSE private data structure
* return    SUCCESS
*/
static unsigned int sgdma_read_init(struct net_device *dev)                       
{
	struct alt_tse_private *tse_priv = netdev_priv(dev);
	int rx_loop;
	
	for (rx_loop = 0; rx_loop < ALT_TSE_RX_SGDMA_DESC_COUNT; rx_loop++) {
		//Should do some checking here.
		tse_sgdma_add_buffer(dev);
	}
	sgdma_async_read(tse_priv,
			 &tse_priv->sgdma_rx_desc[tse_priv->
						  rx_sgdma_descriptor_tail]);
	if (netif_msg_rx_status(tse_priv))
		printk(KERN_WARNING "%s :Read init is completed\n", dev->name);
	
	return SUCCESS;
}


/*******************************************************************************
* actual ethernet stuff
*
*******************************************************************************/
/* NAPI Polling function
*	processes packets received, until end of received packets
*	or budget is reached
*	Clear TX buffers
*	also restarts SGDMAs for TX, RX as needed
*/
static int tse_poll(struct napi_struct *napi, int budget)
{
	struct alt_tse_private *tse_priv = container_of(napi, struct alt_tse_private, napi);
	struct net_device *dev = tse_priv->dev;
	volatile struct alt_sgdma_descriptor *temp_desc_pointer;
	unsigned int desc_status, desc_control;
	int howmany = 0;
	unsigned int rx_bytes, netif_rx_status;
	unsigned char *skb_Rxbuffer;
	struct sk_buff *skb;
	unsigned long flags;
	unsigned int tx_tail;
	unsigned int tx_loop;
	int done;

	if (netif_msg_intr(tse_priv))
		printk(KERN_WARNING "%s :Entering tse_poll with budget = 0x%x\n",
			dev->name, budget);
	
	temp_desc_pointer =
	    &tse_priv->sgdma_rx_desc[tse_priv->
				     rx_sgdma_descriptor_tail];
	desc_status = temp_desc_pointer->descriptor_status;

	//loop over descriptors until one is not complete
	while ((desc_status &
	       ALT_SGDMA_DESCRIPTOR_STATUS_TERMINATED_BY_EOP_MSK) && (howmany < budget)) 
	{
		if (netif_msg_intr(tse_priv))
			printk(KERN_WARNING "%s NAPI RX Loop\n", dev->name);
		
		if ((desc_status & ALT_SGDMA_DESCRIPTOR_STATUS_ERROR_MSK) &&
			(netif_msg_rx_err(tse_priv)))
				printk(KERN_WARNING "%s :TSE RX Err: Status = 0x%x\n",
					dev->name, desc_status);
		
		//get desc SKB
		skb =
		    tse_priv->rx_skb[tse_priv->
				     rx_sgdma_descriptor_tail];
		tse_priv->rx_skb[tse_priv->rx_sgdma_descriptor_tail] =
		    NULL;

		rx_bytes = temp_desc_pointer->actual_bytes_transferred;
		rx_bytes -= NET_IP_ALIGN;

		//process packet
		/* Align IP header to 32 bits */
		skb_reserve(skb, NET_IP_ALIGN);
		skb_Rxbuffer = skb_put(skb, rx_bytes);
		skb->protocol = eth_type_trans(skb, dev);

		netif_rx_status = netif_receive_skb(skb);
		
		if (netif_rx_status == NET_RX_DROP)
		{
			if(netif_msg_rx_err(tse_priv))
				printk(KERN_WARNING "%s :NET_RX_DROP occurred\n",
					dev->name);
			
	//		tse_priv->mac_dev->command_config.image |= ALTERA_TSE_CMD_XOFF_GEN_MSK;	
			tse_priv->status.rx_dropped++;
		}                      

		//next descriptor
		tse_priv->rx_sgdma_descriptor_tail = 
			((tse_priv->rx_sgdma_descriptor_tail + 1) & (ALT_RX_RING_MOD_MASK));                      
		   
		//add new desc	
		if ((tse_sgdma_add_buffer(dev)) && (netif_msg_rx_err(tse_priv)))
			printk(KERN_WARNING "%s :ah, something happened, and no desc was added to rx",
				dev->name);

		//update temp_desc to next desc
		temp_desc_pointer =
		    &tse_priv->sgdma_rx_desc[tse_priv->
					     rx_sgdma_descriptor_tail];
		desc_status = temp_desc_pointer->descriptor_status;
		
		howmany++;
	}
	
	if (netif_msg_rx_status(tse_priv))
		printk(KERN_INFO "%s : RX SGDMA STATUS=0x%x, tail=0x%x, head=0x%x\n",
			dev->name, tse_priv->rx_sgdma_dev->status, 
			tse_priv->rx_sgdma_descriptor_tail,
			tse_priv->rx_sgdma_descriptor_head);
	
	//check sgdma status, and restart as needed
	if ((tse_priv->rx_sgdma_dev->status & ALT_SGDMA_STATUS_CHAIN_COMPLETED_MSK) || 
		!(tse_priv->rx_sgdma_dev->status & ALT_SGDMA_STATUS_BUSY_MSK)) 
	{
		if (netif_msg_rx_status(tse_priv))
			printk(KERN_INFO "%s :starting with rx_tail = %d and rx_head = %d\n",
				dev->name,
				tse_priv->rx_sgdma_descriptor_tail, 
				tse_priv->rx_sgdma_descriptor_head);
//		spin_lock(tse_priv->rx_lock);
		sgdma_async_read(tse_priv,
			&tse_priv->sgdma_rx_desc[tse_priv->rx_sgdma_descriptor_tail]);
//		spin_unlock(tse_priv->rx_lock);
	}
	

	//now do TX stuff
	if (tse_priv->tx_sgdma_descriptor_tail != tse_priv->tx_sgdma_descriptor_head) {
		if (spin_trylock_irqsave(&tse_priv->tx_lock, flags))
		{       
			if (netif_msg_intr(tse_priv))
				printk(KERN_WARNING "%s :NAPI TX Section\n", dev->name);     
		
			tx_tail = tse_priv->tx_sgdma_descriptor_tail; 
			temp_desc_pointer = &tse_priv->sgdma_tx_desc[tx_tail];
			desc_control = temp_desc_pointer->descriptor_control;	
			
			//loop over tx desc from tail till head, check for !hw owned
			//for (tx_loop = 0; tx_loop < ALT_TSE_TX_SGDMA_DESC_COUNT; tx_loop++)
			tx_loop = 0;
			while(!(desc_control & ALT_SGDMA_DESCRIPTOR_CONTROL_OWNED_BY_HW_MSK) &&
				(tx_tail != tse_priv->tx_sgdma_descriptor_head))
			{
				dev_kfree_skb(tse_priv->tx_skb[tx_tail]);
				tse_priv->tx_skb[tx_tail] = NULL;
			
				tx_loop++;
				tx_tail = ((tx_tail + 1) & (ALT_TX_RING_MOD_MASK));         
				temp_desc_pointer = &tse_priv->sgdma_tx_desc[tx_tail];
				desc_control = temp_desc_pointer->descriptor_control;
				
				//if((desc_control & ALT_SGDMA_DESCRIPTOR_CONTROL_OWNED_BY_HW_MSK))
				//	break;
			}
			tse_priv->tx_sgdma_descriptor_tail = tx_tail;
			temp_desc_pointer = &tse_priv->sgdma_tx_desc[tx_tail];                                    

			//check is tx sgdma is running, and if it should be
			if (!(tse_priv->tx_sgdma_dev->status & ALT_SGDMA_STATUS_BUSY_MSK) & 
				(temp_desc_pointer->descriptor_control & ALT_SGDMA_DESCRIPTOR_CONTROL_OWNED_BY_HW_MSK)) 
			{
				if (netif_msg_intr(tse_priv))
					printk(KERN_WARNING "%s :NAPI Starting TX SGDMA with Desc %d\n", 
						dev->name, tx_tail);
				//restart sgdma
				sgdma_async_write(tse_priv, &tse_priv->sgdma_tx_desc[tx_tail]);
				
			}
			
			//restart queue if it was stopped
			if (netif_queue_stopped(dev))
			{
				if (netif_msg_intr(tse_priv))
					printk(KERN_WARNING "%s :Cleared %d descriptors,tail = %d, head = %d, Waking QUEUE\n", 
						dev->name, tx_loop, tx_tail, tse_priv->tx_sgdma_descriptor_head);
				netif_wake_queue(dev);
			}
	
			spin_unlock_irqrestore(&tse_priv->tx_lock, flags);
		}
	}
	                                                        
	done = 0;
	/* if all packets processed, complete rx, and turn on normal IRQs */
	if (howmany < budget) {
		if (netif_msg_intr(tse_priv))
			printk(KERN_WARNING "%s :NAPI Complete, did %d packets with budget = %d\n", 
				dev->name, howmany, budget);
		napi_complete(napi);

		/* turn on desc irqs again */
		tse_priv->rx_sgdma_imask |= ALT_SGDMA_CONTROL_IE_GLOBAL_MSK;
		tse_priv->rx_sgdma_dev->control |= ALT_SGDMA_CONTROL_IE_GLOBAL_MSK;
#ifndef NO_TX_IRQ
		tse_priv->tx_sgdma_imask |= ALT_SGDMA_CONTROL_IE_GLOBAL_MSK;
		tse_priv->tx_sgdma_dev->control |= ALT_SGDMA_CONTROL_IE_GLOBAL_MSK;
#endif		
		done = 1;
	}

        budget -= howmany;
	return done?0:1;
}



/* SG-DMA TX & RX FIFO interrupt routing
* arg1     :irq number
* arg2     :user data passed to isr
* arg3     :pt_resgs structure passed from kernel
*/
static irqreturn_t alt_sgdma_isr(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = dev_id;
	struct alt_tse_private *tse_priv = netdev_priv(dev);
	
	if (netif_msg_intr(tse_priv))
		printk(KERN_WARNING "%s :TSE IRQ TX head = %d, tail = %d\n", 
			dev->name, tse_priv->tx_sgdma_descriptor_head, tse_priv->tx_sgdma_descriptor_tail);
	//turn off desc irqs and enable napi rx 
	if (napi_schedule_prep(&tse_priv->napi)) {
		if (netif_msg_intr(tse_priv))
			printk(KERN_WARNING "%s :NAPI Starting\n", dev->name);
		tse_priv->rx_sgdma_imask &= ~ALT_SGDMA_CONTROL_IE_GLOBAL_MSK;
		tse_priv->rx_sgdma_dev->control &= ~ALT_SGDMA_CONTROL_IE_GLOBAL_MSK;
#ifndef NO_TX_IRQ
		tse_priv->tx_sgdma_imask &= ~ALT_SGDMA_CONTROL_IE_GLOBAL_MSK;
		tse_priv->tx_sgdma_dev->control &= ~ALT_SGDMA_CONTROL_IE_GLOBAL_MSK;
#endif
		__napi_schedule(&tse_priv->napi);
	} else {
	//if we get here, we received another irq while processing NAPI
		if (netif_msg_intr(tse_priv))
			printk(KERN_WARNING "%s :TSE IRQ Received while IRQs disabled\n",
				dev->name);
	}

	//reset IRQ 	
	tse_priv->rx_sgdma_dev->control |= ALT_SGDMA_CONTROL_CLEAR_INTERRUPT_MSK;
	tse_priv->tx_sgdma_dev->control |= ALT_SGDMA_CONTROL_CLEAR_INTERRUPT_MSK;
	
	return IRQ_HANDLED;
}      



#ifdef CONFIG_NET_POLL_CONTROLLER
/*
* Polling receive - used by netconsole and other diagnostic tools
* to allow network i/o with interrupts disabled.
*/
static void tse_net_poll_controller(struct net_device *dev)
{

	struct alt_tse_private *tse_priv = netdev_priv(dev);
	disable_irq(tse_priv->rx_fifo_interrupt);
	disable_irq(tse_priv->tx_fifo_interrupt);
	alt_sgdma_isr(tse_priv->rx_fifo_interrupt, dev, NULL);
	enable_irq(tse_priv->rx_fifo_interrupt);
	enable_irq(tse_priv->tx_fifo_interrupt);

}
#endif



/*******************************************************************************
* TX and RX functions
*	Send Function
*	Receive function, clears RX Ring - Called from NAPI softirq
*	Clear Transmit buffers - Called from NAPI softirq
*
*******************************************************************************/


/* Send Packet Function
* arg1     :skb to send
* arg2     :netdev device
*/
static int tse_hardware_send_pkt(struct sk_buff *skb, struct net_device *dev)
{
	struct alt_tse_private *tse_priv = netdev_priv(dev);             
	unsigned int len;
	unsigned int next_head;
	unsigned int next_head_check;
	unsigned int head;
	unsigned int tail;
	unsigned int aligned_tx_buffer;
	unsigned long flags;
	char	req_tx_shift_16;
//	struct sk_buff *new_skb;
	

	aligned_tx_buffer = (unsigned int)skb->data;
	len = skb->len;
//	saved_len = skb->len;
//	offset = aligned_tx_buffer & 0x3;
	if ((unsigned int)skb->data & 0x2) {
		req_tx_shift_16 = 0x1;
		aligned_tx_buffer -= NET_IP_ALIGN;
		len += NET_IP_ALIGN;
	} else {
		req_tx_shift_16 = 0x0;
	}

	 /* Align len on 4, otherwise it seems we get truncated frames */

//       if (len & 3) {
//	 printk(KERN_WARNING "TSE align to word, skb->data = 0x%x, start address = 0x%x, len=%d, saved_len=%d, offset = %d\n", (unsigned int) skb->data, aligned_tx_buffer, len, saved_len, offset); 
         len += 3;
         len &= ~3UL;
//	 printk(KERN_WARNING "TSE new length = %d\n", len);
//       }

//	tse_priv->mac_dev->tx_cmd_stat.bits.tx_shift16 = 1;
//	aligned_tx_buffer = (unsigned int)skb->data;
//	len = skb->len;
//	if (aligned_tx_buffer & 0x2) {
//		aligned_tx_buffer -= NET_IP_ALIGN;
//		len += NET_IP_ALIGN;
//	} else {
//		new_skb = alloc_skb(skb->len + NET_IP_ALIGN, GFP_KERNEL);
//		skb_reserve(new_skb, NET_IP_ALIGN);
//		memcpy(new_skb->data, skb->data,  skb->len);
//		aligned_tx_buffer = (unsigned int)new_skb->data - NET_IP_ALIGN;
//		len = skb->len + NET_IP_ALIGN;
//		dev_kfree_skb(skb);
//		skb = new_skb;
//	}       

	/* len in align later in alt_sgdma_construct_descriptor_burst(), but 
	 * we can safely ingorned the extra alignement added in  on len here 
	 * since it's not actually part of the data and/or checksum
	 */	
	//Flush raw data from data cache	
	flush_dcache_range(aligned_tx_buffer, aligned_tx_buffer + len);
	
	spin_lock_irqsave(&tse_priv->tx_lock, flags);
	//get the heads
	head = tse_priv->tx_sgdma_descriptor_head;
	tail = tse_priv->tx_sgdma_descriptor_tail;
	next_head = (head + 1) & (ALT_TX_RING_MOD_MASK);
	next_head_check = (head + 2) & (ALT_TX_RING_MOD_MASK);

	if (netif_msg_tx_queued(tse_priv))
		printk(KERN_WARNING "%s :head = %d, next_head = %d, tail = %d\n", 
			dev->name, head, next_head, tse_priv->tx_sgdma_descriptor_tail);
		
	//if next next head is == tail, stop the queue                                 
	//next_head = (next_head + 1) & (ALT_TX_RING_MOD_MASK);
	if (next_head_check == tse_priv->tx_sgdma_descriptor_tail) {
		//no space in ring, we stop the queue
		if (netif_msg_tx_queued(tse_priv))
			printk(KERN_WARNING "%s :TX next_head not clear, stopping queue, tail = %d, head = %d\n",
				dev->name, tse_priv->tx_sgdma_descriptor_tail, tse_priv->tx_sgdma_descriptor_head);
		if (!netif_queue_stopped(dev))
			netif_stop_queue(dev);
		napi_schedule(&tse_priv->napi);
	}

	tse_priv->tx_skb[head] = skb;

	//wait till tx is done, change shift 16
	if(req_tx_shift_16 != tse_priv->last_tx_shift_16)
	{                                
		if (netif_msg_tx_queued(tse_priv))
			printk(KERN_WARNING "%s :tx_shift does not match\n", dev->name);
		
		while(tse_priv->tx_sgdma_dev->status & ALT_SGDMA_STATUS_BUSY_MSK)
		{}
		tse_priv->mac_dev->tx_cmd_stat.bits.tx_shift16 = req_tx_shift_16 & 0x1;
		tse_priv->last_tx_shift_16 = req_tx_shift_16;
	}               
	
	alt_sgdma_construct_descriptor_burst(
		(volatile struct alt_sgdma_descriptor *)&tse_priv->sgdma_tx_desc[head],
		(volatile struct alt_sgdma_descriptor *)&tse_priv->sgdma_tx_desc[next_head],
		(unsigned int *)aligned_tx_buffer, //read addr
		(unsigned int *)0,
		(len), //length or EOP
		0x1, //gen eop
		0x0, //read fixed
		0x1, //write fixed or sop                                            
		0x0, //read burst
		0x0, //write burst
		0x0 //channel
	);	


                                                           
	
	//now check is the sgdma is running, if it is then do nothing.
	//if it is not, start it up with irq's enabled.

	if (!(tse_priv->tx_sgdma_dev->status & ALT_SGDMA_STATUS_BUSY_MSK)) {
		if (netif_msg_tx_queued(tse_priv))
			printk(KERN_WARNING "%s :TX SGDMA Not Running\n", dev->name);
		sgdma_async_write(tse_priv, &tse_priv->sgdma_tx_desc[tail]);
	}                                                        
	
	tse_priv->tx_sgdma_descriptor_head = next_head;
	
	spin_unlock_irqrestore(&tse_priv->tx_lock,flags);

	
	tse_priv->dev->trans_start = jiffies;	
	
	return SUCCESS;
}            

/* Called every time the controller might need to be made
 * aware of new link state.  The PHY code conveys this                                  
 * information through variables in the phydev structure, and this
 * function converts those variables into the appropriate
 * register values, and can bring down the device if needed.               
 */                         
static void adjust_link(struct net_device *dev)
{
	struct alt_tse_private *tse_priv = netdev_priv(dev); 
	unsigned long flags;                                              
	struct phy_device *phydev = tse_priv->phydev;
	int new_state = 0;
	unsigned int refvar;
	
	//only change config if there is a link
	spin_lock_irqsave(&tse_priv->tx_lock, flags);
	if (phydev->link) 
	{              
		//read old config
		refvar = tse_priv->mac_dev->command_config.image;
		
		//check duplex
		if (phydev->duplex != tse_priv->oldduplex) 
		{
			new_state = 1;
			//not duplex
			if (!(phydev->duplex))
				refvar |= ALTERA_TSE_CMD_HD_ENA_MSK;
			else
				refvar &= ~ALTERA_TSE_CMD_HD_ENA_MSK;
			
			if (netif_msg_link(tse_priv))
				printk(KERN_WARNING "%s :Link duplex = 0x%x\n", dev->name, 
					phydev->duplex);
			
			tse_priv->oldduplex = phydev->duplex;
		}
		
		if (phydev->speed != tse_priv->oldspeed) 
		{
			new_state = 1;
			switch (phydev->speed) {
			case 1000:
				refvar |= ALTERA_TSE_CMD_ETH_SPEED_MSK;
				refvar &= ~ALTERA_TSE_CMD_ENA_10_MSK;
				break;				
			case 100:
				refvar &= ~ALTERA_TSE_CMD_ETH_SPEED_MSK;
				refvar &= ~ALTERA_TSE_CMD_ENA_10_MSK;
				break;				
			case 10:
				refvar &= ~ALTERA_TSE_CMD_ETH_SPEED_MSK;
				refvar |= ALTERA_TSE_CMD_ENA_10_MSK;
				break;				
			default:
				if (netif_msg_link(tse_priv))
					printk(KERN_WARNING
						"%s: Ack!  Speed (%d) is not 10/100/1000!\n",
						dev->name, phydev->speed);
				break;
			}
			
			tse_priv->oldspeed = phydev->speed;
		}
		
		tse_priv->mac_dev->command_config.image = refvar;         
		
		netif_carrier_on(tse_priv->dev);
		
	} else if (tse_priv->oldlink) {
		new_state = 1;
		tse_priv->oldlink = 0;
		tse_priv->oldspeed = 0;
		tse_priv->oldduplex = -1;                   
		netif_carrier_off(tse_priv->dev);
	} 
	
	if (new_state && netif_msg_link(tse_priv))
		phy_print_status(phydev);

	spin_unlock_irqrestore(&tse_priv->tx_lock, flags);

}

/*******************************************************************************
* Phy init
*	Using shared PHY control interface
*
*******************************************************************************/
/* Initializes driver's PHY state, and attaches to the PHY.
 * Returns 0 on success.
 */
static int init_phy(struct net_device *dev)
{
	struct alt_tse_private *tse_priv = netdev_priv(dev);
	struct alt_tse_config * tse_config = tse_priv->tse_config;
	struct phy_device *phydev;
	char phy_id[MII_BUS_ID_SIZE];
	char mii_id[MII_BUS_ID_SIZE];
	phy_interface_t interface;

	/* hard code for now */
	interface = tse_config->interface;

	tse_priv->oldlink = 0;
	tse_priv->oldspeed = 0;
	tse_priv->oldduplex = -1;

	snprintf(mii_id, MII_BUS_ID_SIZE, "%x", tse_config->mii_id);
	snprintf(phy_id, MII_BUS_ID_SIZE, PHY_ID_FMT, mii_id, tse_config->phy_addr);

	phydev = phy_connect(dev, phy_id, &adjust_link, 0, interface);

	if (IS_ERR(phydev)) {
		printk(KERN_ERR "%s: Could not attach to PHY\n", dev->name);
		return PTR_ERR(phydev);
	}

	phydev->supported &= tse_config->tse_supported_modes;
	phydev->advertising = phydev->supported;

	if (tse_config->autoneg == AUTONEG_DISABLE) {
		phydev->autoneg = tse_config->autoneg;
		phydev->speed = tse_config->speed;
		phydev->duplex = tse_config->duplex;
	}

	tse_priv->phydev = phydev;

	return 0;
}

/*******************************************************************************
* MAC setup and control
*	MAC init, and various setting functions
*
*******************************************************************************/ 
/* Initialize MAC core registers
*  arg1   : 'net_device' structure pointer 
*
*/
static int init_mac(struct net_device *dev)
{
	struct alt_tse_private *tse_priv = netdev_priv(dev);
        int counter;     
	int dat;
	
	/* reset the mac */
	tse_priv->mac_dev->command_config.bits.transmit_enable=0;
	tse_priv->mac_dev->command_config.bits.receive_enable=0;      
	tse_priv->mac_dev->command_config.bits.software_reset = 1;
	
	counter = 0;
	while (tse_priv->mac_dev->command_config.bits.software_reset) {
		ndelay(100);
		if (counter++ > ALT_TSE_SW_RESET_WATCHDOG_CNTR)
			break;
	}

	if ((counter >= ALT_TSE_SW_RESET_WATCHDOG_CNTR) && 
		(netif_msg_drv(tse_priv)))
	{
		printk(KERN_WARNING "%s: TSEMAC SW reset bit never cleared!\n",
			dev->name);
	}
	
	//default config is enabled HERE including TX and rx
	dat = tse_priv->mac_dev->command_config.image;

	if ((dat & 0x03) && (netif_msg_drv(tse_priv))) {
		printk
		    (KERN_WARNING "%s: RX/TX not disabled after reset... CMD_CONFIG=0x%08x\n",
		     dev->name, dat);
	} else if (netif_msg_drv(tse_priv)) {
		printk(KERN_INFO "%s: OK, counter=%d, CMD_CONFIG=0x%08x\n", 
			dev->name, counter, dat);
	}	
	
	/* Initialize MAC registers */
	tse_priv->mac_dev->max_frame_length = tse_priv->current_mtu;	//ALT_TSE_MAX_FRAME_LENGTH;
	tse_priv->mac_dev->rx_almost_empty_threshold = 8;
	tse_priv->mac_dev->rx_almost_full_threshold = 8;
	tse_priv->mac_dev->tx_almost_empty_threshold = 8;
	tse_priv->mac_dev->tx_almost_full_threshold = 3;
	tse_priv->mac_dev->tx_sel_empty_threshold = tse_priv->tse_tx_depth - 16;
	tse_priv->mac_dev->tx_sel_full_threshold = 0;
	tse_priv->mac_dev->rx_sel_empty_threshold = tse_priv->tse_rx_depth - 16;
	tse_priv->mac_dev->rx_sel_full_threshold = 0;
	
	/*Enable RX shift 16 for alignment of all received frames on 16-bit start address */
	tse_priv->mac_dev->rx_cmd_stat.bits.rx_shift16 = 1;                
	tse_priv->last_rx_shift_16 = 1;
	/* check if the MAC supports the 16-bit shift option at the RX CMD STATUS Register  */
	if (tse_priv->mac_dev->rx_cmd_stat.bits.rx_shift16) {
		tse_priv->rx_shift_16_ok = 1;
	} else if (netif_msg_drv(tse_priv)) {
		tse_priv->rx_shift_16_ok = 0;
		printk(KERN_WARNING "%s: Incompatible with RX_CMD_STAT register return RxShift16 value. \n",
			dev->name);
		return -1;                                                  
	}

	/*Enable TX shift 16 for alignment of all transmitting frames on 16-bit start address */
	tse_priv->mac_dev->tx_cmd_stat.bits.tx_shift16 = 1;
	tse_priv->mac_dev->tx_cmd_stat.bits.omit_crc = 0;
        tse_priv->last_tx_shift_16 = 1;
	/*
	* check if the MAC supports the 16-bit shift option allowing us
	* to send frames without copying. Used by the send function later.
	*/
	if (tse_priv->mac_dev->tx_cmd_stat.bits.tx_shift16) {
		tse_priv->tx_shift_16_ok = 1;
	} else {
		tse_priv->tx_shift_16_ok = 0;
		printk(KERN_WARNING "%s: Incompatible value with TX_CMD_STAT register return TxShift16 value. \n",
			dev->name);
		return -1;
	}	
	
	//pause quanta??
	//tse_priv->mac_dev->pause_quanta=10;
	
	/* enable MAC */
	dat = 0;
	dat = ALTERA_TSE_CMD_TX_ENA_MSK | ALTERA_TSE_CMD_RX_ENA_MSK |

	//enable pause frame generation
//		ALTERA_TSE_CMD_XOFF_GEN_MSK |	
#if ENABLE_PHY_LOOPBACK
		ALTERA_TSE_CMD_PROMIS_EN_MSK |	// promiscuous mode
		ALTERA_TSE_CMD_LOOPBACK_MSK |	// loopback mode
#endif
	    ALTERA_TSE_CMD_RX_ERR_DISC_MSK;	/* automatically discard frames with CRC errors */
	    
	    tse_priv->mac_dev->command_config.image = dat;
	    
	    if (netif_msg_drv(tse_priv))
	    	printk(KERN_INFO "%s: MAC post-initialization: CMD_CONFIG=0x%08x\n",
			dev->name, tse_priv->mac_dev->command_config.image);
		/* Set the MAC address */
	tse_priv->mac_dev->mac_addr_0 = ((tse_priv->dev->dev_addr[3]) << 24 |
				      (tse_priv->dev->dev_addr[2]) << 16 |
				      (tse_priv->dev->dev_addr[1]) << 8 |
				      (tse_priv->dev->dev_addr[0]));

	tse_priv->mac_dev->mac_addr_1 = ((tse_priv->dev->dev_addr[5] << 8 |
				       (tse_priv->dev->dev_addr[4])) & 0xFFFF);

	/* Set the MAC address */
	tse_priv->mac_dev->supp_mac_addr_0_0 = tse_priv->mac_dev->mac_addr_0;
	tse_priv->mac_dev->supp_mac_addr_0_1 = tse_priv->mac_dev->mac_addr_1;

	/* Set the MAC address */
	tse_priv->mac_dev->supp_mac_addr_1_0 = tse_priv->mac_dev->mac_addr_0;
	tse_priv->mac_dev->supp_mac_addr_1_1 = tse_priv->mac_dev->mac_addr_1;

	/* Set the MAC address */
	tse_priv->mac_dev->supp_mac_addr_2_0 = tse_priv->mac_dev->mac_addr_0;
	tse_priv->mac_dev->supp_mac_addr_2_1 = tse_priv->mac_dev->mac_addr_1;

	/* Set the MAC address */
	tse_priv->mac_dev->supp_mac_addr_3_0 = tse_priv->mac_dev->mac_addr_0;
	tse_priv->mac_dev->supp_mac_addr_3_1 = tse_priv->mac_dev->mac_addr_1;

	//tse_priv->mac_dev->command_config.bits.src_mac_addr_sel_on_tx=0;
	return 0;
}    



/*
* Open and Initialize the interface
* The interface is opened whenever 'ifconfig' activates it
*  arg1   : 'net_device' structure pointer 
*  arg2   : new mtu value
*  return : 0                     
*/

/* Jumbo-grams seem to work :-( */
#define TSE_MIN_MTU 64
#define TSE_MAX_MTU 16384

static int tse_change_mtu(struct net_device *dev, int new_mtu)
{
	struct alt_tse_private *tse_priv = netdev_priv(dev);
	unsigned int free_loop;

	if ((new_mtu > (tse_priv->tse_tx_depth * ALT_TSE_MAC_FIFO_WIDTH)) ||
	    (new_mtu > (tse_priv->tse_rx_depth * ALT_TSE_MAC_FIFO_WIDTH))) {
		printk
		    ("Your system doesn't support new MTU size as TX/RX FIFO size is small\n");
		return -EINVAL;
	}

	if (new_mtu < TSE_MIN_MTU || new_mtu > TSE_MAX_MTU)
		return -EINVAL;

	spin_lock(&tse_priv->rx_lock);

	tse_priv->rx_sgdma_dev->control = ALT_SGDMA_CONTROL_SOFTWARERESET_MSK;
	tse_priv->rx_sgdma_dev->control = 0x0;

	tse_priv->current_mtu = new_mtu;
	dev->mtu = new_mtu;
	tse_priv->mac_dev->max_frame_length = tse_priv->current_mtu;

	/* Disable receiver and transmitter  descriptor(SGDAM) */
	for (free_loop = 0; free_loop < ALT_TSE_RX_SGDMA_DESC_COUNT;
	     free_loop++) {
	/* Free the original skb */
		if (tse_priv->rx_skb[free_loop] != NULL)
			dev_kfree_skb(tse_priv->rx_skb[free_loop]);
	}

	/* Prepare RX SGDMA to receive packets */
	sgdma_read_init(dev);

	printk("TSE: new mtu is %d \n", new_mtu);

	spin_unlock(&tse_priv->rx_lock);

	return 0;
}

/*
* Get the current Ethernet statistics.This may be called with the device open or closed.
* arg1   : net device for which multicasts filter is adjusted
* return : network statistics structure
*/
static struct net_device_stats *tse_get_statistics(struct net_device *dev)
{
	struct alt_tse_private *tse_priv = netdev_priv(dev);
	struct net_device_stats *net_status =
	    (struct net_device_stats *)&tse_priv->status;
	
	/* total packets received without error*/
	net_status->rx_packets =
	    tse_priv->mac_dev->aFramesReceivedOK +
	    tse_priv->mac_dev->ifInErrors;
	
	/* total packets received without error*/
	net_status->tx_packets =
	    tse_priv->mac_dev->aFramesTransmittedOK +
	    tse_priv->mac_dev->ifOutErrors;
	
	/* total bytes received without error  */
	net_status->rx_bytes = tse_priv->mac_dev->aOctetsReceivedOK;
	
	/* total bytes transmitted without error   */
	net_status->tx_bytes = tse_priv->mac_dev->aOctetsTransmittedOK;
	                                                               
	/* bad received packets  */
	net_status->rx_errors = tse_priv->mac_dev->ifInErrors;       
	
	/* bad Transmitted packets */
	net_status->tx_errors = tse_priv->mac_dev->ifOutErrors;
	
	/* multicasts packets received */
	net_status->multicast = tse_priv->mac_dev->ifInMulticastPkts;
	
	return net_status;
}

/*
* Program multicasts mac addresses into hash look-up table                   
* arg1    : net device for which multicasts filter is adjusted
* arg2    : multicasts address count
* arg3    : list of multicasts addresses
*/

static void tse_set_hash_table(struct net_device *dev, int count,
			       struct dev_mc_list *addrs)
{
	int mac_octet, xor_bit, bitshift, hash, loop;
	char octet;
	struct dev_mc_list *cur_addr;
	struct alt_tse_private *tse_priv = netdev_priv(dev);
	alt_tse_mac *p_mac_base = tse_priv->mac_dev;

	cur_addr = addrs;
	for (loop = 0; loop < count; loop++, cur_addr = cur_addr->next) 
	{
		/* do we have a pointer here? */
		if (!cur_addr)
			break;

		/* make sure this is a multicasts address    */
		if (!(*cur_addr->dmi_addr & 1))	//
			continue;

		//PRINTK1("dmi_addr %x-%x-%x-%x-%x-%x\n", cur_addr->dmi_addr[0],
		//	cur_addr->dmi_addr[1],
		//	cur_addr->dmi_addr[2],
		//	cur_addr->dmi_addr[3],
		//	cur_addr->dmi_addr[4], cur_addr->dmi_addr[5]);
		hash = 0;	// the hash value

		for (mac_octet = 5; mac_octet >= 0; mac_octet--) {
			xor_bit = 0;
			octet = cur_addr->dmi_addr[mac_octet];
			for (bitshift = 0; bitshift < 8; bitshift++)
				xor_bit ^= (int)((octet >> bitshift) & 0x01);
			hash = (hash << 1) | xor_bit;
			//PRINTK1("\t\thash=%d,xor_bit=%d octet=%x\n", hash,
			//	xor_bit, octet);
		}                

		p_mac_base->hash_table[hash] = 1;                                
	}

}

/*
* Set/Clear multicasts filter
* arg1    : net device for which multicasts filter is adjusted
*           multicasts table from the linked list of addresses
*           associated with this dev structure.
*/
static void tse_set_multicast_list(struct net_device *dev)
{
	struct alt_tse_private *tse_priv = netdev_priv(dev);
	int hash_loop;

	if (dev->flags & IFF_PROMISC) 
	{
		/* Log any net taps */
		//PRINTK1("%s: Promiscuous mode enabled.\n", dev->name);
		tse_priv->mac_dev->command_config.image |=
		    ALTERA_TSE_CMD_PROMIS_EN_MSK;
	} else {
		tse_priv->mac_dev->command_config.image &=
		    ~ALTERA_TSE_CMD_PROMIS_EN_MSK;
	}

	if (dev->flags & IFF_ALLMULTI) {
		for (hash_loop = 0; hash_loop < 64; hash_loop++)
			tse_priv->mac_dev->hash_table[hash_loop] = 1;
	} else {
		for (hash_loop = 0; hash_loop < 64; hash_loop++)
			tse_priv->mac_dev->hash_table[hash_loop] = 0;	// Clear any existing hash entries

		if (dev->mc_count)
			tse_set_hash_table(dev, dev->mc_count, dev->mc_list);
	}
}

/*
* Initialize the MAC address
* arg1    : net device for which TSE MAC driver is registered
* arg2    : address passed from upper layer
* return : 0
*/
static int tse_set_hw_address(struct net_device *dev, void *port)
{
	struct sockaddr *addr = port;
	struct alt_tse_private *tse_priv = netdev_priv(dev);

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

/* Set the MAC address */
//tse_priv->mac_dev->mac_addr_0 = (( dev->dev_addr[2] ) << 24
//                                |( dev->dev_addr[3] ) << 16
//                                |( dev->dev_addr[4] ) <<  8
//                                |( dev->dev_addr[5] ));

//tse_priv->mac_dev->mac_addr_1 = (( dev->dev_addr[0] << 8
//                                |( dev->dev_addr[1] )) & 0xFFFF );

	tse_priv->mac_dev->mac_addr_0 = ((dev->dev_addr[3]) << 24
					 | (dev->dev_addr[2]) << 16
					 | (dev->dev_addr[1]) << 8
					 | (dev->dev_addr[0]));

	tse_priv->mac_dev->mac_addr_1 = ((dev->dev_addr[5] << 8
					  | (dev->dev_addr[4])) & 0xFFFF);

	/* Set the MAC address */
	tse_priv->mac_dev->supp_mac_addr_0_0 = tse_priv->mac_dev->mac_addr_0;
	tse_priv->mac_dev->supp_mac_addr_0_1 = tse_priv->mac_dev->mac_addr_1;

	/* Set the MAC address */
	tse_priv->mac_dev->supp_mac_addr_1_0 = tse_priv->mac_dev->mac_addr_0;
	tse_priv->mac_dev->supp_mac_addr_1_1 = tse_priv->mac_dev->mac_addr_1;

	/* Set the MAC address */
	tse_priv->mac_dev->supp_mac_addr_2_0 = tse_priv->mac_dev->mac_addr_0;
	tse_priv->mac_dev->supp_mac_addr_2_1 = tse_priv->mac_dev->mac_addr_1;

	/* Set the MAC address */
	tse_priv->mac_dev->supp_mac_addr_3_0 = tse_priv->mac_dev->mac_addr_0;
	tse_priv->mac_dev->supp_mac_addr_3_1 = tse_priv->mac_dev->mac_addr_1;

	if (netif_msg_hw(tse_priv))
		printk(KERN_INFO "%s :Set MAC address %pM\n", dev->name,
		       dev->dev_addr);

	return 0;
}

/*******************************************************************************
* Driver Open, shutdown, probe functions
*
*******************************************************************************/


/*
* Open and Initialize the interface
* The interface is opened whenever 'ifconfig' activates it
*  arg1   : 'net_device' structure pointer
*  return : 0
*/
static int tse_open(struct net_device *dev)
{
	struct alt_tse_private *tse_priv = netdev_priv(dev);
	int status;
	int retval = 0;

	/* start NAPI */
	napi_enable(&tse_priv->napi);

	/* Reset and configure TSE MAC and probe associated PHY */
	if(init_phy(dev)) {
		napi_disable(&tse_priv->napi);
		return -EAGAIN;
	}

	if(init_mac(dev)) {
		napi_disable(&tse_priv->napi);
		return -EAGAIN;  	
	}

	/* Initialize SGDMA */
	tse_priv->rx_sgdma_descriptor_tail = 0;
	tse_priv->rx_sgdma_descriptor_head = 0;
	tse_priv->tx_sgdma_descriptor_tail = 0;
	tse_priv->tx_sgdma_descriptor_head = 0;
	
	sgdma_config(tse_priv);

	/* Prepare RX SGDMA to receive packets */
	status = sgdma_read_init(dev);


	/* Register RX SGDMA interrupt */
	retval =
	    request_irq(tse_priv->rx_fifo_interrupt, (void *)alt_sgdma_isr,
			0, "SGDMA_RX", dev);
	if (retval) {
		printk
		    ("%s:Unable to register Rx SGDMA interrupt %d (retval=%d).\n",
		     dev->name, tse_priv->rx_fifo_interrupt, retval);
		napi_disable(&tse_priv->napi);
		return -EAGAIN;
	}
	/* Register TX SGDMA interrupt */
	retval =
	    request_irq(tse_priv->tx_fifo_interrupt, (void *)alt_sgdma_isr,
			0, "SGDMA_TX", dev);
	if (retval) {
		printk
		    ("%s:Unable to register TX SGDMA interrupt %d (retval=%d).\n",
		     dev->name, tse_priv->tx_fifo_interrupt, retval);
		free_irq(tse_priv->rx_fifo_interrupt, (void *)dev);
		napi_disable(&tse_priv->napi);
		return -EAGAIN;
	}
//#ifdef CONFIG_PHY_IRQ_PRESENCE
//#error "ToDo"
//#error "Left due to unavailability of Marvell PHY IRQ connection to NEEK/(3c120)"
///* Register IRQ interrupt */
//	retval =
//	    request_irq(tse_priv->alarm_irq, (void *)alt_tse_link_isr, 0,
//			"TSE_ALARM_LINK", dev);
//	if (retval) {
//		PRINTK3
//		    ("%s:Unable to register timer interrupt %d (retval=%d).\n",
//		     "TSE_ALARM_LINK", tse_priv->alarm_irq, retval);
//		free_irq(tse_priv->rx_fifo_interrupt, (void *)dev);
//		free_irq(tse_priv->tx_fifo_interrupt, (void *)dev);
//		napi_disable(&tse_priv->napi);
//		return -EAGAIN;
//	}
//#else
//	phy_timer.expires = jiffies + msecs_to_jiffies(PHY_TIMER_MSEC);
//	add_timer(&phy_timer);
//#endif
	//start phy
	phy_start(tse_priv->phydev);

	/* Start network queue */     	
	netif_start_queue(dev);
	//tasklet_init(&tse_priv->tse_rx_tasklet, tse_sgdma_rx, (unsigned long)dev);
	return SUCCESS;
}

/*
*  Stop TSE MAC interface - this puts the device in an inactive state
*  arg1   : 'net_device' structure pointer
*  return : 0
*/

static int tse_shutdown(struct net_device *dev)
{
	struct alt_tse_private *tse_priv = netdev_priv(dev);
	unsigned int free_loop;
	int counter;
	

	
	napi_disable(&tse_priv->napi);
	

	//tasklet_kill(&tse_priv->tse_rx_tasklet);

	/* Free interrupt handler */
	free_irq(tse_priv->rx_fifo_interrupt, (void *)dev);
	free_irq(tse_priv->tx_fifo_interrupt, (void *)dev);

#ifdef CONFIG_PHY_IRQ_PRESENCE
	free_irq(tse_priv->alarm_irq, (void *)dev);
#endif

	// disable and reset the MAC, empties fifo
	tse_priv->mac_dev->command_config.bits.software_reset = 1;

	counter = 0;
	while (tse_priv->mac_dev->command_config.bits.software_reset) {
		ndelay(100);
		if (counter++ > ALT_TSE_SW_RESET_WATCHDOG_CNTR)
			break;
	}

	if ((counter >= ALT_TSE_SW_RESET_WATCHDOG_CNTR) && 
		netif_msg_ifdown(tse_priv))
	{
		printk(KERN_WARNING "%s :SHUTDOWN: TSEMAC SW reset bit never cleared!\n",
			dev->name);
	}
	
	spin_lock(&tse_priv->rx_lock);
	spin_lock(&tse_priv->tx_lock);

	//Need to reset/turn off sgdmas
	sgdma_config(tse_priv);

	/* Disable receiver and transmitter  descriptor(SGDAM) */
	for (free_loop = 0; free_loop < ALT_TSE_TX_SGDMA_DESC_COUNT;
	     free_loop++) 
	{
		/* Free the original skb */
		if (tse_priv->tx_skb[free_loop] != NULL) {
			dev_kfree_skb(tse_priv->tx_skb[free_loop]);
			tse_priv->tx_skb[free_loop] = NULL;
		}
	}

	/* Disable receiver and transmitter  descriptor(SGDMA) */
	for (free_loop = 0; free_loop < ALT_TSE_RX_SGDMA_DESC_COUNT;
	     free_loop++) 
	{
		/* Free the original skb */
		if (tse_priv->rx_skb[free_loop] != NULL) {
			dev_kfree_skb(tse_priv->rx_skb[free_loop]);
			tse_priv->rx_skb[free_loop] = NULL;
		}
	}

	spin_unlock(&tse_priv->rx_lock);
	spin_unlock(&tse_priv->tx_lock);

	phy_disconnect(tse_priv->phydev);
	tse_priv->phydev = NULL;

	netif_stop_queue(dev);

	return SUCCESS;
}

static const struct net_device_ops tse_netdev_ops = {
	.ndo_open		= tse_open,
	.ndo_stop		= tse_shutdown,
	.ndo_start_xmit		= tse_hardware_send_pkt,
	.ndo_get_stats		= tse_get_statistics,
	.ndo_set_mac_address	= tse_set_hw_address,
	.ndo_set_multicast_list	= tse_set_multicast_list,
	.ndo_change_mtu		= tse_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
};

/*
* Initialize 'net_device' structure, resets and re-configures MAC and PHY
* arg1   : 'net_device' structure pointer allocated for TSE interface
* arg2   : Interface number
*/
static void __devinit tse_dev_probe(struct net_device *dev)
{
	struct alt_tse_private *tse_priv = netdev_priv(dev);
	struct alt_tse_config *tse_config = tse_priv->tse_config;

	tse_priv->dev = dev;

	dev->base_addr = (unsigned long)tse_priv->mac_dev;	//TSE_MAC_BASE;

	/* Set initial SGDMA descriptor address */
	tse_priv->desc =
	    (volatile struct alt_sgdma_descriptor *)tse_priv->desc_mem_base;
	tse_priv->sgdma_tx_desc =
	    (volatile struct alt_sgdma_descriptor *)tse_priv->desc;
	tse_priv->sgdma_rx_desc =
	    (volatile struct alt_sgdma_descriptor *)&tse_priv->
	    desc[ALT_TSE_TX_SGDMA_DESC_COUNT];

//	tse_priv->antoneg_enable = AUTONEG_ENABLE;

	tse_priv->rx_sgdma_imask =  ( ALT_SGDMA_CONTROL_IE_CHAIN_COMPLETED_MSK \
                                    | ALT_SGDMA_STATUS_DESC_COMPLETED_MSK      \
                                    | ALT_SGDMA_CONTROL_IE_GLOBAL_MSK );
#ifndef NO_TX_IRQ
	tse_priv->tx_sgdma_imask =  ( ALT_SGDMA_CONTROL_IE_CHAIN_COMPLETED_MSK \
                                    | ALT_SGDMA_CONTROL_IE_GLOBAL_MSK );
#else
	tse_priv->tx_sgdma_imask = 0;
#endif

	/* set MTU as max frame size */
	tse_priv->current_mtu = ALT_TSE_MAX_FRAME_LENGTH;

	/* Set default MAC address */
	memcpy(dev->dev_addr, tse_config->ethaddr, ETH_ALEN);

	/* Fill in the fields of the device structure with Ethernet values */
	ether_setup(dev);

	dev->netdev_ops = &tse_netdev_ops;
	tse_set_ethtool_ops(dev);
//	dev->features = NETIF_F_HIGHDMA;

	/* add napi interface */
	netif_napi_add(dev, &tse_priv->napi, tse_poll, ALT_TSE_RX_SGDMA_DESC_COUNT);

#ifdef CONFIG_NET_POLL_CONTROLLER

	dev->poll_controller = tse_net_poll_controller;
#endif

	/* Spin lock variable initialize */
	spin_lock_init(&tse_priv->rx_lock);
	spin_lock_init(&tse_priv->tx_lock);
}

/*
* Driver entry point called by Platform devise driver
* arg    : platform_device object
* return : 'net_device' structure pointer on success else 0
*/
static int __devinit alt_tse_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	int ret = -ENODEV;
	struct resource *res_alt_tse;
	struct alt_tse_private *tse_priv;
	struct alt_tse_config *tse_config = (struct alt_tse_config *) pdev->dev.platform_data;
	resource_size_t mac_dev_base, sgdma_rx_base, sgdma_tx_base, desc_mem_base;
	resource_size_t mac_dev_size, sgdma_rx_size, sgdma_tx_size, desc_mem_size;

	dev = alloc_etherdev(sizeof(struct alt_tse_private));
	if (!dev) {
		printk(KERN_ERR "Could not allocate network device\n");
		return -ENODEV;
	}
	//printk("\n\nTSE DEV NAME : %s", dev->name);
	//sprintf(dev->name, "eth0");
	netdev_boot_setup_check(dev);
	//dev->priv = (void *)ioremap_nocache((unsigned long)dev->priv,
	//				    sizeof(struct alt_tse_private));
	tse_priv = netdev_priv(dev);

	/* Get TSE MAC base address */
	/* ioremap() requires buffer_size as the 2nd argument, but it is not being used inside anyway, so put 0xFFFF */
	/* 1. Get tse MAC resource */
	res_alt_tse = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			TSE_RESOURCE_MAC_DEV);
	if (!res_alt_tse) {
		printk(KERN_ERR "ERROR: %s:%d: platform_get_resource_byname() failed\n", __FILE__, __LINE__);
		ret = -ENODEV;
		goto out_mac_dev;
	}

	mac_dev_base = res_alt_tse->start;
	mac_dev_size = resource_size(res_alt_tse);
	if (!request_mem_region(mac_dev_base, mac_dev_size, "altera_tse")) {
		printk(KERN_ERR "ERROR: %s:%d: request_mem_region() failed\n", __FILE__, __LINE__);
		ret = -EBUSY;
		goto out_mac_dev;
	}

	tse_priv->mac_dev = ioremap_nocache(mac_dev_base, sizeof(alt_tse_mac));
	if (!tse_priv->mac_dev) {
		printk(KERN_ERR "ERROR: %s:%d: ioremap_nocache() failed\n", __FILE__, __LINE__);
		ret = -ENOMEM;
		goto out_mac_dev_ioremap;
	}

	/* Get RX and TX SGDMA addresses */
	/* 2. Get sgdma_rx mem resource */
	res_alt_tse = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			TSE_RESOURCE_SGDMA_RX_DEV);
	if (!res_alt_tse) {
		printk(KERN_ERR "ERROR: %s:%d: platform_get_resource_byname() failed\n", __FILE__, __LINE__);
		ret = -ENODEV;
		goto out_sgdma_rx;
	}

	sgdma_rx_base = res_alt_tse->start;
	sgdma_rx_size = resource_size(res_alt_tse);
	if (!request_mem_region(sgdma_rx_base, sgdma_rx_size, "altera_tse")) {
		printk(KERN_ERR "ERROR: %s:%d: request_mem_region() failed\n", __FILE__, __LINE__);
		ret = -EBUSY;
		goto out_sgdma_rx;
	}

	tse_priv->rx_sgdma_dev = ioremap_nocache(sgdma_rx_base, sizeof(struct alt_sgdma_registers));
	if (!tse_priv->rx_sgdma_dev) {
		printk(KERN_ERR "ERROR: %s:%d: ioremap_nocache() failed\n", __FILE__, __LINE__);
		ret = -ENOMEM;
		goto out_sgdma_rx_ioremap;
	}

	/* 3. Get sgdma_tx mem resource */
	res_alt_tse = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			TSE_RESOURCE_SGDMA_TX_DEV);
	if (!res_alt_tse) {
		printk(KERN_ERR "ERROR: %s:%d: platform_get_resource_byname() failed\n", __FILE__, __LINE__);
		ret = -ENODEV;
		goto out_sgdma_tx;
	}

	sgdma_tx_base = res_alt_tse->start;
	sgdma_tx_size = resource_size(res_alt_tse);
	if (!request_mem_region(sgdma_tx_base, sgdma_tx_size, "altera_tse")) {
		printk(KERN_ERR "ERROR: %s:%d: request_mem_region() failed\n", __FILE__, __LINE__);
		ret = -EBUSY;
		goto out_sgdma_tx;
	}

	tse_priv->tx_sgdma_dev = ioremap_nocache(sgdma_tx_base, sizeof(struct alt_sgdma_registers));
	if (!tse_priv->tx_sgdma_dev) {
		printk(KERN_ERR "ERROR: %s:%d: ioremap_nocache() failed\n", __FILE__, __LINE__);
		ret = -ENOMEM;
		goto out_sgdma_tx_ioremap;
	}

	/* 4. Get sgdma_rx irq resource */
	res_alt_tse = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
			TSE_RESOURCE_SGDMA_RX_IRQ);
	if (!res_alt_tse) {
		printk(KERN_ERR "ERROR: %s:%d: platform_get_resource_byname() failed\n", __FILE__, __LINE__);
		ret = -ENODEV;
		goto out_sgdma_irq;
	}
	tse_priv->rx_fifo_interrupt = res_alt_tse->start;;

	/* 5. Get sgdma_tx irq resource */
	res_alt_tse = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
			TSE_RESOURCE_SGDMA_TX_IRQ);
	if (!res_alt_tse) {
		printk(KERN_ERR "ERROR: %s:%d: platform_get_resource_byname() failed\n", __FILE__, __LINE__);
		ret = -ENODEV;
		goto out_sgdma_irq;
	}
	tse_priv->tx_fifo_interrupt = res_alt_tse->start;

	/* 6. Get sgdma_tx/rx fifo mem resource */

	/* Set TSE MAC RX and TX fifo depth */
	tse_priv->tse_rx_depth = tse_config->rx_fifo_depth;
	tse_priv->tse_tx_depth = tse_config->tx_fifo_depth;

	/* 7. Get sgdma descriptor mem resource */
	res_alt_tse = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			TSE_RESOURCE_SGDMA_DES_DEV);
	if (!res_alt_tse) {
		printk(KERN_ERR "ERROR: %s:%d: platform_get_resource_byname() failed\n", __FILE__, __LINE__);
		ret = -ENODEV;
		goto out_sgdma_irq;
	}

	desc_mem_base = res_alt_tse->start;
	desc_mem_size = resource_size(res_alt_tse);
#if 0
	/* FIXME: this fails and I don't know why...
	 * we don't need it since we'll just ioremap later...
	 */
	if (!request_mem_region (desc_mem_base, desc_mem_size, "altera_tse")) {
		printk("ERROR: %s:%d: request_mem_region() failed\n", __FILE__,
				__LINE__);
		ret = -EBUSY;
		goto out_sgdma_irq;
	}
#endif
	tse_priv->desc_mem_base = (unsigned int) ioremap_nocache(desc_mem_base,
			ALT_TSE_TOTAL_SGDMA_DESC_SIZE);
	if (!tse_priv->desc_mem_base) {
		printk(KERN_ERR "ERROR: %s:%d: ioremap_nocache() failed\n", __FILE__, __LINE__);
		ret = -ENOMEM;
		goto out_desc_mem;
	}

	tse_priv->tse_config = tse_config;

	ret = altera_tse_mdio_register(tse_priv);
	if (ret)
		goto out_desc_mem;

	/* Probe ethernet device */
	tse_dev_probe(dev);

	/* Register ethernet device driver */
	ret = register_netdev(dev);
	if (ret) {
		printk(KERN_ERR "%s: Failed to register TSE net device\n",
				dev->name);
		goto out;
	}

	SET_NETDEV_DEV(dev, &pdev->dev);
	platform_set_drvdata(pdev, dev);

	printk("%s", version);

	return 0;

out:
	iounmap((void *) tse_priv->desc_mem_base);
out_desc_mem:
#if 0
	release_mem_region(desc_mem_base, desc_mem_size);
#endif
out_sgdma_irq:
	iounmap((void *) tse_priv->tx_sgdma_dev);
out_sgdma_tx_ioremap:
	release_mem_region(sgdma_tx_base, sgdma_tx_size);
out_sgdma_tx:
	iounmap((void *) tse_priv->rx_sgdma_dev);
out_sgdma_rx_ioremap:
	release_mem_region(sgdma_rx_base, sgdma_rx_size);
out_sgdma_rx:
	iounmap((void *) tse_priv->mac_dev);
out_mac_dev_ioremap:
	release_mem_region(mac_dev_base, mac_dev_size);
out_mac_dev:
	free_netdev(dev);
	return ret;
}

static int __devexit alt_tse_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);

	/* TODO: Release all mem regions and do iounmap, see error paths of
	 * alt_tse_probe above */
	platform_set_drvdata(pdev, NULL);
	unregister_netdev(ndev);
	free_netdev(ndev);

	return 0;
}

/*******************************************************************************
* Driver init, platform driver register
*
*******************************************************************************/
static struct platform_driver alt_tse_driver = {
	.driver = {
		   .name = ALT_TSE_NAME,
		   .owner = THIS_MODULE,
		   },
	.probe = alt_tse_probe,
	.remove = __devexit_p(alt_tse_remove),
	.suspend = NULL,
	.resume = NULL,
};

static int __init atse_init(void)
{
	/* probe board and register */
	return platform_driver_register(&alt_tse_driver);
}

static void __exit atse_exit(void)
{
	platform_driver_unregister(&alt_tse_driver);
	printk("ALT:%s:%d:%s \n", __FILE__, __LINE__, __FUNCTION__);
}

module_init(atse_init);
module_exit(atse_exit);

MODULE_AUTHOR("Altera");
MODULE_DESCRIPTION("Altera Triple Speed MAC IP");
MODULE_LICENSE("GPL");
