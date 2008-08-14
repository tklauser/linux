
/*
 *  linux/drivers/net/altera_tse.c
 *
 * Copyright (C) 2008 Altera Corporation.
 *
 * History:
 *    o  SLS  - Linux 2.6.23
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

static const char version[] =
    "Altera Triple Speed MAC IP Driver"
    "(v8.0)"
    "Linux 2.6.26 JULY-2008\n";

#include <linux/module.h>     /* for module-version */
#include <linux/kernel.h>     /* printk(), and other useful stuff */
#include <linux/sched.h>      /* for jiffies, HZ, etc. */
#include <linux/string.h>     /* inline memset(), etc. */
#include <linux/ptrace.h>
#include <linux/errno.h>      /* return codes */
#include <linux/ioport.h>     /* request_region(), release_region() */
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/inet.h>
#include <linux/netdevice.h>  /* struct device, and other headers */
#include <linux/etherdevice.h>/* eth_type_trans */
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/init.h>       /* __init (when not using as a module) */

#include <linux/pm.h>  /* pm_message_t */
#include <linux/platform_device.h>

#include <asm/irq.h>          /* For NR_IRQS only. */
#include <asm/pgtable.h>
#include <asm/page.h>
//#include <asm/nios.h>       //for 2.6.26
#include <asm/cacheflush.h>

#include <asm/processor.h>     /* Processor type for cache alignment. */
#include <asm/bitops.h>
#include <asm/io.h>            /* I/O functions */
#include <asm/uaccess.h>       /* User space memory access functions */

#include "altera_tse.h"


/* DEBUG flags */
#define DEBUG_INFO     0
#define DEBUG_WARNING  0
#define DEBUG_ERROR    1

#if DEBUG_INFO == 1
    #define PRINTK1(args...) printk(args)
#else
    #define PRINTK1(args...)
#endif

#if DEBUG_WARNING == 1
    #define PRINTK2(args...) printk(args)
#else
    #define PRINTK2(args...)
#endif

#if DEBUG_ERROR == 1
    #define PRINTK3(args...) printk(args)
#else
    #define PRINTK3(args...)
#endif

/*
 * This structure is private to each device. It is used to pass
 * packets in and out, so there is place for a packet
 */
struct  alt_tse_private{
  struct net_device_stats status;
  struct net_device       *dev;

  alt_tse_mac             *mac_dev;

  volatile struct alt_sgdma_registers     *rx_sgdma_dev;
  volatile struct alt_sgdma_registers     *tx_sgdma_dev;

  struct alt_tse_phy_profile     *pphy_profile;
  unsigned int            phy_address;

  unsigned int            tx_fifo_interrupt;
  unsigned int            rx_fifo_interrupt;
  unsigned char           tx_shift_16_ok;
  unsigned char           rx_shift_16_ok;

  unsigned int            desc_mem_base;                    /* Base address of Descriptor Memory if ext_desc_mem = 1 */
  unsigned int            chain_loop;

  unsigned int            tse_tx_depth;                     /* TX Receive FIFO depth                                 */
  unsigned int            tse_rx_depth;                     /* RX Receive FIFO depth                                 */

  // Location for the SGDMA Descriptors
  volatile struct alt_sgdma_descriptor    *desc;
  volatile struct alt_sgdma_descriptor    *sgdma_rx_desc;
  volatile struct alt_sgdma_descriptor    *sgdma_tx_desc;

  volatile struct alt_sgdma_descriptor    *desc_pointer;

  unsigned int            current_mtu;

  unsigned int            rx_sgdma_descriptor_tail;
  unsigned int            rx_sgdma_descriptor_head;

  unsigned int            tse_tx_sgdma_cur;

  struct sk_buff          *rx_skb[ALT_TSE_RX_SGDMA_DESC_COUNT];
  struct sk_buff          *tx_skb[ALT_TSE_TX_SGDMA_DESC_COUNT];

  /* Tasklets for handling hardware IRQ related operations outside hw IRQ handler */
  struct tasklet_struct   tse_rx_tasklet;
  int                     sem;
  spinlock_t              rx_lock;
  spinlock_t              tx_lock;

  /* system info */
  unsigned int            link_status;
  unsigned int            speed;
  unsigned int            duplex;
  unsigned int            phy_speed;
  unsigned int            phy_duplex;
  unsigned int            antoneg_enable;

  /*link info */
  unsigned int            alarm_irq;
  struct timer_reg        *alarm_link_check;

} ;

#if DEBUG_INFO == 1
static void tse_print_packet(unsigned int , int );
static void phy_print_profile ( struct alt_tse_private * );
#endif

#ifdef CONFIG_NET_POLL_CONTROLLER
static void tse_net_poll_controller(struct net_device *dev);
#endif

unsigned int marvell_phy_cfg( alt_tse_mac * );
static unsigned int phy_mdio_reg_write( struct alt_tse_private *,unsigned char ,unsigned char,unsigned char ,unsigned short);
static unsigned int phy_mdio_reg_read(struct alt_tse_private *,unsigned char,unsigned char,unsigned char);
static void tse_set_mac_speed_duplex ( struct alt_tse_private *);
static void tse_phy_check_media ( struct alt_tse_private*  );
static unsigned int phy_restart_auto_negotiate ( struct alt_tse_private*  );
static unsigned int tse_phy_probe ( struct alt_tse_private*  );
static irqreturn_t alt_tse_link_isr( int irq, void *dev_id, struct pt_regs *regs);
static void sgdma_config(struct alt_tse_private* );
static int tse_mac_phy_config ( struct alt_tse_private*  );
static void alt_sgdma_construct_descriptor_burst(volatile struct alt_sgdma_descriptor *,volatile struct alt_sgdma_descriptor *,
                     unsigned int *,unsigned int *,unsigned short ,int,int,int,int,int,unsigned char);
static int alt_sgdma_do_async_transfer( volatile struct alt_sgdma_registers *, volatile struct alt_sgdma_descriptor *);
#if 0
static unsigned char  alt_sgdma_do_sync_transfer (volatile struct alt_sgdma_registers * , volatile struct alt_sgdma_descriptor *);
#endif
static void alt_sgdma_construct_stream_to_mem_desc( volatile struct alt_sgdma_descriptor *,volatile struct alt_sgdma_descriptor *,
                     unsigned int *,unsigned short,int);
static inline void alt_sgdma_construct_mem_to_stream_desc( volatile struct alt_sgdma_descriptor *,volatile struct alt_sgdma_descriptor *,
                     unsigned int *,unsigned short ,int,int,int, unsigned char);
static struct sk_buff  * tse_alloc_rx_skb ( struct net_device *, unsigned int  );
static  int sgdma_async_read ( struct alt_tse_private *, volatile struct alt_sgdma_descriptor *);
static unsigned int sgdma_read_init (struct net_device *);
static int tse_sgdma_rx(struct net_device *);
static irqreturn_t alt_sgdma_rx_isr( int , void *, struct pt_regs * );
static irqreturn_t alt_sgdma_tx_isr( int , void *, struct pt_regs * );
static inline unsigned int sgdma_async_write ( struct alt_tse_private* , volatile struct alt_sgdma_descriptor *);
static int tse_hardware_send_pkt ( struct sk_buff * , struct net_device * );
static struct net_device_stats *tse_get_statistics ( struct net_device * );
static void tse_set_hash_table( struct net_device *, int count, struct dev_mc_list * );
static int tse_set_hw_address ( struct net_device * , void * );
static int tse_change_mtu(struct net_device *, int );
static int tse_open(struct net_device *);
static int tse_shutdown(struct net_device *);
static void tse_get_drvinfo ( struct net_device *, struct ethtool_drvinfo * );
static unsigned int tse_get_link ( struct net_device * );
static int tse_get_settings(struct net_device *, struct ethtool_cmd *);
static int tse_set_settings(struct net_device *, struct ethtool_cmd *);


#if DEBUG_INFO == 1
/* Display RX/TX buffer
 * arg1     :buffer address
 * arg2     :length of buffer
 */
static void tse_print_packet(unsigned int add, int len)
{
  int offset;
  printk("ipacket: add = %x len = %d\n", add, len);
  for(offset = 0; offset < len; offset++) {
    if(!(offset % 16))
       printk("\n");
    printk(" %.2x", *(((unsigned char *)add) + offset));
  }
  printk("\n");
}
#endif

/* configure marvell phy
 * arg1     :  TSE private data structure
 * return   :  0
 */
unsigned int marvell_phy_cfg( alt_tse_mac *p_mac_base )
{
  unsigned short dat;

   /* If there is no link yet, we enable auto crossover and reset the PHY */
  if(( p_mac_base->mdio_phy1.status & 0x10 ) == 0)  {
    PRINTK1("MARVELL : Enabling auto crossover\n");
//    p_mac_base->mdio_phy1.control = 0x0078;
    PRINTK1("MARVELL : PHY reset\n");
    dat = p_mac_base->mdio_phy1.control;
//    p_mac_base->mdio_phy1.control = dat | 0x8000;
  }

  return 0;
}

/* Write value of data with bit_length number of bits to MDIO register based
 * on register location reg_num and start from bit location lsb_num.
 * arg1      : TSE private data structure
 * arg2      :location of MDIO register to be written.
 * arg3      :least significant bit location of MDIO register to be written.
 * arg4      :number of bits to be written to the register.
 * arg5      :data to be written to the register at specific bit location of register.
 * return     SUCCESS
 */

static unsigned int phy_mdio_reg_write( struct alt_tse_private *p_tse,
                                     unsigned char reg_num,
                                     unsigned char lsb_num,
                                     unsigned char bit_length,
                                     unsigned short data)
{
  unsigned short temp_data;
  unsigned short bit_mask;
  unsigned int   bit_shift;
  unsigned int   *mdio_reg = ( unsigned int * ) &p_tse->mac_dev->mdio_phy1;

  bit_mask = 0x00;

  /* generate mask consist of bit_length number of 1
   * eg: bit_length = 3, bit_mask = 0b0000 0000 0000 0111
   */
  for( bit_shift = 0 ; bit_shift < bit_length ; bit_shift++ ) {
    bit_mask <<= 1;
    bit_mask |= 0x01;
  }

  /* shifting mask to left by bit_num */
  bit_mask <<= lsb_num;

  /* read register data */
  temp_data = mdio_reg[reg_num];

  /* clear bits to be written */
  temp_data &= ~bit_mask;

  /* OR-ed together corresponding bits data */
  temp_data |= ((data << lsb_num) & bit_mask);

  /* write data to MDIO register */
  mdio_reg[reg_num] = temp_data;

  return SUCCESS;

}

/* Read bit_length number of bits from MDIO register based on register location reg_num
 * and start from bit location lsb_num.
 * arg1     : TSE private data structure
 * arg2     : reg_num          location of MDIO register to be read.
 * arg3     : lsb_num          least significant bit location of MDIO register to be read.
 * arg4     : bit_length       number of bits to be read from the register.
 * return   : data read from MDIO register
 */
static unsigned int phy_mdio_reg_read(struct alt_tse_private *p_tse,
                               unsigned char reg_num,
                               unsigned char lsb_num,
                               unsigned char bit_length)
{
  unsigned short temp_data;
  unsigned int   bit_mask;
  unsigned int   bit_shift;
  unsigned int   *mdio_reg = ( unsigned int * ) &p_tse->mac_dev->mdio_phy1;

  bit_mask = 0x00;
  /* generate mask consist of bit_length number of 1
   * eg: bit_length = 3, bit_mask = 0b0000 0000 0000 0111
   */
  for(bit_shift = 0; bit_shift < bit_length; bit_shift++) {
    bit_mask <<= 1;
    bit_mask |= 0x01;
  }

  /* read register data */
  temp_data = mdio_reg[reg_num];

  /* shifting read data */
  temp_data >>= lsb_num;

  return (temp_data & bit_mask);
}

#if DEBUG_INFO == 1
/* Display PHYs found in listed profile.
 * arg1     :TSE private data structure
 */
static void phy_print_profile ( struct alt_tse_private *p_tse )
{
  struct alt_tse_phy_profile   *pphy_profile = p_tse->pphy_profile;

  /* display PHY in profile */

  printk("PHY Name        : %s\n",     pphy_profile->name);
  printk("PHY OUI         : 0x%06x\n", (int)pphy_profile->oui);
  printk("PHY Model Num.  : 0x%02x\n", pphy_profile->model_number);
  printk("PHY Rev. Num.   : 0x%02x\n", pphy_profile->revision_number);
  printk("Status Register : 0x%02x\n", pphy_profile->status_reg_location);
  printk("Speed Bit       : %d\n",     pphy_profile->speed_lsb_location);
  printk("Duplex Bit      : %d\n",     pphy_profile->duplex_bit_location);
  printk("Link Bit        : %d\n\n",   pphy_profile->link_bit_location);

}
#endif


/* Restart Auto-Negotiation for the PHY
 * arg1     :TSE private data structure
 * return   :TSE_PHY_AN_COMPLETE if success
 *          :TSE_PHY_AN_NOT_COMPLETE if auto-negotiation not completed
 *          :TSE_PHY_AN_NOT_CAPABLE if the PHY not capable for AN
 */
static unsigned int phy_restart_auto_negotiate ( struct alt_tse_private* p_tse )
{

  /* Record previous MDIO address, to be restored at the end of function */
  unsigned int temp_mdio_phy1_addr = p_tse->mac_dev->mdio_phy1_addr;
  unsigned int timeout_threshold = 0;

  /* write PHY address to MDIO to access the i-th PHY */
  p_tse->mac_dev->mdio_phy1_addr = p_tse->phy_address;

  if(!phy_mdio_reg_read ( p_tse, TSE_PHY_MDIO_STATUS, TSE_PHY_MDIO_STATUS_AN_ABILITY, 1) ) {
    PRINTK3("ERROR   : PHY - PHY not capable for Auto-Negotiation\n");
    return TSE_PHY_AN_NOT_CAPABLE;
  }

  /* enable Auto-Negotiation */
  phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_CONTROL, TSE_PHY_MDIO_CONTROL_AN_ENA, 1, 1 );

  /* send PHY reset command */
  phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_CONTROL, TSE_PHY_MDIO_CONTROL_RESTART_AN, 1, 1 );
  PRINTK1("INFO    : PHY - Restart Auto-Negotiation, checking PHY link...\n");

  while( phy_mdio_reg_read ( p_tse, TSE_PHY_MDIO_STATUS, TSE_PHY_MDIO_STATUS_AN_COMPLETE, 1 ) == 0 ) {
    if( timeout_threshold++ > ALT_AUTONEG_TIMEOUT_THRESHOLD ) {
      PRINTK1("WARNING : PHY - Auto-Negotiation FAILED\n");
      return TSE_PHY_AN_NOT_COMPLETE;
    }
  }

  PRINTK1("INFO    : PHY - Auto-Negotiation PASSED\n");

  /* Restore previous MDIO address */
  p_tse->mac_dev->mdio_phy1_addr = temp_mdio_phy1_addr;

  return TSE_PHY_AN_COMPLETE;
}

/* @Function Description: Set the advertisement of PHY for 1000 Mbps
 * @API Type:    Internal
 * @param pmac   Pointer to the alt_tse_phy_info structure
 *        enable set Enable = 1 to advertise this speed if the PHY capable
 *               set Enable = 0 to disable advertise of this speed
 * @return       return SUCCESS
 */
int tse_phy_set_adv_1000(struct alt_tse_private* p_tse , unsigned char enable)
{
  /* Record previous MDIO address, to be restored at the end of function */
  unsigned int temp_mdio_phy1_addr = p_tse->mac_dev->mdio_phy1_addr;
  unsigned char cap;


  /* write PHY address to MDIO to access the i-th PHY */
  p_tse->mac_dev->mdio_phy1_addr = p_tse->phy_address;

    /* if enable = 1, set advertisement based on PHY capability */
    if(enable) {
        cap =  phy_mdio_reg_read ( p_tse, TSE_PHY_MDIO_EXT_STATUS,
                                          TSE_PHY_MDIO_EXT_STATUS_1000BASE_T_FULL, 1);
        phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_1000BASE_T_CTRL,
                                    TSE_PHY_MDIO_1000BASE_T_CTRL_FULL_ADV, 1, cap);
        PRINTK3("INFO    : PHY - Advertisement of 1000 Base-T Full Duplex set to %d\n", cap);

        /* 1000 Mbps Half duplex not supported by TSE MAC */
        cap = phy_mdio_reg_read ( p_tse, TSE_PHY_MDIO_EXT_STATUS,
                                         TSE_PHY_MDIO_EXT_STATUS_1000BASE_T_HALF, 1);
        phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_1000BASE_T_CTRL,
                                    TSE_PHY_MDIO_1000BASE_T_CTRL_HALF_ADV, 1, cap);
        PRINTK3("INFO    : PHY - Advertisement of 1000 Base-T Half Duplex set to %d\n", cap);

    }  else {/* else disable advertisement of this speed */

        phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_1000BASE_T_CTRL,
                                    TSE_PHY_MDIO_1000BASE_T_CTRL_FULL_ADV, 1, 0);
        PRINTK3("INFO    : PHY - Advertisement of 1000 Base-T Full Duplex set to %d\n", 0);

        /* 1000 Mbps Half duplex not supported by TSE MAC */
        phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_1000BASE_T_CTRL,
                                    TSE_PHY_MDIO_1000BASE_T_CTRL_HALF_ADV, 1, 0);
        PRINTK3("INFO    : PHY - Advertisement 1000 Base-T half Duplex set to %d\n", 0);
    }

  /* Restore previous MDIO address */
  p_tse->mac_dev->mdio_phy1_addr = temp_mdio_phy1_addr;

  return SUCCESS;
}


/* @Function Description: Set the advertisement of PHY for 100 Mbps
 * @API Type:    Internal
 * @param pmac   Pointer to the alt_tse_phy_info structure
 *        enable set Enable = 1 to advertise this speed if the PHY capable
 *               set Enable = 0 to disable advertise of this speed
 * @return       return SUCCESS
 */
int tse_phy_set_adv_100(struct alt_tse_private* p_tse, unsigned char  enable)
{
  /* Record previous MDIO address, to be restored at the end of function */
  unsigned int temp_mdio_phy1_addr = p_tse->mac_dev->mdio_phy1_addr;
  unsigned char cap;

  /* write PHY address to MDIO to access the i-th PHY */
  p_tse->mac_dev->mdio_phy1_addr = p_tse->phy_address;

  /* if enable = 1, set advertisement based on PHY capability */
  if(enable) {
    cap = phy_mdio_reg_read ( p_tse, TSE_PHY_MDIO_STATUS, TSE_PHY_MDIO_STATUS_100BASE_T4, 1);
    phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_ADV, TSE_PHY_MDIO_ADV_100BASE_T4, 1, cap);
    PRINTK3("INFO  : PHY - Advertisement of 100 Base-T4 set to %d\n",cap);

    cap = phy_mdio_reg_read ( p_tse, TSE_PHY_MDIO_STATUS, TSE_PHY_MDIO_STATUS_100BASE_X_FULL, 1);
    phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_ADV, TSE_PHY_MDIO_ADV_100BASE_TX_FULL, 1, cap);
    PRINTK3("INFO  : PHY - Advertisement of 100 Base-TX Full Duplex set to %d\n", cap);

    cap = phy_mdio_reg_read ( p_tse, TSE_PHY_MDIO_STATUS, TSE_PHY_MDIO_STATUS_100BASE_X_HALF, 1);
    phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_ADV, TSE_PHY_MDIO_ADV_100BASE_TX_HALF, 1, cap);
    PRINTK3("INFO  : PHY - Advertisement of 100 Base-TX Half Duplex set to %d\n", cap);

  } else {  /* else disable advertisement of this speed */

    phy_mdio_reg_write ( p_tse,TSE_PHY_MDIO_ADV, TSE_PHY_MDIO_ADV_100BASE_T4, 1, 0);
    PRINTK3("INFO  : PHY - Advertisement of 100 Base-T4 set to %d\n", 0);

    phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_ADV, TSE_PHY_MDIO_ADV_100BASE_TX_FULL, 1, 0);
    PRINTK3("INFO  : PHY - Advertisement of 100 Base-TX Full Duplex set to %d\n", 0);

    phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_ADV, TSE_PHY_MDIO_ADV_100BASE_TX_HALF, 1, 0);
    PRINTK3("INFO  : PHY - Advertisement of 100 Base-TX Half Duplex set to %d\n", 0);
  }

  /* Restore previous MDIO address */
  p_tse->mac_dev->mdio_phy1_addr = temp_mdio_phy1_addr;

  return SUCCESS;

}


/* @Function Description: Set the advertisement of PHY for 10 Mbps
 * @API Type:    Internal
 * @param pmac   Pointer to the alt_tse_phy_info structure
 *        enable set Enable = 1 to advertise this speed if the PHY capable
 *               set Enable = 0 to disable advertise of this speed
 * @return       return SUCCESS
 */
int tse_phy_set_adv_10(struct alt_tse_private* p_tse, unsigned char enable)
{
  /* Record previous MDIO address, to be restored at the end of function */
  unsigned int temp_mdio_phy1_addr = p_tse->mac_dev->mdio_phy1_addr;
  unsigned char cap;

  /* write PHY address to MDIO to access the i-th PHY */
  p_tse->mac_dev->mdio_phy1_addr = p_tse->phy_address;

  /* if enable = 1, set advertisement based on PHY capability */
  if(enable) {
        cap = phy_mdio_reg_read ( p_tse, TSE_PHY_MDIO_STATUS, TSE_PHY_MDIO_STATUS_10BASE_T_FULL, 1);
        phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_ADV, TSE_PHY_MDIO_ADV_10BASE_TX_FULL, 1, cap);
        PRINTK3("INFO    : PHY - Advertisement of 10 Base-TX Full Duplex set to %d\n", cap);

        cap = phy_mdio_reg_read ( p_tse, TSE_PHY_MDIO_STATUS, TSE_PHY_MDIO_STATUS_10BASE_T_HALF, 1);
        phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_ADV, TSE_PHY_MDIO_ADV_10BASE_TX_HALF, 1, cap);
        PRINTK3("INFO    : PHY - Advertisement of 10 Base-TX Half Duplex set to %d\n", cap);

  } else {   /* else disable advertisement of this speed */

        phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_ADV, TSE_PHY_MDIO_ADV_10BASE_TX_FULL, 1, 0);
        PRINTK3("INFO    : PHY - Advertisement of 10 Base-TX Full Duplex set to %d\n", 0);

        phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_ADV, TSE_PHY_MDIO_ADV_10BASE_TX_HALF, 1, 0);
        PRINTK3("INFO    : PHY - Advertisement of 10 Base-TX Half Duplex set to %d\n", 0);
  }

  /* Restore previous MDIO address */
  p_tse->mac_dev->mdio_phy1_addr = temp_mdio_phy1_addr;

  return SUCCESS;
}


static unsigned int tse_get_link_speed( struct alt_tse_private *p_tse )
{
  unsigned int speed;
  struct alt_tse_phy_profile     *pphy_profile = p_tse->pphy_profile;

  speed = phy_mdio_reg_read ( p_tse,p_tse->pphy_profile->status_reg_location,
                        p_tse->pphy_profile->speed_lsb_location,p_tse->pphy_profile->speed_bits_length);

  if((pphy_profile->oui==DP83848C_OUI)
                      &&(pphy_profile->model_number==DP83848C_MODEL)
                      &&(pphy_profile->revision_number==DP83848C_REV))
    speed = ((speed & 0x1) ^ 0x1) ;

  return speed;

}

/* @Function Description: Set the common speed to all PHYs connected to the MAC within the same group
 * @API Type:               Internal
 * @param pmac_group        Pointer to the TSE MAC Group structure which group all the MACs that should use the same speed
 *        common_speed      common speed supported by all PHYs
 * @return      common speed supported by all PHYs connected to the MAC, return TSE_PHY_SPEED_NO_COMMON if invalid common speed specified
 */
static void tse_phy_set_speed_duplex_antoneg( struct alt_tse_private *p_tse )
{

  /* Record previous MDIO address, to be restored at the end of function */
  unsigned int temp_mdio_phy1_addr = p_tse->mac_dev->mdio_phy1_addr,common_speed=TSE_PHY_SPEED_100;

  /* write PHY address to MDIO to access the i-th PHY */
  p_tse->mac_dev->mdio_phy1_addr = p_tse->phy_address;


  if ( p_tse->speed == SPEED_1000 )
    common_speed = TSE_PHY_SPEED_1000;
  else if ( p_tse->speed == SPEED_100 )
    common_speed = TSE_PHY_SPEED_100;
  else if ( p_tse->speed == SPEED_10 )
    common_speed = TSE_PHY_SPEED_10;

//  /* set Auto-Negotiation advertisement based on common speed */
  if ( p_tse->speed == SPEED_1000 ){
      tse_phy_set_adv_1000(p_tse, 1);
      tse_phy_set_adv_100(p_tse, 1);
      tse_phy_set_adv_10(p_tse, 1);

  } else if ( p_tse->speed == SPEED_100 ) {
      tse_phy_set_adv_1000(p_tse, 0);
      tse_phy_set_adv_100(p_tse, 1);
      tse_phy_set_adv_10(p_tse, 1);

  } else if ( p_tse->speed == SPEED_10 ) {
      tse_phy_set_adv_1000(p_tse, 0);
      tse_phy_set_adv_100(p_tse, 0);
      tse_phy_set_adv_10(p_tse, 1);

  } else {
      tse_phy_set_adv_1000(p_tse, 0);
      tse_phy_set_adv_100(p_tse, 0);
      tse_phy_set_adv_10(p_tse, 0);
  }

  /*Duplex setting*/
  phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_CONTROL, TSE_PHY_MDIO_CONTROL_DUPLEX, 1, p_tse->duplex);
  /* write msb of speed */
  phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_CONTROL, TSE_PHY_MDIO_CONTROL_SPEED_MSB, 1, common_speed >> 1);
  /* write lsb of speed */
  phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_CONTROL, TSE_PHY_MDIO_CONTROL_SPEED_LSB, 1, common_speed);

  phy_restart_auto_negotiate ( p_tse );

  PRINTK3("INFO    : PHY set to common speed : %d Mbps common_speed=%d\n",  p_tse->speed,common_speed);

  p_tse->mac_dev->mdio_phy1_addr = temp_mdio_phy1_addr;

  return ;
}
/* Set TSE speed and duplex mode
 * arg1     :TSE private data structure
 * arg2     :speed
 * arg3     :duplex
 */
static void tse_set_mac_speed_duplex ( struct alt_tse_private *p_tse)
{
  unsigned int refvar;

  refvar = p_tse->mac_dev->command_config.image;

  /*Speed setting*/
  if ( p_tse->speed == SPEED_1000 ){
    refvar |= ALTERA_TSE_CMD_ETH_SPEED_MSK;
    refvar &= ~ALTERA_TSE_CMD_ENA_10_MSK;
  } else if ( p_tse->speed == SPEED_100 ) {
    refvar &= ~ALTERA_TSE_CMD_ETH_SPEED_MSK;
    refvar &= ~ALTERA_TSE_CMD_ENA_10_MSK;
  } else if ( p_tse->speed == SPEED_10 ) {
    refvar &= ~ALTERA_TSE_CMD_ETH_SPEED_MSK;
    refvar |= ALTERA_TSE_CMD_ENA_10_MSK;
  }

  /*Duplex setting*/
  if(p_tse->duplex == DUPLEX_HALF)
    refvar |= ALTERA_TSE_CMD_HD_ENA_MSK;
  else
    refvar &= ~ALTERA_TSE_CMD_HD_ENA_MSK;

  p_tse->mac_dev->command_config.image=refvar;

  return ;
}

/* Check media link status
 * arg1     :TSE private data structure
 */
static void tse_phy_check_media ( struct alt_tse_private* p_tse )
{

  /* Record previous MDIO address, to be restored at the end of function */
  unsigned int temp_mdio_phy1_addr = p_tse->mac_dev->mdio_phy1_addr,speed;
  struct alt_tse_phy_profile   *pphy_profile = p_tse->pphy_profile;

  /* write PHY address to MDIO to access the i-th PHY */
  p_tse->mac_dev->mdio_phy1_addr = p_tse->phy_address;

  speed = tse_get_link_speed(p_tse);

  if(speed == TSE_PHY_SPEED_10)
    p_tse->speed = SPEED_10;
  else if(speed ==TSE_PHY_SPEED_100)
    p_tse->speed = SPEED_100;
  else if(speed ==TSE_PHY_SPEED_1000)
    p_tse->speed = SPEED_1000;
  else
    p_tse->speed = SPEED_10;

  p_tse->phy_speed = speed;

  p_tse->duplex = phy_mdio_reg_read(p_tse,pphy_profile->status_reg_location,
                                    pphy_profile->duplex_bit_location,1);

  p_tse->link_status = phy_mdio_reg_read(p_tse,pphy_profile->status_reg_location,
                                    pphy_profile->link_bit_location,1);

  if( p_tse->link_status )
    netif_carrier_on(p_tse->dev);
  else
    netif_carrier_off(p_tse->dev);

  p_tse->mac_dev->mdio_phy1_addr = temp_mdio_phy1_addr;

}

/* Find phy profile structure and reset and autonagotiate PHY.
 * If the link speed cannot be determined, it is fall back to 10/100.
 * arg1     :TSE private data structure
 * return   : unsinged integer variable having following bit pattern
 *  --------------------------------------------------------------------------------
 * | 31-6  | Reserved                                                               |
 * |    5  | 1: PHY auto-negotiation not completed                                  |
 * |    4  | 1: No PHY profile match                                                |
 * |    3  | 1: 10 Mbps link                                                        |
 * |    2  | 1: 100 Mbps link                                                       |
 * |    1  | 1: 1000 Mbps link                                                      |
 * |    0  | 1: Full Duplex   0: Half Duplex                                        |
 *  --------------------------------------------------------------------------------
 */
static unsigned int tse_phy_probe ( struct alt_tse_private* p_tse )
{
  unsigned int  phyid,phyid2 = 0,phyadd,is_phy_in_profile = 0;
  unsigned int           speed, duplex, result=0,phy_ref;
  alt_tse_mac   *p_mac_base = p_tse->mac_dev;

  /* Record previous MDIO address, to be restored at the end of function */
  unsigned int  temp_mdio_phy1_addr = p_mac_base->mdio_phy1_addr;

  /* PHY OUI (Organizationally Unique Identififier) */
  unsigned int  oui;

  /* PHY model number */
  unsigned char model_number;

  /* PHY revision number */
  unsigned char revision_number;

  p_tse->phy_address = -1;

  /* loop all valid PHY address to look for connected PHY */
  for ( phyadd = 0x00; phyadd < 0x20; phyadd++ ){

    p_mac_base->mdio_phy1_addr = phyadd;
    phyid  = p_mac_base->mdio_phy1.phy_id1;      // read PHY ID
    phyid2 = p_mac_base->mdio_phy1.phy_id2;      //read PHY ID
    /* PHY found */
    if (phyid != phyid2){
      /* get oui, model number, and revision number from PHYID and PHYID2 */
      oui = (phyid << 6) | ((phyid2 >> 10) & 0x3f);
      model_number = (phyid2 >> 4) & 0x3f;
      revision_number = phyid2 & 0x0f;

      PRINTK1("INFO    : PHY OUI             =  0x%06x\n", (int) oui);
      PRINTK1("INFO    : PHY Model Number    =  0x%02x\n", model_number);
      PRINTK1("INFO    : PHY Revision Number =  0x%01x\n", revision_number);

      /* map the PHY with PHY in profile */
      is_phy_in_profile = 0;
      for(phy_ref = 0; phy_ref < TSE_MAX_PHY_PROFILE; phy_ref++) {
         PRINTK1("INFO    (%d):oui=0x%06x,model_number = 0x%02x\n",
                          phy_ref,phy_profiles[phy_ref].oui, phy_profiles[phy_ref].model_number);

        /* if PHY match with PHY in profile */
        if((phy_profiles[phy_ref].oui == oui) && (phy_profiles[phy_ref].model_number == model_number)){
          p_tse->pphy_profile = &phy_profiles[phy_ref];
          /* PHY found, add it to phy_list */
          PRINTK1("INFO    : PHY %s found at PHY address 0x%02x of MAC\n", phy_profiles[phy_ref].name, phyadd);
          is_phy_in_profile = 1;
          break;
        }
      }//inner for loop closing
      /* PHY not found in PHY profile */
      if(is_phy_in_profile == 0){
        PRINTK2("WARNING : Unknown PHY found at PHY address 0x%02x of MAC \n", phyadd);
        PRINTK2("WARNING : Please add PHY information to PHY profile\n");
      }
      break;
    }//If closing

  }//for loop closing

  if(is_phy_in_profile) { /* if PHY is found in profile */

    /* store PHY address */
    p_tse->phy_address = phyadd;

    /* Disable PHY loopback to allow Auto-Negotiation completed */
    phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_CONTROL, TSE_PHY_MDIO_CONTROL_LOOPBACK, 1, 0 );   // disable PHY loopback
    if ( phy_restart_auto_negotiate ( p_tse ) != TSE_PHY_AN_COMPLETE )
      result = ALT_TSE_E_AN_NOT_COMPLETE;

    /* Perform additional setting if there is any */
    /* Profile specific */
    if( p_tse->pphy_profile->phy_cfg ){
      PRINTK3("INFO  : Applying additional PHY configuration of %s\n", p_tse->pphy_profile->name);
      p_tse->pphy_profile->phy_cfg ( p_mac_base );
    }

    if(phy_mdio_reg_read ( p_tse,p_tse->pphy_profile->status_reg_location,
                                 p_tse->pphy_profile->link_bit_location,1 ) )  {
      speed  = tse_get_link_speed(p_tse);//phy_mdio_reg_read ( p_tse,p_tse->pphy_profile->status_reg_location,p_tse->pphy_profile->speed_lsb_location,p_tse->pphy_profile->speed_bits_length);
      duplex = phy_mdio_reg_read ( p_tse,p_tse->pphy_profile->status_reg_location,
                                   p_tse->pphy_profile->duplex_bit_location,1 );
      p_tse->link_status =1;
    } else {
      PRINTK3("WARNING : Link is not available\n");
      speed  = ALT_TSE_DEFAULT_SPEED; // 100 Mbps
      duplex = ALT_TSE_DEFAULT_DUPLEX_MODE;

      /* Set Phy with 100Mbps full duplex */
      //phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_CONTROL, TSE_PHY_MDIO_CONTROL_SPEED_LSB, 1, 1 );
      //phy_mdio_reg_write ( p_tse, TSE_PHY_MDIO_CONTROL, TSE_PHY_MDIO_CONTROL_DUPLEX, 1, 1 );

      p_tse->link_status=0;
    }
  } else { /* if PHY not found in profile */

    PRINTK3("WARNING : PHY - PHY not found in PHY profile\n");
    speed  = ALT_TSE_DEFAULT_SPEED;  // 100 Mbps
    duplex = ALT_TSE_DEFAULT_DUPLEX_MODE;
    result = ALT_TSE_E_NO_PHY_PROFILE;
    p_tse->link_status=0;
  }

  if(speed == TSE_PHY_SPEED_10)
    p_tse->speed = SPEED_10;
  else if(speed ==TSE_PHY_SPEED_100)
    p_tse->speed = SPEED_100;
  else if(speed ==TSE_PHY_SPEED_1000)
    p_tse->speed = SPEED_1000;
  else
    p_tse->speed = SPEED_10;

  p_tse->phy_speed = speed;
  p_tse->duplex = duplex;

  if( p_tse->link_status )
    netif_carrier_on(p_tse->dev);
  else
    netif_carrier_off(p_tse->dev);

  result |= ((duplex & 0x01)|
            (((speed==TSE_PHY_SPEED_1000)?1:0) << 1) |
            (((speed==TSE_PHY_SPEED_100)?1:0)  << 2) |
            (((speed==TSE_PHY_SPEED_10)?1:0)   << 3) );


  PRINTK3("INFO : PHY - speed = %s, Duplex = %s\n",
                        (speed == 2) ? "1Gbps":(speed == 1)? "100Mbps" : "10Mbps",
                         duplex == 1 ? "Full" : "Half");
 /* Restore previous MDIO address */
  p_mac_base->mdio_phy1_addr = temp_mdio_phy1_addr;
  return result;
 }


/*
*
*Link check
*
**/
static void tse_link_monitor( struct alt_tse_private* p_tse )
{
  /* Record previous MDIO address, to be restored at the end of function */
  unsigned int temp_mdio_phy1_addr = p_tse->mac_dev->mdio_phy1_addr;
  unsigned int link,speed,duplex,update=0;

  /* write PHY address to MDIO to access the i-th PHY */
  p_tse->mac_dev->mdio_phy1_addr = p_tse->phy_address;

  link=phy_mdio_reg_read( p_tse , p_tse->pphy_profile->status_reg_location,
                          p_tse->pphy_profile->link_bit_location , 1);

  if( link )  {
    if(p_tse->link_status==0)    {
      printk("Link UP\n");
      update = 1;
      netif_carrier_on(p_tse->dev);
    } else {
      speed =tse_get_link_speed( p_tse );//phy_mdio_reg_read ( p_tse,p_tse->pphy_profile->status_reg_location,p_tse->pphy_profile->speed_lsb_location,p_tse->pphy_profile->speed_bits_length);
      if( speed != p_tse->phy_speed )   {
        printk("Speed is changed \n");
        update = 1;
      } else {
        duplex = phy_mdio_reg_read ( p_tse,p_tse->pphy_profile->status_reg_location,
                                     p_tse->pphy_profile->duplex_bit_location,1 );
        if(duplex !=p_tse->duplex) {
          printk("Duplex is changed \n");
          update = 1;
        }
      }
    }
  } else {
    if(p_tse->link_status) {
      p_tse->link_status=0;
      printk("Link DOWN\n" );
      netif_carrier_off(p_tse->dev);
    }
  }

  if( update ) {
     if(p_tse->antoneg_enable)
       phy_restart_auto_negotiate ( p_tse );
     tse_phy_check_media ( p_tse );
     tse_set_mac_speed_duplex ( p_tse );
     printk("LINK : speed = %dMbps, Duplex = %d\n",p_tse->speed,p_tse->duplex );
  }

  p_tse->mac_dev->mdio_phy1_addr=temp_mdio_phy1_addr;

  return;
}

#ifdef CONFIG_PHY_IRQ_PRESENCE

/* phy interrupt routing
 * arg1     :irq number
 * arg2     :user data passed to isr
 * arg3     :pt_resgs structure passed from kernel
 */
static irqreturn_t alt_tse_link_isr( int irq, void *dev_id, struct pt_regs *regs)
{
  struct  net_device *dev = dev_id;
  struct alt_tse_private* tse_priv = (struct alt_tse_private*) dev->priv;

  /*Clear Irq */
  //TO DO

  tse_link_monitor( tse_priv );

  return SUCCESS;
}

#else

/* Timer interrupt routing
 * arg1     :irq number
 * arg2     :user data passed to isr
 * arg3     :pt_resgs structure passed from kernel
 */
static irqreturn_t alt_tse_link_isr( int irq, void *dev_id, struct pt_regs *regs)
{
  struct  net_device *dev = dev_id;
  struct alt_tse_private* tse_priv = (struct alt_tse_private*) dev->priv;

  /*Clear Irq */
  tse_priv->alarm_link_check->status = 0; // clear interrupt flag

  tse_link_monitor( tse_priv );

  tse_priv->alarm_link_check->control   =  ( TIMER_CONTROL_ITO_MSK | TIMER_CONTROL_START_MSK );

  return SUCCESS;
}
#endif

/* Clear descriptor memory ,initialize SGDMA descriptor chain and reset SGDMA.
 * arg1     :TSE private data structure
 * @return  :1 on success
 *           less than zero on error
 */
static void sgdma_config(struct alt_tse_private* p_tse)
{
  unsigned int mem_off,*mem_ptr=(unsigned int *)p_tse->desc_mem_base,
               mem_size=ALT_TSE_TOTAL_SGDMA_DESC_SIZE,chain_loop;

  //Clearing SGDMA desc Memory
  for (mem_off = 0 ; mem_off < mem_size; mem_off+=4)
    *mem_ptr++ = 0x00000000;

  //Setting Desc Memory Pointer


  for(chain_loop = 0;chain_loop < ALT_TSE_RX_SGDMA_DESC_COUNT; chain_loop++)
    p_tse->sgdma_rx_desc[chain_loop].descriptor_status &=
                                                  (~ALT_SGDMA_DESCRIPTOR_STATUS_TERMINATED_BY_EOP_MSK);

  p_tse->rx_sgdma_dev->control = ALT_SGDMA_CONTROL_SOFTWARERESET_MSK;
  p_tse->rx_sgdma_dev->control = 0x0;
}

/* Determine link speed our PHY negotiated with our link partner and
 * initialize TSE MAC core register
 * arg1     :TSE private data structure
 * @return  :1 on success
 *           less than zero on error
 */
static int tse_mac_phy_config ( struct alt_tse_private* p_tse )
{
  int dat;
  int duplex, result, counter;

  /* Detect & init PHY*/
  result = tse_phy_probe ( p_tse );
  if( result & ALT_TSE_E_NO_PHY_PROFILE )
    return -1;

  /* Check media and fill private data */
  tse_phy_check_media ( p_tse );

  /* reset the mac */
  p_tse->mac_dev->command_config.bits.transmit_enable=1;
  p_tse->mac_dev->command_config.bits.receive_enable=1;
  p_tse->mac_dev->command_config.bits.software_reset=1;

  counter=0;
  while( p_tse->mac_dev->command_config.bits.software_reset ) {
    if( counter++ > ALT_TSE_SW_RESET_WATCHDOG_CNTR )
      break;
  }

  if(counter >= ALT_TSE_SW_RESET_WATCHDOG_CNTR)
    PRINTK3("TSEMAC SW reset bit never cleared!\n");

  dat = p_tse->mac_dev->command_config.image;

  if( (dat&0x03)  != 0 )
    PRINTK2("WARN: RX/TX not disabled after reset... missing PHY clock? CMD_CONFIG=0x%08x\n", dat);
  else
    PRINTK1("OK, counter=%d, CMD_CONFIG=0x%08x\n", counter, dat);

  /* Initialize MAC registers */
  p_tse->mac_dev->max_frame_length = p_tse->current_mtu;//ALT_TSE_MAX_FRAME_LENGTH;
  p_tse->mac_dev->rx_almost_empty_threshold = 8;
  p_tse->mac_dev->rx_almost_full_threshold = 8;
  p_tse->mac_dev->tx_almost_empty_threshold = 8;
  p_tse->mac_dev->tx_almost_full_threshold = 3;
  p_tse->mac_dev->tx_sel_empty_threshold = p_tse->tse_tx_depth - 16;
  p_tse->mac_dev->tx_sel_full_threshold = 0;
  p_tse->mac_dev->rx_sel_empty_threshold = p_tse->tse_rx_depth - 16;
  p_tse->mac_dev->rx_sel_full_threshold = 0;

  /*Enable RX shift 16 for alignment of all received frames on 16-bit start address */
  p_tse->mac_dev->rx_cmd_stat.bits.rx_shift16=1;

  /* check if the MAC supports the 16-bit shift option at the RX CMD STATUS Register  */
  if(p_tse->mac_dev->rx_cmd_stat.bits.rx_shift16) {
    p_tse->rx_shift_16_ok = 1;
  } else {
    p_tse->rx_shift_16_ok = 0;
    PRINTK3("Error: Incompatible with RX_CMD_STAT register return RxShift16 value. \n");
    return -1;
  }

  /*Enable TX shift 16 for alignment of all transmitting frames on 16-bit start address */
  p_tse->mac_dev->tx_cmd_stat.bits.tx_shift16 = 1;

  /*
   * check if the MAC supports the 16-bit shift option allowing us
   * to send frames without copying. Used by the send function later.
   */
  if(p_tse->mac_dev->tx_cmd_stat.bits.tx_shift16) {
    p_tse->tx_shift_16_ok = 1;
  } else {
    p_tse->tx_shift_16_ok = 0;
    PRINTK3("Error: Incompatible value with TX_CMD_STAT register return TxShift16 value. \n");
    return -1;
  }

  /* enable MAC */
  dat = 0;
  duplex = result & 0x01;
  dat = ALTERA_TSE_CMD_TX_ENA_MSK       |
        ALTERA_TSE_CMD_RX_ENA_MSK       |
        #if ENABLE_PHY_LOOPBACK
         ALTERA_TSE_CMD_PROMIS_EN_MSK   |     // promiscuous mode
         ALTERA_TSE_CMD_LOOPBACK_MSK    |     // loopback mode
        #endif
        ALTERA_TSE_CMD_RX_ERR_DISC_MSK  ;     /* automatically discard frames with CRC errors */

  /*Speed setting*/
  if ( result & 0x2 ){ // 1 Gbps
    dat |= ALTERA_TSE_CMD_ETH_SPEED_MSK;
    dat &= ~ALTERA_TSE_CMD_ENA_10_MSK;
  } else if ( result & 0x4 ){ //100 Mbps
    dat &= ~ALTERA_TSE_CMD_ETH_SPEED_MSK;
    dat &= ~ALTERA_TSE_CMD_ENA_10_MSK;
  } else if ( result & 0x8 ){//10 Mbps
    dat &= ~ALTERA_TSE_CMD_ETH_SPEED_MSK;
    dat |= ALTERA_TSE_CMD_ENA_10_MSK;
  }

  /*Duplex setting*/
  if(duplex == 0)
    dat |= ALTERA_TSE_CMD_HD_ENA_MSK;

  p_tse->mac_dev->command_config.image = dat;
  PRINTK1("MAC post-initialization: CMD_CONFIG=0x%08x\n",p_tse->mac_dev->command_config.image );

  /* Set the MAC address */
  p_tse->mac_dev->mac_addr_0 = (( p_tse->dev->dev_addr[2] ) << 24 |
                               ( p_tse->dev->dev_addr[3] ) << 16  |
                               ( p_tse->dev->dev_addr[4] ) <<  8  |
                               ( p_tse->dev->dev_addr[5] ));

  p_tse->mac_dev->mac_addr_1 = (( p_tse->dev->dev_addr[0] << 8 |
                               ( p_tse->dev->dev_addr[1] )) & 0xFFFF );

  /* Set the MAC address */
  p_tse->mac_dev->supp_mac_addr_0_0 = p_tse->mac_dev->mac_addr_0;
  p_tse->mac_dev->supp_mac_addr_0_1 = p_tse->mac_dev->mac_addr_1;

  /* Set the MAC address */
  p_tse->mac_dev->supp_mac_addr_1_0 = p_tse->mac_dev->mac_addr_0;
  p_tse->mac_dev->supp_mac_addr_1_1 = p_tse->mac_dev->mac_addr_1;

  /* Set the MAC address */
  p_tse->mac_dev->supp_mac_addr_2_0 = p_tse->mac_dev->mac_addr_0;
  p_tse->mac_dev->supp_mac_addr_2_1 = p_tse->mac_dev->mac_addr_1;

  /* Set the MAC address */
  p_tse->mac_dev->supp_mac_addr_3_0 = p_tse->mac_dev->mac_addr_0;
  p_tse->mac_dev->supp_mac_addr_3_1 = p_tse->mac_dev->mac_addr_1;

  //p_tse->mac_dev->command_config.bits.src_mac_addr_sel_on_tx=0;
  return 1;

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
static void alt_sgdma_construct_descriptor_burst(volatile struct alt_sgdma_descriptor *desc,
                                volatile struct alt_sgdma_descriptor *next,
                                unsigned int         *read_addr,
                                unsigned int         *write_addr,
                                unsigned short        length_or_eop,
                                int                   generate_eop,
                                int                   read_fixed,
                                int                   write_fixed_or_sop,
                                int                   read_burst,
                                int                   write_burst,
                                unsigned char         atlantic_channel)
{
  /*
   * Mark the "next" descriptor as "not" owned by hardware. This prevents
   * The SGDMA controller from continuing to process the chain. This is
   * done as a single IO write to bypass cache, without flushing
   * the entire descriptor, since only the 8-bit descriptor status must
   * be flushed.
   */
  next->descriptor_control = ( next->descriptor_control
                               & ~ALT_SGDMA_DESCRIPTOR_CONTROL_OWNED_BY_HW_MSK );

  desc->source            = read_addr;
  desc->destination       = write_addr;
  desc->next              = (unsigned int *) next;
  desc->source_pad            = 0x0;
  desc->destination_pad       = 0x0;
  desc->next_pad              = 0x0;
  desc->bytes_to_transfer        = length_or_eop;
  desc->actual_bytes_transferred = 0;
  desc->descriptor_status              = 0x0;

  /* SGDMA burst not currently supported */
  desc->read_burst               = 0;//read_burst;  //TBD
  desc->write_burst              = 0;//write_burst; //TBD

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
            (read_fixed ?   ALT_SGDMA_DESCRIPTOR_CONTROL_READ_FIXED_ADDRESS_MSK : 0x0)  |
            (write_fixed_or_sop ?  ALT_SGDMA_DESCRIPTOR_CONTROL_WRITE_FIXED_ADDRESS_MSK : 0x0) |
            (atlantic_channel ? ( (atlantic_channel & 0x0F) << 3) : 0)
          );

  /*
   * Flush completed buffer out of cache. This is done rather than
   * individual cache-bypassed writes to take advantage of any
   * burst-capabilities in the memory we're writing to
   */

}


/* Set up and commence a non-blocking transfer of one of more descriptors
 * (or descriptor chain). If the SGDMA controller is busy at the time of this call, the
 * routine will immediately return -EBUSY; the application can then
 * decide how to proceed without being blocked
 * arg1     :pointer of SGDMA register structure.
 * arg2     :Pointer of first descriptor structure of RX SGDMA chain
 * return   :0 on success
 *           -EBUSY on error
 */
static int alt_sgdma_do_async_transfer( volatile struct alt_sgdma_registers *dev,
                                        volatile struct alt_sgdma_descriptor *desc)
{
  unsigned int control;

  /* Return with error immediately if controller is busy */
  if( dev->status & ALT_SGDMA_STATUS_BUSY_MSK) {
    PRINTK2("Busy bits is 1\n");
    return -EBUSY;
  }

  /*
   * Clear any (previous) status register information
   * that might occlude our error checking later.
   */
  dev->status = 0xFF;

  /* Point the controller at the descriptor */
  dev->next_descriptor_pointer=(unsigned int)desc;


  /*
   * If a callback routine has been previously registered which will be
   * called from the SGDMA ISR. Set up controller to:
   *  - Run
   *  - Stop on an error with any particular descriptor
   *  - Include any control register bits registered with along with
   *    the callback routine (effectively, interrupts are controlled
   *    via the control bits set during callback-register time).
   */
    control = dev->control;
    control |= (ALT_SGDMA_CONTROL_IE_CHAIN_COMPLETED_MSK |
                ALT_SGDMA_CONTROL_IE_GLOBAL_MSK          |
                ALT_SGDMA_CONTROL_RUN_MSK                |
                ALT_SGDMA_CONTROL_STOP_DMA_ER_MSK  );

    dev->control = control;

  /*
   * Error detection/handling should be performed at the application
   * or callback level as appropriate.
   */

  return 0;
}

#if 0
/* Initialize RX SGDMA to receive frame from RX MAC streaming module to data memory.
 * This routine will block both before transfer (if the controller is busy), and until
 * the requested transfer has completed.
 * arg1     :pointer of SGDMA register structure.
 * arg2     :Pointer of first descriptor structure of TX SGDMA chain
 * return   :Content of SGDMA status register
 */
static  unsigned char  alt_sgdma_do_sync_transfer (volatile struct alt_sgdma_registers *dev ,
                                                   volatile struct alt_sgdma_descriptor *desc)
{
  volatile unsigned int status;

  /* Wait for any pending transfers to complete */
  status =dev->status;

  while (dev->status & ALT_SGDMA_STATUS_BUSY_MSK){}

  /*
   * Clear any (previous) status register information
   * that might occlude our error checking later.
   */
  dev->status=0xFF;

  /* Point the controller at the descriptor */
  dev->next_descriptor_pointer=(unsigned int)desc;

  /*
   * Set up SGDMA controller to:
   * - Disable interrupt generation
   * - Run once a valid descriptor is written to controller
   * - Stop on an error with any particular descriptor
   */
  dev->control       = (  ALT_SGDMA_CONTROL_RUN_MSK |
                          ALT_SGDMA_CONTROL_STOP_DMA_ER_MSK |
                          dev->control );

  /* Wait for the descriptor (chain) to complete */
  status =dev->status;
  while (dev->status & ALT_SGDMA_STATUS_BUSY_MSK){}

  /* Clear Run */
  dev->control = (dev->control &(~ALT_SGDMA_CONTROL_RUN_MSK));

  /* Get & clear status register contents */
  status = dev->status;
  dev->status=0xFF;

  return status;
}
#endif

/* Initialize RX SGDMA to receive frame from RX MAC streaming module to data memory.
 * arg1     :pointer to first TX SGDMA descriptor.
 * arg2     :pointer to next TX SGDMA descriptor.
 * arg3     :address to where data will be received from RX MAC module
 * arg4     :no of byte to transmit.
 * arg5     :0
 */
static void alt_sgdma_construct_stream_to_mem_desc( volatile struct alt_sgdma_descriptor *desc,
                                   volatile struct alt_sgdma_descriptor *next,
                                   unsigned int         *write_addr,
                                   unsigned short       length_or_eop,
                                   int                  write_fixed)
{
    alt_sgdma_construct_descriptor_burst(
        desc,
        next,
        (unsigned int) 0x0,     /* Read addr: not required for stream-to-mem mode*/
        write_addr,
        length_or_eop,
        0x0,                    /* Generate EOP: not required for stream-to-mem mode*/
        0x0,                    /* Read fixed: not required for stream-to-mem mode*/
        write_fixed,
        0,                      /* Read_burst : not required for stream-to-mem mode*/
        0,                      /* write_burst*/
        (unsigned char) 0x0     /* Atlantic channel: not required for stream-to-mem mode*/
        );
}

/* Initialize TX SGDMA to transmit frame from data memory to  TX MAC streaming module.
 * arg1     :pointer to first TX SGDMA descriptor.
 * arg2     :pointer to next TX SGDMA descriptor.
 * arg3     :address from where data will be transmitted to TX MAC module
 * arg4     :no of byte to transmit.
 * arg5     :0
 * arg6     :variable indicating to generate start of packet or not
 * arg7     :variable indicating to generate end of packet or not
 * arg8     :0
 */
static inline void alt_sgdma_construct_mem_to_stream_desc( volatile struct alt_sgdma_descriptor *desc,
                                  volatile struct alt_sgdma_descriptor *next,
                                  unsigned int         *read_addr,
                                  unsigned short        length,
                                  int                   read_fixed,
                                  int                   generate_sop,
                                  int                   generate_eop,
                                  unsigned char         atlantic_channel)
{
  alt_sgdma_construct_descriptor_burst(
        desc,
        next,
        read_addr,
        (unsigned int) 0x0,     /*write address: not required for mem-to-stream mode*/
        length,
        generate_eop,
        read_fixed,
        generate_sop,
        0,                      /* read_burst */
        0,                      /* not required for mem-to-stream mode*/
        atlantic_channel
        );
}


/* Allocate skb structure memory from kernel space.
 * arg1     :TSE private data structure
 * arg2     :Pointer to first descriptor structure of RX SGDMA chain
 * return   pointer of allocated skb
 *          less than 0 on errors
 */
static struct sk_buff  * tse_alloc_rx_skb ( struct net_device *dev, unsigned int skb_size )
{
  struct sk_buff  *skb;

  /* Do not overwrite any of the map or rp information
   * until we are sure we can commit to a new buffer.
   *
   * Callers depend upon this behavior and assume that
   * we leave everything unchanged if we fail.
   */
  skb = dev_alloc_skb( skb_size );
  if ( skb == NULL ) {
    PRINTK3("ENOMEM:::skb_size=%d\n",skb_size);
    return NULL;
  }
  skb->dev = dev;

  return skb;
}

/* Start to copy from rxFIFO into given buffer memory area with Asynchronous .so the
 * function does not return the actual bytes transferred for current descriptor
 * arg1     :TSE private data structure
 * arg2     :Pointer to first descriptor structure of RX SGDMA chain
 * return   SUCCESS on success
 *          less than 0 on errors
 */
static  int sgdma_async_read ( struct alt_tse_private *p_tse,
                               volatile struct alt_sgdma_descriptor *rx_desc)
{
  unsigned int          timeout;
  unsigned int          retval = 0;

  /* Make sure SGDMA controller is not busy from a former command */
  timeout = 0;

  PRINTK1("\nWaiting while rx SGDMA is busy.........");

  /* Wait for the descriptor (chain) to complete */
  while ( p_tse->rx_sgdma_dev->status & ALT_SGDMA_STATUS_BUSY_MSK){
    if(timeout++ == ALT_TSE_SGDMA_BUSY_WATCHDOG_CNTR) {
        PRINTK3("RX SGDMA Timeout\n");
        return -EBUSY;
    }
  }
  p_tse->rx_sgdma_dev->control = 0;
  p_tse->rx_sgdma_dev->control = ALTERA_TSE_SGDMA_INTR_MASK;
  /* SGDMA operation invoked for RX (non-blocking call)*/
  retval = alt_sgdma_do_async_transfer(
                 p_tse->rx_sgdma_dev,
                (volatile struct alt_sgdma_descriptor *) rx_desc);

  return retval;
}

/* tse_sgdma_next_rx_descriptor number
 * given current, returns next
 */
static unsigned int tse_sgdma_next_rx_descriptor (int current_descriptor)
{
  if(current_descriptor == (ALT_TSE_RX_SGDMA_DESC_COUNT-1))
    return 0;
  else
    return (current_descriptor + 1);
}

/* Create TSe descriptor for next buffer
 * or error if no buffer available
 *
 */
static int tse_sgdma_add_buffer (struct net_device *dev)
{
  struct alt_tse_private *tse_priv = (struct alt_tse_private *)dev->priv;
  int next_head = 0;

  tse_priv->rx_skb[tse_priv->rx_sgdma_descriptor_head]=tse_alloc_rx_skb(dev,tse_priv->current_mtu);//ALT_TSE_MAX_FRAME_LENGTH+4);
  dcache_push(((unsigned long) (tse_priv->rx_skb[tse_priv->rx_sgdma_descriptor_head])), sizeof(struct sk_buff));

  next_head = tse_sgdma_next_rx_descriptor(tse_priv->rx_sgdma_descriptor_head);
  if ( next_head == tse_priv->rx_sgdma_descriptor_tail )
    return -EBUSY;

  PRINTK1("INFO :addr=%x\n",tse_priv);
  alt_sgdma_construct_stream_to_mem_desc(
    (volatile struct alt_sgdma_descriptor *) &tse_priv->sgdma_rx_desc[tse_priv->rx_sgdma_descriptor_head],  // descriptor I want to work with
    (volatile struct alt_sgdma_descriptor *) &tse_priv->sgdma_rx_desc[next_head],// pointer to "next"
    (unsigned int *) tse_priv->rx_skb[tse_priv->rx_sgdma_descriptor_head]->data,// tse_priv->rx_buffer, //                                                      // starting write_address
    0,                                                                                         // read until EOP
    0);                                                                                        // don't write to constant address

  tse_priv->rx_sgdma_descriptor_head = next_head;

  return SUCCESS;
}

/* Init and setup SGDMA Descriptor chain.
 * arg1     :TSE private data structure
 * return    SUCCESS
 */
static unsigned int sgdma_read_init (struct net_device *dev)
{
  struct alt_tse_private *tse_priv = ( struct alt_tse_private *)dev->priv;
  int rx_loop;

  tse_priv->rx_sgdma_descriptor_tail = 0;
  tse_priv->rx_sgdma_descriptor_head = 0;

  for(rx_loop = 0; rx_loop < ALT_TSE_RX_SGDMA_DESC_COUNT ; rx_loop++) {
    //Should do some checking here.
    tse_sgdma_add_buffer(dev);
  }
  sgdma_async_read( tse_priv, &tse_priv->sgdma_rx_desc[tse_priv->rx_sgdma_descriptor_tail] ) ;

  PRINTK1("Read init is completed\n");
  return SUCCESS;
}
/* Pass packets from RX FIFO buffer to kernel TCP/IP stack
 * and initialize next buffer descriptor chain.
 * arg1     :Network device to which packet is sent
 * return    SUCCESS on success
 */
static int tse_sgdma_rx(struct net_device *dev)
{
  unsigned int          rx_bytes,netif_rx_status;
  struct alt_tse_private       *tse_priv = (struct alt_tse_private*) dev->priv;
  volatile struct alt_sgdma_descriptor  *temp_desc_pointer;
  unsigned char         *skb_Rxbuffer;
  struct sk_buff        *skb;

  temp_desc_pointer = &tse_priv->sgdma_rx_desc[tse_priv->rx_sgdma_descriptor_tail];

  skb = tse_priv->rx_skb[tse_priv->rx_sgdma_descriptor_tail] ;

  tse_priv->rx_skb[tse_priv->rx_sgdma_descriptor_tail] = NULL ;

  if(temp_desc_pointer->descriptor_status & ALT_SGDMA_DESCRIPTOR_STATUS_TERMINATED_BY_EOP_MSK ) {
    rx_bytes  = temp_desc_pointer->actual_bytes_transferred;
    rx_bytes -= ALIGNED_BYTES;

    if(rx_bytes > tse_priv->current_mtu) {
       dev_kfree_skb_irq( skb );
       return SUCCESS;
    }

    temp_desc_pointer->descriptor_status &= (~ALT_SGDMA_DESCRIPTOR_STATUS_TERMINATED_BY_EOP_MSK);

    /* Align IP header to 32 bits */
    skb_reserve(skb, ALIGNED_BYTES);

    skb_Rxbuffer=skb_put(skb, rx_bytes);
    //tse_print_packet(skb_Rxbuffer,rx_bytes);
    skb->protocol = eth_type_trans(skb,dev);
    dcache_push(((unsigned long) (skb_Rxbuffer)), rx_bytes);
    dcache_push(((unsigned long) (skb)), sizeof(struct sk_buff));

    netif_rx_status = netif_rx(skb);

    switch (netif_rx_status) {
    case NET_RX_DROP:
         PRINTK3("%s :NET_RX_DROP occurred\n",dev->name);
         tse_priv->status.rx_dropped++;
         break;
    case NET_RX_SUCCESS:
         PRINTK1("NET_RX_SUCCESS with %d bytes with %d descriptor\n",
                            rx_bytes,tse_priv->rx_sgdma_descriptor_tail);
         break;
    default:
         PRINTK3("%s: netif_rx rtnsts: %08X\n",dev->name,netif_rx_status);
         break;
    }
  }
  else {//Error handling
     dev_kfree_skb_irq( skb );
  }

  return SUCCESS;
}

/* RX SG-DMA FIFO interrupt routing
 * arg1     :irq number
 * arg2     :user data passed to isr
 * arg3     :pt_resgs structure passed from kernel
 */
static irqreturn_t alt_sgdma_rx_isr( int irq, void *dev_id, struct pt_regs *regs )
{
   struct  net_device *dev = dev_id;
   struct alt_tse_private* tse_priv = (struct alt_tse_private*) dev->priv;

   tse_priv->rx_sgdma_dev->control = (tse_priv->rx_sgdma_dev->control | 0x80000000) ;

   if( tse_priv->rx_sgdma_dev->status & 0x0F ) {

     spin_lock(&tse_priv->rx_lock);

     PRINTK1("RX Avalon - ST interrupt with status=%x\n",tse_priv->rx_sgdma_dev->status);

     tse_sgdma_rx( dev );

     //increment tail
     tse_priv->rx_sgdma_descriptor_tail = tse_sgdma_next_rx_descriptor(tse_priv->rx_sgdma_descriptor_tail);

     //allocate next desc anf skb
     //check if it works
     if (tse_sgdma_add_buffer(dev) == -EBUSY)
       printk("ah, something happened, and no desc was added to rx");

     if(tse_priv->rx_sgdma_dev->status & ALT_SGDMA_STATUS_CHAIN_COMPLETED_MSK) {

       tse_priv->rx_sgdma_dev->control = 0;
       tse_priv->rx_sgdma_dev->control = ALTERA_TSE_SGDMA_INTR_MASK;

       //for now, what if we are busy?
       if(tse_priv->rx_sgdma_dev->status & ALT_SGDMA_STATUS_BUSY_MSK)
         printk("restarting rx sgdma while still busy?");

       sgdma_async_read( tse_priv, &tse_priv->sgdma_rx_desc[tse_priv->rx_sgdma_descriptor_tail] ) ;
     }

     spin_unlock(&tse_priv->rx_lock);
   } else {
     PRINTK3("Unexpected RX Avalon - ST interrupt with status=%x\n",tse_priv->rx_sgdma_dev->status);
   }

   return SUCCESS;
}


/* TX SG-DMA FIFO interrupt routing
 * arg1     :irq number
 * arg2     :user data passed to isr
 * arg3     :pt_resgs structure passed from kernel
 */
static irqreturn_t alt_sgdma_tx_isr( int irq, void *dev_id, struct pt_regs *regs)
{
   struct  net_device *dev = dev_id;
   struct alt_tse_private* tse_priv = (struct alt_tse_private*) dev->priv;
   volatile struct alt_sgdma_descriptor *tx_next_desc =(volatile struct alt_sgdma_descriptor *) tse_priv->tx_sgdma_dev->next_descriptor_pointer;

   tse_priv->tx_sgdma_dev->control |= ALT_SGDMA_CONTROL_CLEAR_INTERRUPT_MSK;

   if( tse_priv->tx_sgdma_dev->status & (ALT_SGDMA_STATUS_CHAIN_COMPLETED_MSK |
                                         ALT_SGDMA_STATUS_ERROR_MSK) ) {

      tx_next_desc = (volatile struct alt_sgdma_descriptor *) tx_next_desc->next;

      if(tx_next_desc->descriptor_control & ALT_SGDMA_DESCRIPTOR_CONTROL_OWNED_BY_HW_MSK){
        PRINTK3("Avalon - ST next discriptor=%08x\n",(unsigned int)tx_next_desc);
        /* Clear the status and control bits of the TX SGDMA descriptor */
        tse_priv->tx_sgdma_dev->control = 0;
        tse_priv->tx_sgdma_dev->status  = 0xFF;

        /* Start TX SGDMA (blocking call) */
        alt_sgdma_do_async_transfer ( tse_priv->tx_sgdma_dev,(volatile struct alt_sgdma_descriptor *) tx_next_desc);
      } else {
        unsigned int loop;
        dev->trans_start  = jiffies;

        for(loop = 0 ; loop <= tse_priv->tse_tx_sgdma_cur; loop++)
        {
          dev_kfree_skb_irq (tse_priv->tx_skb[loop]);
          tse_priv->tx_skb[loop] = NULL;
        }
        tse_priv->tse_tx_sgdma_cur = 0;
      }

   }

   return SUCCESS;
}

/*
 * Synchronous SGDMA copy from buffer memory into transmit FIFO.
 * arg1   : TSE private structure.
 * arg2   : TX descriptor pointer
 * return : 0 on success,
 *          less than 0 on errors
 */
static inline unsigned int sgdma_async_write ( struct alt_tse_private* p_tse,
                                               volatile struct alt_sgdma_descriptor *tx_desc)
{
  unsigned int timeout = 0;
  unsigned char result = 0;

  if( timeout++ == ALT_TSE_SGDMA_BUSY_WATCHDOG_CNTR ) {
    PRINTK3("TX SGDMA timeout reached\n");
    return -EBUSY;
  }

  /* Clear the status and control bits of the TX SGDMA descriptor */
  p_tse->tx_sgdma_dev->control = 0;
  p_tse->tx_sgdma_dev->status  = 0xFF;

  /* Start TX SGDMA (blocking call) */
  result = alt_sgdma_do_async_transfer ( p_tse->tx_sgdma_dev,
                                        ( volatile struct alt_sgdma_descriptor *) &tx_desc[0] );

  return result;
}


/*
 * Begin packet transmission in blocking mode
 * arg1   : Packet to be sent
 * arg2   : Network device to which packet is sent
 * return : 0 on success,
 *          less than 0 on errors
 */
static int tse_hardware_send_pkt ( struct sk_buff *skb , struct net_device *dev )
{
  struct alt_tse_private *tse_priv = ( struct alt_tse_private* ) dev->priv;
  int retval=0;
  unsigned int len ;
  volatile unsigned int *aligned_tx_buffer;

  /* Align TX buffer address to 32 bit boundary */
  aligned_tx_buffer = ( unsigned int* ) (skb->data - ALIGNED_BYTES);
  len = skb->len + ALIGNED_BYTES;

  /*Flush raw data from data cache */
  dcache_push ( (unsigned long) aligned_tx_buffer, len);

  if( tse_priv->tx_sgdma_dev->status & ALT_SGDMA_STATUS_BUSY_MSK )
  {
    PRINTK3( "Transmitting... status=%x!!!\n",tse_priv->tx_sgdma_dev->status );
    if( tse_priv->tse_tx_sgdma_cur > (ALT_TSE_TX_SGDMA_DESC_COUNT-1) ) {
      PRINTK3( "Transmitting... Try again!!!\n" );
      return SOURCE_BUSY;
    }
    tse_priv->tse_tx_sgdma_cur += 1;
  }
  else {
    tse_priv->tse_tx_sgdma_cur  = 0;
  }

  /* Prepare TX SGDMA for data transmission */
  if( len <= tse_priv->current_mtu ) {
    /* Initialize TX SG-DMA streaming interface  */
    alt_sgdma_construct_mem_to_stream_desc(
      (volatile struct alt_sgdma_descriptor *) &tse_priv->sgdma_tx_desc[tse_priv->tse_tx_sgdma_cur],  // starting TX descriptor
      (volatile struct alt_sgdma_descriptor *) &tse_priv->sgdma_tx_desc[tse_priv->tse_tx_sgdma_cur+1],  // pointer to "next" descriptor
      (unsigned int *)aligned_tx_buffer,                     // starting read address
      (len),                                                 // length of bytes
      0,                                                     // don't read from constant address
      1,                                                     // generate sop
      1,                                                     // generate endofpacket signal
      0);                                                    //

   // if(tse_priv->tx_skb[tse_priv->tse_tx_sgdma_cur] != NULL)
   //   dev_kfree_skb( tse_priv->tx_skb[tse_priv->tse_tx_sgdma_cur] );

    tse_priv->tx_skb[tse_priv->tse_tx_sgdma_cur] = skb;

    if(!(tse_priv->tx_sgdma_dev->status & ALT_SGDMA_STATUS_BUSY_MSK )) {
      /* Start TX SGDMA */
      PRINTK1("Start TX SGDMA with %d descriptor no\n",tse_priv->tse_tx_sgdma_cur);

      /* Clear the status and control bits of the TX SGDMA descriptor */
      tse_priv->tx_sgdma_dev->control = 0;
      tse_priv->tx_sgdma_dev->status  = 0xFF;

      /* Start TX SGDMA (blocking call) */
      alt_sgdma_do_async_transfer ( tse_priv->tx_sgdma_dev,
                                           ( volatile struct alt_sgdma_descriptor *)&tse_priv->sgdma_tx_desc[tse_priv->tse_tx_sgdma_cur]);
    }

  }
  else
    retval = -ENP_RESOURCE;

  /* SGDMA not available */
  if( retval < 0 ){
    PRINTK3("tse_hardware_send_pkt(): SGDMA not available\n");
    tse_priv->sem = 0;

    /* Free the original skb */
    dev_kfree_skb( skb );

    /* Inform upper layers. */
    //netif_wake_queue( dev );

    /* Operation not permitted */
    return SUCCESS;

  } else {  /* = 0, success */

    /* Inform upper layers. */
    //netif_wake_queue( dev );

    return SUCCESS;
  }

}

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * Polling receive - used by netconsole and other diagnostic tools
 * to allow network i/o with interrupts disabled.
 */
static void tse_net_poll_controller(struct net_device *dev)
{

  struct alt_tse_private *tse_priv = (struct alt_tse_private *) dev->priv;
  disable_irq(tse_priv->rx_fifo_interrupt);
  net_interrupt(tse_priv->rx_fifo_interrupt,dev,NULL);
  enable_irq(tse_priv->rx_fifo_interrupt);

}
#endif

/*
 * Get the current Ethernet statistics.This may be called with the device open or closed.
 * arg1   : net device for which multicasts filter is adjusted
 * return : network statistics structure
 */
static struct net_device_stats *tse_get_statistics ( struct net_device *dev )
{
  struct alt_tse_private *tse_priv = (struct alt_tse_private *) dev->priv;
  struct net_device_stats *net_status = (struct net_device_stats *) &tse_priv->status;

  /* total packets received without error*/
  net_status->rx_packets = tse_priv->mac_dev->aFramesReceivedOK + tse_priv->mac_dev->ifInErrors;

  /* total packets received without error*/
  net_status->tx_packets = tse_priv->mac_dev->aFramesTransmittedOK + tse_priv->mac_dev->ifOutErrors;

  /* total bytes received without error  */
  net_status->rx_bytes = tse_priv->mac_dev->aOctetsReceivedOK;

  /* total bytes transmitted without error   */
  net_status->tx_bytes = tse_priv->mac_dev->aOctetsTransmittedOK;

  /* bad received packets  */
  net_status->rx_errors =  tse_priv->mac_dev->ifInErrors;

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

static void tse_set_hash_table( struct net_device *dev, int count, struct dev_mc_list *addrs )
{
  int                   mac_octet,xor_bit,bitshift,hash,loop;
  char                  octet;
  struct   dev_mc_list  *cur_addr;
  struct alt_tse_private *tse_priv = (struct alt_tse_private *) dev->priv;
  alt_tse_mac   *p_mac_base = tse_priv->mac_dev;

  cur_addr = addrs;
  for (loop = 0; loop < count ; loop++, cur_addr = cur_addr->next ) {
    /* do we have a pointer here? */
    if ( !cur_addr )
       break;

    /* make sure this is a multicasts address    */
    if ( !( *cur_addr->dmi_addr & 1 ) )//
            continue;

    PRINTK1("dmi_addr %x-%x-%x-%x-%x-%x\n",cur_addr->dmi_addr[0],
                                           cur_addr->dmi_addr[1],
                                           cur_addr->dmi_addr[2],
                                           cur_addr->dmi_addr[3],
                                           cur_addr->dmi_addr[4],
                                           cur_addr->dmi_addr[5]);
    hash = 0; // the hash value

    for(mac_octet=5; mac_octet >= 0; mac_octet--) {
      xor_bit = 0;
      octet = cur_addr->dmi_addr[mac_octet];
      for(bitshift=0;bitshift < 8;bitshift++)
        xor_bit ^= (int)((octet >> bitshift) & 0x01);
      hash = (hash << 1) | xor_bit;
      PRINTK1("\t\thash=%d,xor_bit=%d octet=%x\n",hash,xor_bit,octet);
    }

    p_mac_base->hash_table[ hash ] = 1;
  }

}

/*
 * Set/Clear multicasts filter
 * arg1    : net device for which multicasts filter is adjusted
 *           multicasts table from the linked list of addresses
 *           associated with this dev structure.
 */
static void tse_set_multicast_list ( struct net_device *dev )
{
  struct alt_tse_private *tse_priv = ( struct alt_tse_private *) dev->priv;
  int hash_loop;

  if ( dev->flags & IFF_PROMISC ) {
    /* Log any net taps */
    PRINTK3("%s: Promiscuous mode enabled.\n", dev->name);
    tse_priv->mac_dev->command_config.image |= ALTERA_TSE_CMD_PROMIS_EN_MSK;
  } else {
    tse_priv->mac_dev->command_config.image &= ~ALTERA_TSE_CMD_PROMIS_EN_MSK;
  }

  if (dev->flags & IFF_ALLMULTI){
    for(hash_loop=0; hash_loop < 64; hash_loop++ )
      tse_priv->mac_dev->hash_table[hash_loop] = 1;
  } else {
    for(hash_loop=0; hash_loop < 64; hash_loop++ )
      tse_priv->mac_dev->hash_table[hash_loop] = 0; // Clear any existing hash entries

    if(dev->mc_count)
      tse_set_hash_table ( dev, dev->mc_count, dev->mc_list );

  }
}

/*
 * Initialize the MAC address
 * arg1    : net device for which TSE MAC driver is registered
 * arg2    : address passed from upper layer
 * return : 0
 */
int tse_set_hw_address ( struct net_device *dev , void *port )
{
  struct sockaddr *addr = port;
  struct alt_tse_private *tse_priv = (struct alt_tse_private *) dev->priv;

  memcpy(dev->dev_addr, addr->sa_data,dev->addr_len);

  /* Set the MAC address */
  tse_priv->mac_dev->mac_addr_0 = (( dev->dev_addr[2] ) << 24
                                  |( dev->dev_addr[3] ) << 16
                                  |( dev->dev_addr[4] ) <<  8
                                  |( dev->dev_addr[5] ));

  tse_priv->mac_dev->mac_addr_1 = (( dev->dev_addr[0] << 8
                                  |( dev->dev_addr[1] )) & 0xFFFF );

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

  PRINTK3("Set Mac Address %x-%x-%x-%x-%x-%x\n", dev->dev_addr[0],
                                                 dev->dev_addr[1],
                                                 dev->dev_addr[2],
                                                 dev->dev_addr[3],
                                                 dev->dev_addr[4],
                                                 dev->dev_addr[5]);

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
  struct alt_tse_private *tse_priv = (struct alt_tse_private *) dev->priv;
  unsigned int free_loop;

  if ((new_mtu > (tse_priv->tse_tx_depth * ALT_TSE_MAC_FIFO_WIDTH)) ||
     (new_mtu > (tse_priv->tse_rx_depth*ALT_TSE_MAC_FIFO_WIDTH))) {
    printk("Your system doesn't support new MTU size as TX/RX FIFO size is small\n" );
    return -EINVAL;
  }

  if (new_mtu < TSE_MIN_MTU || new_mtu > TSE_MAX_MTU)
    return -EINVAL;

  spin_lock(&tse_priv->rx_lock);

  tse_priv->rx_sgdma_dev->control = ALT_SGDMA_CONTROL_SOFTWARERESET_MSK;
  tse_priv->rx_sgdma_dev->control = 0x0;

  tse_priv->current_mtu = new_mtu ;
  dev->mtu = new_mtu;
  tse_priv->mac_dev->max_frame_length = tse_priv->current_mtu;

  /* Disable receiver and transmitter  descriptor(SGDAM) */
  for(free_loop=0; free_loop < ALT_TSE_RX_SGDMA_DESC_COUNT;free_loop++) {
    /* Free the original skb */
    if( tse_priv->rx_skb[free_loop] != NULL )
      dev_kfree_skb( tse_priv->rx_skb[free_loop] );
  }

  /* Prepare RX SGDMA to receive packets */
  sgdma_read_init( dev );

  PRINTK3("TSE: new mtu is %d \n",new_mtu);

  spin_unlock(&tse_priv->rx_lock);

  return 0;
}

/*
 * Open and Initialize the interface
 * The interface is opened whenever 'ifconfig' activates it
 *  arg1   : 'net_device' structure pointer
 *  return : 0
 */
static int tse_open(struct net_device *dev)
{
  struct alt_tse_private *tse_priv = ( struct alt_tse_private *)dev->priv;
  int status;
  int retval = 0;

  /* Reset and configure TSE MAC and probe associated PHY */
  tse_mac_phy_config( tse_priv );

  /* Initialize SGDMA */
  sgdma_config( tse_priv );

  /* Prepare RX SGDMA to receive packets */
  status = sgdma_read_init( dev );
  /* Start network queue */
  netif_start_queue( dev );

  /* Register RX SGDMA interrupt */
  retval = request_irq ( tse_priv->rx_fifo_interrupt,(void *)alt_sgdma_rx_isr, 0,
                         "SGDMA_RX", dev );
  if ( retval ) {
    PRINTK3("%s:Unable to register Rx SGDMA interrupt %d (retval=%d).\n",
             dev->name,tse_priv->rx_fifo_interrupt, retval);
    return -EAGAIN;
  }
  /* Register TX SGDMA interrupt */
  retval = request_irq ( tse_priv->tx_fifo_interrupt,(void *)alt_sgdma_tx_isr, 0,"SGDMA_TX", dev );
  if ( retval ) {
    PRINTK3("%s:Unable to register RX SGDMA interrupt %d (retval=%d).\n",
             dev->name,tse_priv->tx_fifo_interrupt, retval);
    return -EAGAIN;
  }
  /* Register TX SGDMA interrupt */
  retval = request_irq ( tse_priv->alarm_irq,(void *)alt_tse_link_isr, 0,"TSE_ALARM_LINK", dev );
  if ( retval ) {
      PRINTK3("%s:Unable to register timer interrupt %d (retval=%d).\n",
               "TSE_ALARM_LINK",tse_priv->alarm_irq, retval);
      return -EAGAIN;
  }
  #ifdef CONFIG_PHY_IRQ_PRESENCE
    //ToDo for Phy irq enable
    //Left due to lack of Marvell PHY (3c120) information
  #else
      tse_priv->alarm_link_check->periodl   =  ( ALT_ALARM_TIMEOUT_THRESHOLD ) & 0xFFFF;    // lower timer
      tse_priv->alarm_link_check->periodh   =  ( ALT_ALARM_TIMEOUT_THRESHOLD>>16 ) & 0xFFFF;// higher timer
      tse_priv->alarm_link_check->control   =  ( TIMER_CONTROL_ITO_MSK | TIMER_CONTROL_START_MSK  );
  #endif
  //tasklet_init(&tse_priv->tse_rx_tasklet, tse_sgdma_rx, (unsigned long)dev);
  return SUCCESS;
}

/*
 *  Stop TSE MAC interface - this puts the device in an inactive state
 *  arg1   : 'net_device' structure pointer
 *  return : 0
 *
 */

static int tse_shutdown(struct net_device *dev)
{
  struct alt_tse_private *tse_priv = ( struct alt_tse_private *) dev->priv;
  unsigned int free_loop;
  netif_stop_queue ( dev );

  //tasklet_kill(&tse_priv->tse_rx_tasklet);

  /* Free interrupt handler */
  free_irq ( tse_priv->rx_fifo_interrupt, ( void *) dev );
  free_irq ( tse_priv->tx_fifo_interrupt, ( void *) dev );
  free_irq ( tse_priv->alarm_irq, ( void *) dev );

  spin_lock(&tse_priv->rx_lock);

  /* Disable receiver and transmitter  descriptor(SGDAM) */
  for(free_loop=0; free_loop < ALT_TSE_TX_SGDMA_DESC_COUNT;free_loop++) {
    /* Free the original skb */
    if( tse_priv->tx_skb[free_loop] != NULL ) {
      dev_kfree_skb( tse_priv->tx_skb[free_loop] );
      tse_priv->tx_skb[free_loop] = NULL;
    }
  }

  /* Disable receiver and transmitter  descriptor(SGDAM) */
  for(free_loop=0; free_loop < ALT_TSE_RX_SGDMA_DESC_COUNT;free_loop++) {
    /* Free the original skb */
    if( tse_priv->rx_skb[free_loop] != NULL ) {
      dev_kfree_skb( tse_priv->rx_skb[free_loop] );
      tse_priv->rx_skb[free_loop] = NULL;
    }
  }

  spin_lock(&tse_priv->rx_lock);

  return SUCCESS;
}


/* Ethtool support... */

static void tse_get_drvinfo ( struct net_device *dev, struct ethtool_drvinfo *info )
{
  strcpy(info->driver, "tse");
  strcpy(info->version, "v 8.0");
  sprintf(info->bus_info, "AVALON");
  PRINTK3("Drvinfo :Drv=%s ,version=%s,bus=%s\n",info->driver,info->version,info->bus_info);
}

static unsigned int tse_get_link ( struct net_device *dev )
{
  struct alt_tse_private *tse_priv = ( struct alt_tse_private *) dev->priv;

  /* Record previous MDIO address, to be restored at the end of function */
  unsigned int temp_mdio_phy1_addr = tse_priv->mac_dev->mdio_phy1_addr;
  unsigned int link;

  /* write PHY address to MDIO to access the i-th PHY */
  tse_priv->mac_dev->mdio_phy1_addr = tse_priv->phy_address;

  link=phy_mdio_reg_read(tse_priv,tse_priv->pphy_profile->status_reg_location,
                              tse_priv->pphy_profile->link_bit_location,1);

  PRINTK1("Link =%d\n",link);

  tse_priv->mac_dev->mdio_phy1_addr=temp_mdio_phy1_addr;
  return link;
}

static int tse_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
  struct alt_tse_private *tse_priv = ( struct alt_tse_private *) dev->priv;

  /* Record previous MDIO address, to be restored at the end of function */
  unsigned int temp_mdio_phy1_addr = tse_priv->mac_dev->mdio_phy1_addr;
  struct alt_tse_phy_profile     *pphy_profile = tse_priv->pphy_profile;

  /* write PHY address to MDIO to access the i-th PHY */
  tse_priv->mac_dev->mdio_phy1_addr = tse_priv->phy_address;

  if(pphy_profile->oui==MV88E1111_OUI) {
    if(phy_mdio_reg_read(tse_priv,TSE_PHY_MDIO_EXT_STATUS, TSE_PHY_MDIO_EXT_STATUS_1000BASE_T_FULL, 1))
       cmd->supported |= SUPPORTED_1000baseT_Full;

    if(phy_mdio_reg_read(tse_priv,TSE_PHY_MDIO_EXT_STATUS, TSE_PHY_MDIO_EXT_STATUS_1000BASE_T_HALF, 1))
       cmd->supported |= SUPPORTED_1000baseT_Half;
  }

  if(phy_mdio_reg_read(tse_priv, TSE_PHY_MDIO_STATUS, TSE_PHY_MDIO_STATUS_100BASE_X_HALF, 1))
     cmd->supported |=SUPPORTED_100baseT_Half;

  if(phy_mdio_reg_read(tse_priv, TSE_PHY_MDIO_STATUS, TSE_PHY_MDIO_STATUS_100BASE_X_FULL, 1))
     cmd->supported |=SUPPORTED_100baseT_Full;

  if(phy_mdio_reg_read(tse_priv, TSE_PHY_MDIO_STATUS, TSE_PHY_MDIO_STATUS_10BASE_T_FULL, 1))
     cmd->supported |=SUPPORTED_10baseT_Full;

  if(phy_mdio_reg_read(tse_priv, TSE_PHY_MDIO_STATUS, TSE_PHY_MDIO_STATUS_100BASE_X_HALF, 1))
     cmd->supported |=SUPPORTED_10baseT_Half;

  if(phy_mdio_reg_read(tse_priv, TSE_PHY_MDIO_STATUS, TSE_PHY_MDIO_STATUS_AN_ABILITY, 1))
     cmd->supported |=SUPPORTED_Autoneg;

  cmd->supported |=SUPPORTED_MII;

  if(pphy_profile->oui==MV88E1111_OUI) {
    if(phy_mdio_reg_read(tse_priv, TSE_PHY_MDIO_1000BASE_T_STATUS, TSE_PHY_MDIO_1000BASE_T_STATUS_LP_FULL_ADV, 1))
       cmd->advertising |=ADVERTISED_1000baseT_Full;

    if(phy_mdio_reg_read(tse_priv, TSE_PHY_MDIO_1000BASE_T_STATUS, TSE_PHY_MDIO_1000BASE_T_STATUS_LP_HALF_ADV, 1))
       cmd->advertising |=ADVERTISED_1000baseT_Half;
  }

  if(phy_mdio_reg_read(tse_priv, TSE_PHY_MDIO_REMADV, TSE_PHY_MDIO_ADV_100BASE_TX_FULL, 1))
     cmd->advertising  |=ADVERTISED_100baseT_Full;

  if(phy_mdio_reg_read(tse_priv, TSE_PHY_MDIO_REMADV, TSE_PHY_MDIO_ADV_100BASE_TX_HALF, 1))
     cmd->advertising |=ADVERTISED_100baseT_Half;

  if(phy_mdio_reg_read(tse_priv, TSE_PHY_MDIO_REMADV, TSE_PHY_MDIO_ADV_10BASE_TX_FULL, 1))
     cmd->advertising |=ADVERTISED_10baseT_Full;

  if(phy_mdio_reg_read(tse_priv, TSE_PHY_MDIO_REMADV, TSE_PHY_MDIO_ADV_10BASE_TX_HALF, 1))
     cmd->advertising |=ADVERTISED_10baseT_Half;

  cmd->advertising |=ADVERTISED_Autoneg  | ADVERTISED_MII;

  cmd->port = PORT_MII;
  cmd->transceiver = XCVR_INTERNAL;

  if ( tse_priv->link_status ) {
    cmd->speed = tse_priv->speed;
    cmd->duplex = tse_priv->duplex;
  } else {
    cmd->speed = -1;
    cmd->duplex = -1;
  }

  if ( tse_priv->antoneg_enable )
    cmd->autoneg = AUTONEG_ENABLE;
  else
    cmd->autoneg = AUTONEG_DISABLE;

  cmd->phy_address = tse_priv->phy_address;

  tse_priv->mac_dev->mdio_phy1_addr=temp_mdio_phy1_addr;
  return 0;
}

static int tse_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
  struct alt_tse_private *tse_priv = ( struct alt_tse_private *) dev->priv;
  struct alt_tse_phy_profile     *pphy_profile = tse_priv->pphy_profile;

/*
  netif_carrier_off(dev);
*/
  if (cmd->autoneg == AUTONEG_ENABLE){
    //if (tse_priv->antoneg_enable)
    //  return 0;
    //else
    {
      tse_priv->antoneg_enable = 1;

      phy_restart_auto_negotiate ( tse_priv ) ;

      tse_phy_check_media ( tse_priv );

      tse_set_mac_speed_duplex ( tse_priv );

      return 0;
    }
  } else {

    tse_priv->antoneg_enable = 0;

    switch(cmd->speed + cmd->duplex) {

    case SPEED_10 + DUPLEX_HALF:
     tse_priv->speed= 10;
     tse_priv->duplex = 0;
     break;

    case SPEED_10 + DUPLEX_FULL:
     tse_priv->speed = 10;
     tse_priv->duplex = 1;
     break;

    case SPEED_100 + DUPLEX_HALF:
     tse_priv->speed = 100;
     tse_priv->duplex = 0;
     break;

    case SPEED_100 + DUPLEX_FULL:
     tse_priv->speed = 100;
     tse_priv->duplex = 1;
     break;

    case SPEED_1000 + DUPLEX_HALF:
     if(pphy_profile->oui==DP83848C_OUI)/* not supported */
        return -EINVAL;
     tse_priv->speed = 1000;
     tse_priv->duplex = 0;
     break;

    case SPEED_1000 + DUPLEX_FULL:
     if(pphy_profile->oui==DP83848C_OUI)/* not supported */
        return -EINVAL;
     tse_priv->speed = 1000;
     tse_priv->duplex = 1;
     break;

    default:
     return -EINVAL;
    }
    disable_irq(tse_priv->alarm_irq);
    PRINTK1("INFO : New speed =%d and duplex =%d\n",tse_priv->speed,tse_priv->duplex);
    tse_phy_set_speed_duplex_antoneg(tse_priv);
    tse_set_mac_speed_duplex(tse_priv);
    enable_irq(tse_priv->alarm_irq);
  }
  return 0;
}

static const struct ethtool_ops tse_ethtool_ops = {
  .get_drvinfo    = tse_get_drvinfo,
  .get_link       = tse_get_link,
  .get_settings   = tse_get_settings,
  .set_settings   = tse_set_settings,
};

/*
 * Initialize 'net_device' structure, resets and re-configures MAC and PHY
 * arg1   : 'net_device' structure pointer allocated for TSE interface
 * arg2   : Interface number
 * return : 0 on success,
 *          less than 0 on errors
 */
static int tse_dev_probe(struct net_device *dev, unsigned int iface)
{
  struct alt_tse_private     *tse_priv = ( struct alt_tse_private *) dev->priv;

  if ( tse_system_array[iface].tse_mac_base == 0 )
    return -ENOMEM;

  /* clears TSE private structure */
  memset(tse_priv,0,sizeof(struct alt_tse_private));
  tse_priv->dev       = dev;


  /* Get TSE MAC base address */
  /* ioremap() requires buffer_size as the 2nd argument, but it is not being used inside anyway, so put 0xFFFF */

  tse_priv->mac_dev   = ( alt_tse_mac *) ioremap_nocache((unsigned long)tse_system_array[iface].tse_mac_base, sizeof(alt_tse_mac));//TSE_MAC_BASE;
  dev->base_addr      = ( unsigned long )tse_priv->mac_dev ;//TSE_MAC_BASE;

  /* Get RX and TX SGDMA addresses */
  tse_priv->rx_sgdma_dev  = ( volatile struct alt_sgdma_registers *) ioremap_nocache((unsigned long)
                                    tse_system_array[iface].sgdma_rx_base,sizeof(volatile struct alt_sgdma_registers));//RX_SGDMA_BASE;//
  tse_priv->tx_sgdma_dev  = ( volatile struct alt_sgdma_registers *) ioremap_nocache((unsigned long)
                                    tse_system_array[iface].sgdma_tx_base,sizeof(volatile struct alt_sgdma_registers));//TX_SGDMA_BASE;//

  /*Get descriptor memory pointer address*/
  #ifdef CONFIG_DECS_MEMORY_SELECT
    tse_priv->desc_mem_base = ioremap_nocache((unsigned long)tse_system_array[iface].desc_mem_base,
                              sizeof(volatile struct alt_sgdma_descriptor));//(DECS_MEMORY_BASE | 0x80000000) ;//
  #else
    tse_system_array[iface].desc_mem_base = (unsigned int) kmalloc(ALT_TSE_TOTAL_SGDMA_DESC_SIZE, GFP_KERNEL);
    if (!tse_system_array[iface].desc_mem_base)
        return -ENOMEM;
    tse_priv->desc_mem_base = ioremap_nocache((unsigned long)tse_system_array[iface].desc_mem_base,
                              sizeof(volatile struct alt_sgdma_descriptor));//(DECS_MEMORY_BASE | 0x80000000) ;//
  #endif

  /* Set initial SGDMA descriptor address */
  tse_priv->desc = (volatile struct alt_sgdma_descriptor *) tse_priv->desc_mem_base;

  tse_priv->desc = (volatile struct alt_sgdma_descriptor *)tse_priv->desc_mem_base;
  tse_priv->sgdma_tx_desc =(volatile struct alt_sgdma_descriptor *)tse_priv->desc;
  tse_priv->sgdma_rx_desc =(volatile struct alt_sgdma_descriptor *)&tse_priv->desc[ALT_TSE_TX_SGDMA_DESC_COUNT];

  tse_priv->sem = 0;
  tse_priv->antoneg_enable=AUTONEG_ENABLE;

  /* set MTU as max frame size */
  tse_priv->current_mtu = ALT_TSE_MAX_FRAME_LENGTH;

  /* Set TSE MAC RX and TX fifo depth */
  tse_priv->tse_tx_depth = tse_system_array[iface].tse_tx_depth;//TX_MAC_FIFO;//
  tse_priv->tse_rx_depth = tse_system_array[iface].tse_rx_depth;//RX_MAC_FIFO;//
  tse_priv->rx_fifo_interrupt = tse_system_array[iface].tse_sgdma_rx_irq;//RX_SGDMA_IRQ;
  tse_priv->tx_fifo_interrupt = tse_system_array[iface].tse_sgdma_tx_irq;//TX_SGDMA_IRQ;

  tse_priv->alarm_link_check =( struct timer_reg *)ioremap_nocache((unsigned long)
                              tse_system_array[iface].alarm_link_base,sizeof(struct timer_reg));//TX_SGDMA_IRQ;
  tse_priv->alarm_irq = tse_system_array[iface].alarm_link_irq;//TX_SGDMA_IRQ;

  /* Set default MAC address */
  dev->dev_addr[0] = tse_system_array[iface].mac_add[0];//MAC_0_OCTET;
  dev->dev_addr[1] = tse_system_array[iface].mac_add[1];//MAC_1_OCTET;
  dev->dev_addr[2] = tse_system_array[iface].mac_add[2];//MAC_2_OCTET;
  dev->dev_addr[3] = tse_system_array[iface].mac_add[3];//MAC_3_OCTET;
  dev->dev_addr[4] = tse_system_array[iface].mac_add[4];//MAC_4_OCTET;
  dev->dev_addr[5] = tse_system_array[iface].mac_add[5];//MAC_5_OCTET;

  /* Fill in the fields of the device structure with Ethernet values */
  ether_setup ( dev );

  /* The Open Ethernet specific entries in the device structure */
  dev->open              = tse_open ;
  dev->hard_start_xmit   = tse_hardware_send_pkt ;
  dev->stop              = tse_shutdown;
  dev->set_mac_address   = tse_set_hw_address;
  dev->get_stats         = tse_get_statistics;
  dev->set_multicast_list= tse_set_multicast_list;
  dev->change_mtu        = tse_change_mtu;
  dev->ethtool_ops       = &tse_ethtool_ops;

  #ifdef CONFIG_NET_POLL_CONTROLLER
    dev->poll_controller  = tse_net_poll_controller;
  #endif

  /* Spin lock variable initialize */
  spin_lock_init(&tse_priv->rx_lock);
  spin_lock_init(&tse_priv->tx_lock);

  return SUCCESS;
}


static int alt_tse_remove(struct platform_device *pdev)
{
  struct net_device *ndev = platform_get_drvdata(pdev);

  platform_set_drvdata(pdev, NULL);
  unregister_netdev(ndev);
  free_netdev(ndev);

  return 0;
}


/*
 * Driver entry point called by Platform devise driver
 * arg    : platform_device object
 * return : 'net_device' structure pointer on success else 0
 */
static int alt_tse_probe(struct platform_device *pdev)
{

  struct net_device *dev;
  static unsigned int unit;
  int    err = -ENODEV,ret;
  struct resource *res_tse_mac;
  struct resource *res_sgdma_rx;
  struct resource *res_sgdma_tx;

  dev = alloc_etherdev ( sizeof ( struct alt_tse_private ) );
  if ( !dev ) {
      printk(KERN_ERR"eth%d: Etherdev alloc failed, aborting.\n", unit);
      return ERR_PTR( -ENODEV );
  }
  if ( unit >= 0 ) {
    sprintf( dev->name, "eth%d", unit );
    netdev_boot_setup_check( dev );
    dev->priv =(void *) ioremap_nocache((unsigned long)dev->priv,
                        sizeof ( struct alt_tse_private ));//((int)dev->priv | 0x80000000);
  }

  /* 1. Get tse MAC resource */
  res_tse_mac = platform_get_resource_byname(pdev, IORESOURCE_MEM, TSE_RESOURCE_MAC_DEV);
  if (!res_tse_mac) {
    printk("ERROR :%s:%d:platform_get_resource_byname() failed\n", __FILE__, __LINE__);
    ret = -ENODEV;
    goto out;
  }

  if (!request_mem_region(res_tse_mac->start, res_tse_mac->end - res_tse_mac->start + 1, "altera_tse")) {
    printk("ERROR :%s:%d:request_mem_region() failed\n", __FILE__, __LINE__);
    ret = -EBUSY;
    goto out;
  }

  /* 2. Get sgdma_rx mem resource */
  res_sgdma_rx = platform_get_resource_byname(pdev, IORESOURCE_MEM, TSE_RESOURCE_SGDMA_RX_DEV);
  if (!res_sgdma_rx) {
    printk("ERROR :%s:%d:platform_get_resource_byname() failed\n", __FILE__, __LINE__);
    ret = -ENODEV;
    goto out;
  }

  if (!request_mem_region(res_sgdma_rx->start, res_sgdma_rx->end - res_sgdma_rx->start + 1, "altera_tse")) {
    printk("ERROR :%s:%d:request_mem_region() failed\n", __FILE__, __LINE__);
    ret = -EBUSY;
    goto out;
  }

  /* 3. Get sgdma_tx mem resource */
  res_sgdma_tx = platform_get_resource_byname(pdev, IORESOURCE_MEM, TSE_RESOURCE_SGDMA_TX_DEV);
  if (!res_sgdma_tx) {
    printk("ERROR :%s:%d:platform_get_resource_byname() failed\n", __FILE__, __LINE__);
    ret = -ENODEV;
    goto out;
  }

  if (!request_mem_region(res_sgdma_tx->start, res_sgdma_tx->end - res_sgdma_tx->start + 1, "altera_tse")) {
    printk("ERROR :%s:%d:request_mem_region() failed\n", __FILE__, __LINE__);
    ret = -EBUSY;
    goto out;
  }

  /* Probe ethernet device */
  if( tse_dev_probe ( dev, unit ) <0 )
  {
    printk("%s:Failed to initialize ethernet device\n", dev->name);
    err = -ENODEV;
    goto out;
  }
  /* Register ethernet device driver */
  err = register_netdev(dev);
  if (!err)
    printk("%s:Successed to register TSE net device\n", dev->name);
  else
  {
    printk(KERN_ERR"%s:Failed to register TSE net device\n", dev->name);
    goto out;
  }

  SET_NETDEV_DEV(dev, &pdev->dev);
  platform_set_drvdata(pdev, dev);

  return 0;
out:
   free_netdev ( dev );

  return ERR_PTR ( err );
}

static struct platform_driver alt_tse_driver = {
  .driver = {
    .name    = "altera_tse",
    .owner   = THIS_MODULE,
  },
  .probe   = alt_tse_probe,
  .remove  = alt_tse_remove,
  .suspend = NULL,
  .resume  = NULL,
};

static int __init atse_init(void)
{
  /* probe board and register */
  return platform_driver_register(&alt_tse_driver);
}

static void __exit atse_exit(void)
{
  /* platform_driver_unregister(&atse_driver); */
  printk("ALT:%s:%d:%s \n", __FILE__, __LINE__, __FUNCTION__);
}

module_init(atse_init);
module_exit(atse_exit);

MODULE_AUTHOR("Altera");
MODULE_DESCRIPTION("Altera Triple Speed MAC IP");
MODULE_LICENSE("GPL");
