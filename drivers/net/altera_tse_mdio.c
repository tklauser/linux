
/*
*  linux/drivers/net/altera_tse_mdio.c
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
*/
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/phy.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include "altera_tse.h"

static int tse_mdio_reset(struct mii_bus *bus)
{
	/* No TSE MDIO Reset */
	return 0;
}

static int tse_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	alt_tse_mac *mac_dev;
	unsigned int *mdio_regs;
	unsigned int data;
	u16 value;

	mac_dev = (alt_tse_mac *) bus->priv;

	/* set mdio address */
	mac_dev->mdio_phy0_addr = mii_id;
	mdio_regs = (unsigned int *) &mac_dev->mdio_phy0;

	/* get the data */
	data = mdio_regs[regnum];

	value = data & 0xffff;

	return value;
}

static int tse_mdio_write(struct mii_bus *bus, int mii_id, int regnum, u16 value)
{
	alt_tse_mac *mac_dev;
	unsigned int *mdio_regs;
	unsigned int data;

	mac_dev = (alt_tse_mac *) bus->priv;

	/* set mdio address */
	mac_dev->mdio_phy0_addr = mii_id;
	mdio_regs = (unsigned int *) &mac_dev->mdio_phy0;

	/* get the data */
	data = (unsigned int) value;

	mdio_regs[regnum] = data;

	return 0;
}

static int tse_mdio_probe(struct platform_device *pdev)
{
	struct mii_bus *new_bus;
	struct resource *res_alt_tse;
	int ret = -ENODEV;
	int i;
	alt_tse_mac *mac_dev;
	struct alt_tse_mdio_private *tse_mdio_priv;

	if (NULL == pdev)
		return -EINVAL;

	new_bus = mdiobus_alloc();

	if (NULL == new_bus)
		return -ENOMEM;

	tse_mdio_priv = (struct alt_tse_mdio_private *) pdev->dev.platform_data;

	new_bus->name = "Altera TSE MII Bus",
	new_bus->read = &tse_mdio_read,
	new_bus->write = &tse_mdio_write,
	new_bus->reset = &tse_mdio_reset,
	snprintf(new_bus->id, MII_BUS_ID_SIZE, "%x", pdev->id);

	printk(KERN_INFO "%s : New Bus ID\n",new_bus->id);

	res_alt_tse =
		platform_get_resource_byname(pdev, IORESOURCE_MEM,
					 TSE_RESOURCE_MAC_DEV);
	if (!res_alt_tse) {
		printk("ERROR :%s:%d:platform_get_resource_byname() failed\n",
		       __FILE__, __LINE__);
		ret = -ENODEV;
		goto out;
	}

	mac_dev =
		(alt_tse_mac *) ioremap_nocache((unsigned long)
			res_alt_tse->start, sizeof(alt_tse_mac));

	new_bus->priv = (void *) mac_dev;

	new_bus->irq = tse_mdio_priv->irq;

//	new_bus->dev =  dev;
//	dev_set_drvdata(dev, new_bus);
	platform_set_drvdata(pdev, new_bus);

	ret = mdiobus_register(new_bus);

	/* probe bus, report phys */
	for (i = 31; i >= 0; i--) {
		u32 phy_id;
		u32 phy_id_bottom;
		u32 phy_id_top;
		int r;

		r = get_phy_id(new_bus, i, &phy_id);
		phy_id_top = (phy_id>>16) & (0xffff);
		phy_id_bottom = (phy_id) & (0xffff);
		if (r)
			return r;

		if (phy_id_top != phy_id_bottom)
			printk(KERN_INFO "Found phy with ID=0x%x at address=0x%x\n",
				phy_id, i);
	}

	if (0 != ret) {
		printk (KERN_ERR "%s: Cannot register as MDIO bus\n",
				new_bus->name);
		goto out;
	}

	printk(KERN_INFO "%s: MDIO Bus Registered\n", new_bus->name);

	return 0;

out:
	return ret;
}

static int tse_mdio_remove(struct platform_device *pdev)
{
	struct mii_bus *bus = platform_get_drvdata(pdev);

	mdiobus_unregister(bus);

	platform_set_drvdata(pdev, NULL);

	iounmap(bus->priv);
	bus->priv = NULL;
	kfree(bus);

	return 0;
}

static struct platform_driver alt_tse_mdio_driver = {
	.driver =	{
			.name = ALT_TSE_MDIO_NAME,
			.owner = THIS_MODULE,
			},
	.probe = tse_mdio_probe,
	.remove = tse_mdio_remove,
	.suspend = NULL,
	.resume = NULL,
};

static int __init tse_mdio_init(void)
{
	return platform_driver_register(&alt_tse_mdio_driver);
}

static void __exit tse_mdio_exit(void)
{
	platform_driver_unregister(&alt_tse_mdio_driver);
}

module_init(tse_mdio_init);
module_exit(tse_mdio_exit);

MODULE_AUTHOR("Altera");
MODULE_DESCRIPTION("Altera Triple Speed MAC IP MDIO Driver");
MODULE_LICENSE("GPL");
