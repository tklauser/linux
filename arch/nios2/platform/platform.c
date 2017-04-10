/*
 * Copyright (C) 2017 Tobias Klauser
 * Copyright (C) 2013 Altera Corporation
 * Copyright (C) 2011 Thomas Chou
 * Copyright (C) 2011 Walter Goossens
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file COPYING in the main directory of this
 * archive for more details.
 */

#include <linux/init.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>
#include <linux/time.h>
#include <linux/io.h>

/* System ID Registers */
#define SYSID_REG_ID		0x0
#define SYSID_REG_TIMESTAMP	0x4

struct altera_sysid {
	const char *tstamp;
};

static ssize_t sysid_read_timestamp(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct altera_sysid *sysid = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", sysid->tstamp);
}

static const struct device_attribute sysid_timestamp_attr =
	__ATTR(timestamp, 0444, sysid_read_timestamp, NULL);

static const char * __init sysid_get_id(void __iomem *base)
{
	return kasprintf(GFP_KERNEL, "%#x", ioread32(base + SYSID_REG_ID));
}

static const char * __init sysid_get_timestamp(void __iomem *base)
{
	u32 val;
	struct tm tstamp;

	val = ioread32(base + SYSID_REG_TIMESTAMP);
	time_to_tm(val, 0, &tstamp);

	return kasprintf(GFP_KERNEL, "%04u-%02u-%02uT%02u:%02u:%02u+00:00",
			 (unsigned int)(tstamp.tm_year + 1900),
			 tstamp.tm_mon + 1, tstamp.tm_mday, tstamp.tm_hour,
			 tstamp.tm_min, tstamp.tm_sec);
}

static int __init nios2_soc_device_init(void)
{
	struct soc_device *soc_dev;
	struct soc_device_attribute *soc_dev_attr;
	const char *machine;
	struct device_node *np;
	struct altera_sysid *sysid = NULL;

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return 0;

	machine = of_flat_dt_get_machine_name();
	if (machine)
		soc_dev_attr->machine = kstrdup_const(machine, GFP_KERNEL);

	soc_dev_attr->family = "Nios II";

	np = of_find_compatible_node(NULL, NULL, "altr,sysid-1.0");
	if (np) {
		void __iomem *sysid_base = of_iomap(np, 0);

		if (sysid_base) {
			soc_dev_attr->soc_id = sysid_get_id(sysid_base);

			sysid = kmalloc(sizeof(*sysid), GFP_KERNEL);
			if (sysid)
				sysid->tstamp = sysid_get_timestamp(sysid_base);

			iounmap(sysid_base);

			pr_info("Altera System ID: %s %s\n",
				soc_dev_attr->soc_id,
				sysid && sysid->tstamp ? sysid->tstamp : "");
		}
	}

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev)) {
		if (sysid) {
			kfree(sysid->tstamp);
			kfree(sysid);
		}
		kfree(soc_dev_attr->soc_id);
		kfree(soc_dev_attr->machine);
		kfree(soc_dev_attr);
	} else {
		dev_set_drvdata(soc_device_to_device(soc_dev), sysid);
		device_create_file(soc_device_to_device(soc_dev),
				   &sysid_timestamp_attr);
	}

	return 0;
}

device_initcall(nios2_soc_device_init);
