/*
 * Based on MIPS support for CONFIG_OF device tree support
 *
 * Copyright (C) 2010 Cisco Systems Inc. <dediao@cisco.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/bootmem.h>
#include <linux/initrd.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#include <asm/page.h>
#include <asm/prom.h>

static void *dtb_passed; /* need to reserve bootmem */

int __init early_init_dt_scan_memory_arch(unsigned long node,
					  const char *uname, int depth,
					  void *data)
{
	return early_init_dt_scan_memory(node, uname, depth, data);
}

void __init early_init_dt_add_memory_arch(u64 base, u64 size)
{
	return;
}

void * __init early_init_dt_alloc_memory_arch(u64 size, u64 align)
{
	return __alloc_bootmem(size, align, __pa(MAX_DMA_ADDRESS));
}

#ifdef CONFIG_BLK_DEV_INITRD
void __init early_init_dt_setup_initrd_arch(unsigned long start,
					    unsigned long end)
{
	initrd_start = (unsigned long)__va(start);
	initrd_end = (unsigned long)__va(end);
	initrd_below_start_ok = 1;
}
#endif

/*
 * irq_create_of_mapping - Hook to resolve OF irq specifier into a Linux irq#
 *
 * Currently the mapping mechanism is trivial; simple flat hwirq numbers are
 * mapped 1:1 onto Linux irq numbers.  Cascaded irq controllers are not
 * supported.
 */
unsigned int irq_create_of_mapping(struct device_node *controller,
				   const u32 *intspec, unsigned int intsize)
{
	return intspec[0];
}
EXPORT_SYMBOL_GPL(irq_create_of_mapping);

void __init early_init_devtree(void *params)
{
	extern int __dtb_start;

	if (params && be32_to_cpup((__be32 *)params) == OF_DT_HEADER)
		initial_boot_params = dtb_passed = params;
#if defined(CONFIG_NIOS2_DTB_AT_PHYS_ADDR) && CONFIG_NIOS2_DTB_AT_PHYS_ADDR
	else if (be32_to_cpup((__be32 *)CONFIG_NIOS2_DTB_AT_PHYS_ADDR) ==
		 OF_DT_HEADER)
		initial_boot_params = (void *)CONFIG_NIOS2_DTB_AT_PHYS_ADDR;
#endif
	else if (be32_to_cpu((__be32)__dtb_start) == OF_DT_HEADER)
		initial_boot_params = (void *)&__dtb_start;
	else
		return;

	/* Retrieve various informations from the /chosen node of the
	 * device-tree, including the platform type, initrd location and
	 * size, and more ...
	 */
	of_scan_flat_dt(early_init_dt_scan_chosen, NULL);

	/* Scan memory nodes */
	of_scan_flat_dt(early_init_dt_scan_root, NULL);
	of_scan_flat_dt(early_init_dt_scan_memory_arch, NULL);
}

void __init device_tree_init(void)
{
	if (!initial_boot_params)
		return;

	if (dtb_passed) {
		unsigned long base, size;

		base = virt_to_phys((void *)initial_boot_params);
		size = be32_to_cpu(initial_boot_params->totalsize);
		reserve_bootmem(base, size, BOOTMEM_DEFAULT);
		unflatten_device_tree();
		free_bootmem(base, size);
	} else
		unflatten_device_tree();
}

#ifdef CONFIG_EARLY_PRINTK
static int __init early_init_dt_scan_serial(unsigned long node,
			const char *uname, int depth, void *data)
{
	u32 *addr;
	u64 *addr64 = (u64*)data;

	/* find the first serial node */
	if (strncmp(uname, "serial", 6) != 0)
		return 0;
	/* Check for compatible */
	addr = of_get_flat_dt_prop(node, "reg", NULL);
	if(!addr)
		return 0;

	*addr64 = (u64)be32_to_cpup(addr);
	return early_init_dt_translate_address(node,addr64);
}


int __init early_altera_uart_or_juart_console(void)
{
	u64 base = 0;

	if(of_scan_flat_dt(early_init_dt_scan_serial, &base))
		return (int)(base) + IO_REGION_BASE;
	else
		return 0;
}
#endif /* CONFIG_EARLY_PRINTK */
