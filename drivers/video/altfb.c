/*
 *  altfb.c -- Altera framebuffer driver
 * 
 *  Based on vfb.c -- Virtual frame buffer device
 *
 *	Copyright (C) 1997 Geert Uytterhoeven
 *      Copyright (C) 2002 James Simmons
 *	Copyright (C) 2006-2008 Thomas Chou
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>
#include <linux/fb.h>
#include <linux/init.h>

#if defined(CONFIG_ALTERA_NEEK_C3)
#define SGDMABASE na_lcd_sgdma	/* Altera Video Sync Generator */
#define XRES 800
#define YRES 480
#define BPX  32
#else
#define VGABASE na_vga_controller_0	/* Altera VGA controller */
#define XRES 640
#define YRES 480
#define BPX  16
#endif

/*
 *  RAM we reserve for the frame buffer. This defines the maximum screen
 *  size
 *
 *  The default can be overridden if the driver is compiled as a module
 */

#define VIDEOMEMSIZE	(XRES * YRES * (BPX>>3))

static struct fb_var_screeninfo altfb_default __initdata = {
	.xres = XRES,
	.yres = YRES,
	.xres_virtual = XRES,
	.yres_virtual = YRES,
	.bits_per_pixel = BPX,
#if (BPX == 16)
	.red = {11, 5, 0},
	.green = {5, 6, 0},
	.blue = {0, 5, 0},
#else /* BXP == 16 or BXP == 32 */
	.red = {16, 8, 0},
	.green = {8, 8, 0},
	.blue = {0, 8, 0},
#endif
	.activate = FB_ACTIVATE_NOW,
	.height = -1,
	.width = -1,
	.vmode = FB_VMODE_NONINTERLACED,
};

static struct fb_fix_screeninfo altfb_fix __initdata = {
	.id = "Altera FB",
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.line_length = (XRES * (BPX >> 3)),
	.accel = FB_ACCEL_NONE,
};

/* We implement our own mmap to set MAY_SHARE and add the correct size */
static int altfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	vma->vm_flags |= VM_MAYSHARE | VM_SHARED;

	vma->vm_start = info->screen_base;
	vma->vm_end = vma->vm_start + info->fix.smem_len;
	return 0;
}

static int altfb_setcolreg(unsigned regno, unsigned red, unsigned green,
			   unsigned blue, unsigned transp,
			   struct fb_info *info)
{
    /*
     *  Set a single color register. The values supplied have a 32/16 bit
     *  magnitude.
     *  Return != 0 for invalid regno.
     */

    if (regno > 255)
	    return 1;
#if (BPX == 16)
    red>>=11;
    green>>=10;
    blue>>=11;

    if (regno < 255) {
	((u32 *)info->pseudo_palette)[regno] = ((red & 31) <<11) |
					       ((green & 63) << 5) |
					       (blue & 31);
    }
#else
    red>>=8;
    green>>=8;
    blue>>=8;

    if (regno < 255) {
	((u32 *)info->pseudo_palette)[regno] = ((red & 255) <<16) |
					       ((green & 255) << 8) |
					       (blue & 255);
    }
#endif
    return 0;
}

static struct fb_ops altfb_ops = {
	.owner		= THIS_MODULE,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_mmap = altfb_mmap,
	.fb_setcolreg = altfb_setcolreg,
};

static void altfb_platform_release(struct device *device)
{
	/* This is called when the reference count goes to zero. */
	dev_err(device,
		"This driver is broken, please bug the authors so they will fix it.\n");
}

/*
 *  Initialization
 */

#ifdef SGDMABASE

#define ALTERA_SGDMA_IO_EXTENT 0x400

#define ALTERA_SGDMA_STATUS 0
#define ALTERA_SGDMA_STATUS_BUSY_MSK (0x10)

#define ALTERA_SGDMA_CONTROL 16
#define ALTERA_SGDMA_CONTROL_RUN_MSK  (0x20)
#define ALTERA_SGDMA_CONTROL_SOFTWARERESET_MSK (0X10000)
#define ALTERA_SGDMA_CONTROL_PARK_MSK (0X20000)

#define ALTERA_SGDMA_NEXT_DESC_POINTER 32

/* SGDMA can only transfer this many bytes per descriptor */
#define DISPLAY_BYTES_PER_DESC 0xFF00
#define ALTERA_SGDMA_DESCRIPTOR_CONTROL_GENERATE_EOP_MSK (0x1)
#define ALTERA_SGDMA_DESCRIPTOR_CONTROL_GENERATE_SOP_MSK (0x4)
#define ALTERA_SGDMA_DESCRIPTOR_CONTROL_OWNED_BY_HW_MSK (0x80)

typedef struct {
	u32 *read_addr;
	u32 read_addr_pad;

	u32 *write_addr;
	u32 write_addr_pad;

	u32 *next;
	u32 next_pad;

	u16 bytes_to_transfer;
	u8 read_burst;
	u8 write_burst;

	u16 actual_bytes_transferred;
	u8 status;
	u8 control;

} __attribute__ ((packed)) sgdma_desc;

static int altfb_dma_start(unsigned long start, unsigned long len)
{
	unsigned long base =
	    (unsigned long)ioremap(SGDMABASE, ALTERA_SGDMA_IO_EXTENT);
	sgdma_desc *desc, *desc1;
	int ndesc = (len + DISPLAY_BYTES_PER_DESC - 1) / DISPLAY_BYTES_PER_DESC;
	int ndesc_size = sizeof(sgdma_desc) * ndesc;
	int i;

	writel(ALTERA_SGDMA_CONTROL_SOFTWARERESET_MSK,
	       base + ALTERA_SGDMA_CONTROL);	/* halt current transfer */
	writel(0, base + ALTERA_SGDMA_CONTROL);	/* disable interrupts */
	writel(0xff, base + ALTERA_SGDMA_STATUS);	/* clear status */

	/* assume cache line size is 32, which is required by sgdma desc */
	desc1 = kzalloc(ndesc_size, GFP_KERNEL);
	if (desc1 == NULL)
		return -ENOMEM;
	desc1 = ioremap((unsigned long)desc1, ndesc_size);

	for (i = 0, desc = desc1; i < ndesc; i++, desc++) {
		unsigned ctrl = ALTERA_SGDMA_DESCRIPTOR_CONTROL_OWNED_BY_HW_MSK;
		desc->read_addr = (void *)start;
		if (i == (ndesc - 1)) {
			desc->next = (void *)desc1;
			desc->bytes_to_transfer = len;
			ctrl |=
			    ALTERA_SGDMA_DESCRIPTOR_CONTROL_GENERATE_EOP_MSK;
		} else {
			desc->next = (void *)(desc + 1);
			desc->bytes_to_transfer = DISPLAY_BYTES_PER_DESC;
		}
		if (i == 0)
			ctrl |=
			    ALTERA_SGDMA_DESCRIPTOR_CONTROL_GENERATE_SOP_MSK;
		desc->control = ctrl;
		start += DISPLAY_BYTES_PER_DESC;
		len -= DISPLAY_BYTES_PER_DESC;
	}

	writel((unsigned long)desc1, base + ALTERA_SGDMA_NEXT_DESC_POINTER);
	writel(ALTERA_SGDMA_CONTROL_RUN_MSK | ALTERA_SGDMA_CONTROL_PARK_MSK,
	       base + ALTERA_SGDMA_CONTROL);	/* start */
	return 0;
}
#else
static int altfb_dma_start(unsigned long start, unsigned long len)
{
	unsigned long base = ioremap(VGABASE, 16);
	writel(0x0, base + 0);	/* Reset the VGA controller */
	writel(start, base + 4);	/* Where our frame buffer starts */
	writel(len, base + 8);	/* buffer size */
	writel(0x1, base + 0);	/* Set the go bit */
	return 0;
}
#endif

static int __init altfb_probe(struct platform_device *dev)
{
	struct fb_info *info;
	int retval = -ENOMEM;
	void * fbmem_virt;

	altfb_fix.smem_len = VIDEOMEMSIZE;
	if (!(fbmem_virt = dma_alloc_coherent(NULL, altfb_fix.smem_len, 
			&altfb_fix.smem_start, GFP_KERNEL))) {
		printk("altfb: unable to allocate %d Bytes fb memory\n",
		       altfb_fix.smem_len);
		return -ENOMEM;
	}

	info = framebuffer_alloc(sizeof(u32) * 256, &dev->dev);
	if (!info)
		goto err;

	info->screen_base = fbmem_virt;
	info->fbops = &altfb_ops;
	info->var = altfb_default;
	info->fix = altfb_fix;
	info->pseudo_palette = info->par;
	info->par = NULL;
	info->flags = FBINFO_FLAG_DEFAULT;

	retval = fb_alloc_cmap(&info->cmap, 256, 0);
	if (retval < 0)
		goto err1;

	retval = register_framebuffer(info);
	if (retval < 0)
		goto err2;
	platform_set_drvdata(dev, info);

	if (altfb_dma_start(altfb_fix.smem_start, altfb_fix.smem_len))
		goto err2;

	printk(KERN_INFO "fb%d: %s frame buffer device\n", info->node,
	       info->fix.id);
	return 0;
      err2:
	fb_dealloc_cmap(&info->cmap);
      err1:
	framebuffer_release(info);
      err:
	dma_free_coherent(NULL, altfb_fix.smem_len, fbmem_virt, 
		altfb_fix.smem_start);
	return retval;
}

static int altfb_remove(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);

	if (info) {
		unregister_framebuffer(info);
		dma_free_coherent(NULL, info->fix.smem_len, info->screen_base, 
			info->fix.smem_start);
		framebuffer_release(info);
	}
	return 0;
}

static struct platform_driver altfb_driver = {
	.probe = altfb_probe,
	.remove = altfb_remove,
	.driver = {
		   .name = "altfb",
		   },
};

static struct platform_device altfb_device = {
	.name = "altfb",
	.id = 0,
	.dev = {
		.release = altfb_platform_release,
		}
};

static int __init altfb_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&altfb_driver);

	if (!ret) {
		ret = platform_device_register(&altfb_device);
		if (ret)
			platform_driver_unregister(&altfb_driver);
	}
	return ret;
}

module_init(altfb_init);

#ifdef MODULE
static void __exit altfb_exit(void)
{
	platform_device_unregister(&altfb_device);
	platform_driver_unregister(&altfb_driver);
}

module_exit(altfb_exit);

MODULE_LICENSE("GPL");
#endif /* MODULE */
