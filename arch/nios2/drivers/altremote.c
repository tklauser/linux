/*
 *  Altera remote update driver
 *
 *  (c) Copyright 2008 Walter Goossens <waltergoossens@home.nl>
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#define REG_MSM_STATE     0x00
#define REG_CDONE_CHECK   0x04
#define REG_WDOG_COUNTER  0x08
#define REG_WDOG_ENABLE   0x0C
#define REG_BOOT_ADDR     0x10
#define REG_INT_OSC       0x18
#define REG_CFG_SOURCE    0x1C
#define REG_GPR           0x80
#define CFG_CURR          0x00
#define CFG_PREV1         0x20
#define CFG_PREV2         0x40
#define CFG_INPUT         0x60

#define CFG_SOURCE_USER     0x01
#define CFG_SOURCE_WDOG     0x02
#define CFG_SOURCE_NSTATUS  0x04
#define CFG_SOURCE_CRC      0x08
#define CFG_SOURCE_NCONFIG  0x10
#define CFG_SOURCE_ALL      0x1F

#define PET_WDOG            0x02

/*
 * Set configuration offset shift depending on flash data width selected in
 * kernel configuration.
 */
#if defined(CONFIG_ALTERA_REMOTE_UPDATE_FLASH_WIDTH_8)
# define FPGA_IMAGE_SHIFT 2
#elif defined(CONFIG_ALTERA_REMOTE_UPDATE_FLASH_WIDTH_16)
# define FPGA_IMAGE_SHIFT 3
#else
# error "Flash type not supported"
#endif

struct altremote_data {
  void __iomem *base;
  struct resource *res;
  int initsteps;
};

static struct altremote_data altremote = {
  .base = NULL,
  .res = NULL,
  .initsteps = 0,
};

static unsigned long wdt_is_open;
static unsigned long wdt_timeout = 0;

/**
 *  altremote_wdt_pet
 *
 *  Reload counter one with the watchdog heartbeat.
 */
static void altremote_wdt_pet(void)
{
  iowrite32(PET_WDOG, altremote.base + REG_GPR);
  iowrite32(0, altremote.base + REG_GPR);
}

/**
 *  altremote_wdt_write:
 *  @file: file handle to the watchdog
 *  @buf: buffer to write (unused as data does not matter here
 *  @count: count of bytes
 *  @ppos: pointer to the position to write. No seeks allowed
 *
 *  A write to a watchdog device is defined as a keepalive signal. Any
 *  write of data will do, as we we don't define content meaning.
 */
static ssize_t altremote_wdt_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos) {
  if(count) {
    altremote_wdt_pet();
  }
  return count;
}

/**
 *  altremote_wdt_ioctl:
 *  @inode: inode of the device
 *  @file: file handle to the device
 *  @cmd: watchdog command
 *  @arg: argument pointer
 *
 *  The watchdog API defines a common set of functions for all watchdogs
 *  according to their available features. We only actually usefully support
 *  querying capabilities and current status.
 */
static int altremote_wdt_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
  void __user *argp = (void __user *)arg;

  static struct watchdog_info ident = {
    .options =  WDIOF_KEEPALIVEPING,
    .firmware_version = 1,
    .identity = "altremote_wdt",
  };

  switch(cmd) {
    case WDIOC_GETSUPPORT:
      if (copy_to_user(argp, &ident, sizeof(ident)))
        return -EFAULT;
      return 0;
    case WDIOC_KEEPALIVE:
      altremote_wdt_pet();
      return 0;
    default:
      return -EINVAL;
  }
}

/**
 *  altremote_wdt_open:
 *  @inode: inode of device
 *  @file: file handle to device
 *
 *  The watchdog device has been opened. The watchdog device is single
 *  open and on opening we load the counters. 
 *  The timeout depends on the value you selected in SOPC-builder.
 */
static int altremote_wdt_open(struct inode *inode, struct file *file) {
  if(test_and_set_bit(0, &wdt_is_open))
    return -EBUSY;
  return nonseekable_open(inode, file);
}

/**
 *  altremote_wdt_release:
 *  @inode: inode to board
 *  @file: file handle to board
 *
 */
static int altremote_wdt_release(struct inode *inode, struct file *file) {
  clear_bit(0, &wdt_is_open);
  printk(KERN_CRIT "altremote: WDT device closed unexpectedly.  WDT will (can) not stop!\n");
  altremote_wdt_pet();
  return 0;
}

/* following are the sysfs callback functions */
static ssize_t show_status(struct device *dev, struct device_attribute *attr, char *buf)
{
  int num = 0;
  u32 reg;
  u32 cfgfrom;
  u32 msmstate;
  const char *msg;
  char tempbuf[30];
  
  msmstate = ioread32(altremote.base + REG_MSM_STATE);
  cfgfrom = ioread32(altremote.base + (CFG_PREV1 | REG_CFG_SOURCE));
  if (cfgfrom > CFG_SOURCE_ALL) {
    msg = tempbuf;
    sprintf(tempbuf, "Unknown source 0x%X", cfgfrom);
  }
  else if (cfgfrom & CFG_SOURCE_NCONFIG)
    msg = "external configuration request (nCONFIG)";
  else if (cfgfrom & CFG_SOURCE_CRC)
    msg = "CRC Error in application";
  else if (cfgfrom & CFG_SOURCE_NSTATUS)
    msg = "nSTATUS assertion";
  else if (cfgfrom & CFG_SOURCE_WDOG)
    msg = "user watchdog timeout";
  else if (cfgfrom & CFG_SOURCE_USER)
    msg = "user request";
  else
    msg = "initial configuration";

  num = sprintf(buf, "Reconfigured by: %s\n", msg);
  /* Read the boot address.  In application mode, this seems to be in the
   * "past status 2" area (not documented). */
  if (msmstate & 1)
    reg = ioread32(altremote.base + (CFG_PREV2 | REG_BOOT_ADDR));
  else
    reg = ioread32(altremote.base + REG_BOOT_ADDR);
  num += sprintf(buf + num,"Configured from 0x%06X\n", reg);
  if(ioread32(altremote.base + (CFG_PREV1 | REG_WDOG_ENABLE)))
  {
    num += sprintf(buf + num, "Watchdog running\n");
  } else {
    num += sprintf(buf + num, "Watchdog NOT running\n");
  }
  switch (msmstate)
  {
  case 0:
    msg = "factory";
    break;
  case 1:
    msg = "application";
    break;
  case 3:
    msg = "application with watchdog";
    break;
  default:
    msg = tempbuf;
    sprintf(tempbuf, "unknown 0x%X", msmstate);
    break;
  }
  num += sprintf(buf + num, "Mode: %s\n", msg);
  if (((msmstate & 1) == 0) && (cfgfrom != 0)) {
    reg = ioread32(altremote.base + (CFG_PREV1 | REG_BOOT_ADDR));
  } else {
    reg = 0;
  }
  num += sprintf(buf + num, "Previously configured from 0x%06X\n", reg);

  return num;
}

static DEVICE_ATTR(status, S_IRUGO, show_status, NULL);

static ssize_t show_config_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
  u32 config_addr = ioread32(altremote.base + (CFG_INPUT | REG_BOOT_ADDR));
  return sprintf(buf, "0x%X\n", config_addr << FPGA_IMAGE_SHIFT);
}

static ssize_t set_config_addr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  unsigned long val = simple_strtoul(buf, NULL, 16);
  dev_printk(KERN_INFO, dev, "We'll try to reboot to 0x%lX (0x%lX)\n", val, val >> FPGA_IMAGE_SHIFT);
  iowrite32(val >> FPGA_IMAGE_SHIFT, altremote.base + REG_BOOT_ADDR);
  return count;
}

static DEVICE_ATTR(config_addr, S_IWUSR | S_IRUGO, show_config_addr, set_config_addr);

static ssize_t reconfig(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  dev_printk(KERN_WARNING, dev, "Warning! We'll reboot!\n");
  //Enable internal osc.
  iowrite32(0x00, altremote.base + REG_WDOG_ENABLE);
  if(wdt_timeout>0)
  {
    iowrite32(wdt_timeout, altremote.base + REG_WDOG_COUNTER);
    iowrite32(0x01, altremote.base + REG_WDOG_ENABLE);
  }
  iowrite32(0x01, altremote.base + REG_INT_OSC);
  iowrite32(0x01, altremote.base + REG_CDONE_CHECK);
  iowrite32(CFG_SOURCE_ALL, altremote.base + REG_CFG_SOURCE);
  iowrite32(0x01, altremote.base + REG_GPR);
  return count;
}

static DEVICE_ATTR(reconfig, S_IWUSR, NULL, reconfig);

static ssize_t show_watchdog(struct device *dev, struct device_attribute *attr, char *buf)
{
  if(altremote.initsteps>=4)
  {
    return sprintf(buf, "%u\n",
	ioread32(altremote.base + (CFG_PREV1 | REG_WDOG_COUNTER)) >> 17);
  } else {
    return sprintf(buf, "%lu\n", wdt_timeout);
  }
}

static ssize_t set_watchdog(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  if(altremote.initsteps>=4)
  {
    altremote_wdt_pet();
  } else {
    wdt_timeout = simple_strtoul(buf, NULL, 10);
  }
  return count;
}

static DEVICE_ATTR(watchdog, S_IWUSR | S_IRUGO, show_watchdog, set_watchdog);

static const struct file_operations altremote_wdt_fops = {
  .owner    = THIS_MODULE,
  .llseek   = no_llseek,
  .write    = altremote_wdt_write,
  .ioctl    = altremote_wdt_ioctl,
  .open     = altremote_wdt_open,
  .release  = altremote_wdt_release,
};

static struct miscdevice altremote_wdt_miscdev = {
  .minor  = WATCHDOG_MINOR,
  .name = "watchdog",
  .fops = &altremote_wdt_fops,
};

static int altremote_remove(struct platform_device* pdev)
{
  if(altremote.initsteps>=4)
  {
    misc_deregister(&altremote_wdt_miscdev);
  }
  if(altremote.initsteps>=3)
  {
    device_remove_file(&pdev->dev, &dev_attr_watchdog);
    device_remove_file(&pdev->dev, &dev_attr_config_addr);
    device_remove_file(&pdev->dev, &dev_attr_reconfig);
    device_remove_file(&pdev->dev, &dev_attr_status);
    iounmap(altremote.base);
  }
  if(altremote.initsteps>=2)
  {
    release_mem_region(altremote.res->start, resource_size(altremote.res));
  }
  altremote.initsteps = 0;
  return 0;
}

static int __devinit altremote_probe(struct platform_device *pdev)
{
  u32 status;
  int ret;

  if(altremote.initsteps != 0)
  {
    //We only allow ONE instance!!
    return -ENODEV;
  }
  altremote.res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!altremote.res)
    return -ENODEV;

  altremote.initsteps++;
  if (!request_mem_region(altremote.res->start, resource_size(altremote.res), pdev->name)) {
    dev_err(&pdev->dev, "Memory region busy\n");
    ret = -EBUSY;
    goto err_out_request_mem_region;
  }
  altremote.initsteps++;
  altremote.base = ioremap(altremote.res->start, resource_size(altremote.res));
  if (!altremote.base) {
    dev_err(&pdev->dev, "Unable to map registers\n");
    ret = -EIO;
    goto err_out_ioremap;
  }
  altremote.initsteps++;

  ret = device_create_file(&pdev->dev, &dev_attr_status);
  if (ret)
    goto err_out_sysfs;

  ret = device_create_file(&pdev->dev, &dev_attr_reconfig);
  if (ret)
    goto err_out_sysfs;

  status = ioread32(altremote.base + REG_MSM_STATE);
  switch(status)
  {
    case 0: {
      dev_info(&pdev->dev, "Found altremote block in Factory Mode\n");
      ret = device_create_file(&pdev->dev, &dev_attr_config_addr);
      if (ret)
        goto err_out_sysfs;

      ret = device_create_file(&pdev->dev, &dev_attr_watchdog);
      if (ret)
        goto err_out_sysfs;
    } break;
    case 1: {
      dev_printk(KERN_INFO, &pdev->dev, "Found altremote block in Application Mode\n");
    } break;
    case 3: {
      dev_printk(KERN_INFO, &pdev->dev, "Found altremote block in Application Mode with Watchdog enabled\n");
      ret = misc_register(&altremote_wdt_miscdev);
      if (ret) {
        printk(KERN_ERR "altremote_wdt: cannot register miscdev on minor=%d (err=%d)\n", WATCHDOG_MINOR, ret);
        goto err_out_misc_register;
      }
      ret = device_create_file(&pdev->dev, &dev_attr_watchdog);
      if (ret)
        goto err_out_wdt;
      altremote.initsteps++;
    } break;
    default: {
      dev_info(&pdev->dev, "Found altremote block unknwon state 0x%X\n", status);
    }
  }
  return 0;

err_out_wdt:
  misc_deregister(&altremote_wdt_miscdev);
err_out_misc_register:
err_out_sysfs:
  device_remove_file(&pdev->dev, &dev_attr_config_addr);
  device_remove_file(&pdev->dev, &dev_attr_reconfig);
  device_remove_file(&pdev->dev, &dev_attr_status);
  iounmap(altremote.base);
err_out_ioremap:
  release_mem_region(altremote.res->start, resource_size(altremote.res));
err_out_request_mem_region:
  altremote_remove(pdev);
  return ret;
}

static struct platform_driver altremote_driver = {
  .probe  = altremote_probe,
  .remove = __devexit_p(altremote_remove),
  .driver = {
    .owner = THIS_MODULE,
    .name = "altremote",
  },
};

static int __init altremote_init(void)
{
  return platform_driver_register(&altremote_driver);
}

static void __exit altremote_exit(void)
{
  platform_driver_unregister(&altremote_driver);
}

module_init(altremote_init);
module_exit(altremote_exit);

MODULE_AUTHOR("Walter Goossens <waltergoossens@home.nl>");
MODULE_DESCRIPTION("Altera Remote Update Driver");
MODULE_LICENSE("GPL");
