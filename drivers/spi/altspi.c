/* 
 * Altera Avalon SPI driver
 *
 * Copyright (C) 2008 Thomas Chou <thomas@wytron.com.tw>
 *
 * Based on linux/drivers/spi/spi_s3c24xx.c, which is:
 * Copyright (c) 2006 Ben Dooks
 * Copyright (c) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <asm/io.h>

#define ALTERA_SPI_RXDATA	0
#define ALTERA_SPI_TXDATA       4
#define ALTERA_SPI_STATUS       8
#define ALTERA_SPI_CONTROL      12
#define ALTERA_SPI_SLAVE_SEL    20

#define ALTERA_SPI_STATUS_ROE_MSK              (0x8)
#define ALTERA_SPI_STATUS_TOE_MSK              (0x10)
#define ALTERA_SPI_STATUS_TMT_MSK              (0x20)
#define ALTERA_SPI_STATUS_TRDY_MSK             (0x40)
#define ALTERA_SPI_STATUS_RRDY_MSK             (0x80)
#define ALTERA_SPI_STATUS_E_MSK                (0x100)

#define ALTERA_SPI_CONTROL_IROE_MSK            (0x8)
#define ALTERA_SPI_CONTROL_ITOE_MSK            (0x10)
#define ALTERA_SPI_CONTROL_ITRDY_MSK           (0x40)
#define ALTERA_SPI_CONTROL_IRRDY_MSK           (0x80)
#define ALTERA_SPI_CONTROL_IE_MSK              (0x100)
#define ALTERA_SPI_CONTROL_SSO_MSK             (0x400)

struct altera_spi {
	/* bitbang has to be first */
	struct spi_bitbang bitbang;
	struct completion done;

	unsigned long base;
	int irq;
	int len;
	int count;
    int bytesPerWord;
	unsigned long imr;

	/* data buffers */
	const unsigned char *tx;
	unsigned char *rx;

	struct resource *ioarea;
	struct spi_master *master;
	struct spi_device *curdev;
	struct device *dev;
};

static inline struct altera_spi *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static void altera_spi_chipsel(struct spi_device *spi, int value)
{
	struct altera_spi *hw = to_hw(spi);

	if (spi->mode & SPI_CS_HIGH) {
		switch (value) {
		case BITBANG_CS_INACTIVE:
			writel(1 << spi->chip_select,
			       hw->base + ALTERA_SPI_SLAVE_SEL);
			hw->imr |= ALTERA_SPI_CONTROL_SSO_MSK;
			writel(hw->imr, hw->base + ALTERA_SPI_CONTROL);
			break;

		case BITBANG_CS_ACTIVE:
			hw->imr &= ~ALTERA_SPI_CONTROL_SSO_MSK;
			writel(hw->imr, hw->base + ALTERA_SPI_CONTROL);
			writel(0, hw->base + ALTERA_SPI_SLAVE_SEL);
			break;
		}
	} else {
		switch (value) {
		case BITBANG_CS_INACTIVE:
			hw->imr &= ~ALTERA_SPI_CONTROL_SSO_MSK;
			writel(hw->imr, hw->base + ALTERA_SPI_CONTROL);
			break;

		case BITBANG_CS_ACTIVE:
			writel(1 << spi->chip_select,
			       hw->base + ALTERA_SPI_SLAVE_SEL);
			hw->imr |= ALTERA_SPI_CONTROL_SSO_MSK;
			writel(hw->imr, hw->base + ALTERA_SPI_CONTROL);
			break;
		}
	}
}

static int altera_spi_setupxfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct altera_spi *hw = to_hw(spi);

	spin_lock(&hw->bitbang.lock);
	if (!hw->bitbang.busy) {
		hw->bitbang.chipselect(spi, BITBANG_CS_INACTIVE);
		/* need to ndelay for 0.5 clocktick ? */
	}
	spin_unlock(&hw->bitbang.lock);

	return 0;
}

/* the spi->mode bits understood by this driver: */
#define MODEBITS (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH)

static int altera_spi_setup(struct spi_device *spi)
{
	int ret;

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	if (spi->mode & ~MODEBITS) {
		dev_dbg(&spi->dev, "setup: unsupported mode bits %x\n",
			spi->mode & ~MODEBITS);
		return -EINVAL;
	}

	ret = altera_spi_setupxfer(spi, NULL);
	if (ret < 0) {
		dev_err(&spi->dev, "setupxfer returned %d\n", ret);
		return ret;
	}

	dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n",
		__func__, spi->mode, spi->bits_per_word, spi->max_speed_hz);

	return 0;
}

static inline unsigned int hw_txbyte(struct altera_spi *hw, int count)
{
    if (hw->tx)
        switch (hw->bytesPerWord)
        {
            case 1:
                return hw->tx[count];
            case 2:
                return (hw->tx[count * 2] | (hw->tx[count * 2 + 1] << 8));
        }
    else return 0;
}

static int altera_spi_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	struct altera_spi *hw = to_hw(spi);

	dev_dbg(&spi->dev, "txrx: tx %p, rx %p, len %d\n",
		t->tx_buf, t->rx_buf, t->len);

	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;
	hw->count = 0;
    hw->bytesPerWord = (t->bits_per_word ? : spi->bits_per_word) / 8;
	hw->len = t->len / hw->bytesPerWord;

	init_completion(&hw->done);

	/* send the first byte */
	writel(hw_txbyte(hw, 0), hw->base + ALTERA_SPI_TXDATA);

	wait_for_completion(&hw->done);

	return hw->count * hw->bytesPerWord;
}

static irqreturn_t altera_spi_irq(int irq, void *dev)
{
	struct altera_spi *hw = dev;
	unsigned int spsta = readl(hw->base + ALTERA_SPI_STATUS);
	unsigned int count = hw->count;
	unsigned int rxd;

	if (spsta & ALTERA_SPI_STATUS_ROE_MSK) {
		dev_dbg(hw->dev, "data-collision\n");
		complete(&hw->done);
		goto irq_done;
	}

	if (!(spsta & ALTERA_SPI_STATUS_TRDY_MSK)) {
		dev_dbg(hw->dev, "spi not ready for tx?\n");
		complete(&hw->done);
		goto irq_done;
	}

	hw->count++;

	rxd = readl(hw->base + ALTERA_SPI_RXDATA);
	if (hw->rx)
        switch (hw->bytesPerWord)
        {
            case 1:
                hw->rx[count] = rxd;
                break;
            case 2:
                hw->rx[count * 2] = rxd;
                hw->rx[count * 2 + 1] = rxd >> 8;
                break;
        }

	count++;

	if (count < hw->len)
		writel(hw_txbyte(hw, count), hw->base + ALTERA_SPI_TXDATA);
	else
		complete(&hw->done);

      irq_done:
	return IRQ_HANDLED;
}

static int __init altera_spi_probe(struct platform_device *pdev)
{
	struct altera_spi *hw;
	struct spi_master *master;
	struct resource *res;
	int err = 0;

	master = spi_alloc_master(&pdev->dev, sizeof(struct altera_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	hw = spi_master_get_devdata(master);
	memset(hw, 0, sizeof(struct altera_spi));

	hw->master = spi_master_get(master);
	hw->dev = &pdev->dev;

	platform_set_drvdata(pdev, hw);
	init_completion(&hw->done);

	/* setup the master state. */

	master->bus_num = pdev->id;
	master->num_chipselect = 16;

	/* setup the state for the bitbang driver */

	hw->bitbang.master = hw->master;
	hw->bitbang.setup_transfer = altera_spi_setupxfer;
	hw->bitbang.chipselect = altera_spi_chipsel;
	hw->bitbang.txrx_bufs = altera_spi_txrx;
	hw->bitbang.master->setup = altera_spi_setup;

	dev_dbg(hw->dev, "bitbang at %p\n", &hw->bitbang);

	/* find and map our resources */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_iores;
	}

	hw->ioarea = request_mem_region(res->start, (res->end - res->start) + 1,
					pdev->name);

	if (hw->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve region\n");
		err = -ENXIO;
		goto err_no_iores;
	}

	hw->base =
	    (unsigned long)ioremap(res->start, (res->end - res->start) + 1);
	if (hw->base == 0) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}

	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_no_irq;
	}

	/* program defaults into the registers */
	hw->imr = 0;		/* disable spi interrupts */
	writel(hw->imr, hw->base + ALTERA_SPI_CONTROL);
	writel(0, hw->base + ALTERA_SPI_STATUS);	/* clear status reg */
	if (readl(hw->base + ALTERA_SPI_STATUS) & ALTERA_SPI_STATUS_RRDY_MSK)
		readl(hw->base + ALTERA_SPI_RXDATA);	/* flush rxdata */

	err = request_irq(hw->irq, altera_spi_irq, 0, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_no_irq;
	}

	hw->imr |= ALTERA_SPI_CONTROL_IRRDY_MSK;	/* enable receive interrupt */
	writel(hw->imr, hw->base + ALTERA_SPI_CONTROL);

	/* register our spi controller */

	err = spi_bitbang_start(&hw->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master\n");
		goto err_register;
	}

	return 0;

      err_register:
	free_irq(hw->irq, hw);

      err_no_irq:
	iounmap((void *)hw->base);

      err_no_iomap:
	release_resource(hw->ioarea);
	kfree(hw->ioarea);

      err_no_iores:
	spi_master_put(hw->master);;

      err_nomem:
	return err;
}

static int __exit altera_spi_remove(struct platform_device *dev)
{
	struct altera_spi *hw = platform_get_drvdata(dev);

	/* diable spi intrrupts */
	hw->imr = 0;
	writel(hw->imr, hw->base + ALTERA_SPI_CONTROL);

	platform_set_drvdata(dev, NULL);

	spi_unregister_master(hw->master);

	free_irq(hw->irq, hw);
	iounmap((void *)hw->base);

	release_resource(hw->ioarea);
	kfree(hw->ioarea);

	spi_master_put(hw->master);
	return 0;
}

#define altera_spi_suspend NULL
#define altera_spi_resume  NULL

static struct platform_driver altera_spidrv = {
	.remove = __exit_p(altera_spi_remove),
	.suspend = altera_spi_suspend,
	.resume = altera_spi_resume,
	.driver = {
		   .name = "altspi",
		   .owner = THIS_MODULE,
		   },
};

static int __init altera_spi_init(void)
{
	return platform_driver_probe(&altera_spidrv, altera_spi_probe);
}

static void __exit altera_spi_exit(void)
{
	platform_driver_unregister(&altera_spidrv);
}

module_init(altera_spi_init);
module_exit(altera_spi_exit);

MODULE_DESCRIPTION("ALTERA SPI Driver");
MODULE_AUTHOR("Thomas Chou <thomas@wytron.com.tw>");
MODULE_LICENSE("GPL");
