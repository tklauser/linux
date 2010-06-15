#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/gpio.h>

#define ALTERA_PIO_DATA 0
#define ALTERA_PIO_DIR 4

int gpio_direction_input(unsigned gpio)
{
	struct altera_pio_port *p = &altera_pio_ports[gpio / 32];
	u32 mask = 1 << (gpio % 32);
	writel(p->dir_reg &= ~mask, p->membase + ALTERA_PIO_DIR);
	return 0;
}

int gpio_direction_output(unsigned gpio, int value)
{
	struct altera_pio_port *p = &altera_pio_ports[gpio / 32];
	u32 mask = 1 << (gpio % 32);
	u32 flags;
	local_irq_save(flags);
	if (value)
		p->data_reg |= mask;
	else
		p->data_reg &= ~mask;
	writel(p->data_reg, p->membase + ALTERA_PIO_DATA);
	writel(p->dir_reg |= mask, p->membase + ALTERA_PIO_DIR);
	local_irq_restore(flags);
	return 0;
}

int gpio_get_value(unsigned gpio)
{
	struct altera_pio_port *p = &altera_pio_ports[gpio / 32];
	u32 mask = 1 << (gpio % 32);
	if (p->dir_reg & mask)
		return (p->data_reg & mask) ? 1 : 0;
	else
		return (readl(p->membase + ALTERA_PIO_DATA) & mask) ? 1 : 0;
}

void gpio_set_value(unsigned gpio, int value)
{
	struct altera_pio_port *p = &altera_pio_ports[gpio / 32];
	u32 mask = 1 << (gpio % 32);
	u32 flags;
	local_irq_save(flags);
	if (value)
		p->data_reg |= mask;
	else
		p->data_reg &= ~mask;
	writel(p->data_reg, p->membase + ALTERA_PIO_DATA);
	local_irq_restore(flags);
}
