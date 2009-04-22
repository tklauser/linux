#ifndef _ASM_NIOS2_GPIO_H_
#define _ASM_NIOS2_GPIO_H_ 1

#include <linux/spinlock.h>
#include <asm/io.h>
extern resource_size_t nios2_gpio_mapbase;
extern spinlock_t nios2_gpio_lock;

#define ARCH_NR_GPIOS	32

#define AVALON_PIO_DATA (nios2_gpio_mapbase)
#define AVALON_PIO_DIR  (nios2_gpio_mapbase + 4)
#define AVALON_PIO_IMR  (nios2_gpio_mapbase + 8)
#define AVALON_PIO_SET  (nios2_gpio_mapbase + 16)
#define AVALON_PIO_RST  (nios2_gpio_mapbase + 20)

static inline int gpio_is_valid(int number)
{
	return (number >= 0) && (number < ARCH_NR_GPIOS);
}

static inline int gpio_request(unsigned gpio, const char *label)
{
	return 0;
}

static inline void gpio_free(unsigned gpio)
{
}

static inline int gpio_direction_input(unsigned gpio)
{
	unsigned long flags;
	spin_lock_irqsave(&nios2_gpio_lock, flags);
	writel(readl(AVALON_PIO_DIR) & ~(1 << gpio), AVALON_PIO_DIR);
	spin_unlock_irqrestore(&nios2_gpio_lock, flags);
	return 0;
}

static inline int gpio_direction_output(unsigned gpio, int value)
{
	unsigned long flags;
	if (value)
		writel(1 << gpio, AVALON_PIO_SET);
	else
		writel(1 << gpio, AVALON_PIO_RST);
	spin_lock_irqsave(&nios2_gpio_lock, flags);
	writel(readl(AVALON_PIO_DIR) | (1 << gpio), AVALON_PIO_DIR);
	spin_unlock_irqrestore(&nios2_gpio_lock, flags);
	return 0;
}

static inline int gpio_get_value(unsigned gpio)
{
	return (readl(AVALON_PIO_DATA) >> gpio) & 1;
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	if (value)
		writel(1 << gpio, AVALON_PIO_SET);
	else
		writel(1 << gpio, AVALON_PIO_RST);
}

static inline int gpio_cansleep(unsigned gpio)
{
	return 1;
}

static inline int gpio_get_value_cansleep(unsigned gpio)
{
	return gpio_get_value(gpio);
}

static inline void gpio_set_value_cansleep(unsigned gpio, int value)
{
	gpio_set_value(gpio, value);
}

static inline int gpio_export(unsigned gpio, bool direction_may_change)
{
	return 0;
}

static inline void gpio_unexport(unsigned gpio)
{
}

static inline int gpio_to_irq(unsigned gpio)
{
	return -EINVAL;
}

static inline int irq_to_gpio(unsigned irq)
{
	return -EINVAL;
}

#endif /* _ASM_NIOS2_GPIO_H_ */
