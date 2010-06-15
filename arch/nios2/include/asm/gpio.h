#ifndef _ASM_NIOS2_GPIO_H_
#define _ASM_NIOS2_GPIO_H_ 1

#ifdef CONFIG_ALTERA_PIO_GPIO

struct altera_pio_port {
	unsigned long mapbase;	/* Physical address base */
	unsigned char __iomem *membase;
	u32 data_reg;
	u32 dir_reg;
};
extern struct altera_pio_port altera_pio_ports[];

static inline int gpio_is_valid(int number)
{
	return 1;
}

static inline int gpio_request(unsigned gpio, const char *label)
{
	return 0;
}

static inline void gpio_free(unsigned gpio)
{
}

int gpio_direction_input(unsigned gpio);
int gpio_direction_output(unsigned gpio, int value);
int gpio_get_value(unsigned gpio);
void gpio_set_value(unsigned gpio, int value);

static inline int gpio_cansleep(unsigned gpio)
{
	return 0;
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

#else

#include <asm/io.h>
extern resource_size_t nios2_gpio_mapbase;

#define AVALON_GPIO_PORT(p) (nios2_gpio_mapbase + ((p) << 2))

static inline int gpio_is_valid(int number)
{
	return 1;
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
	writel(1, AVALON_GPIO_PORT(gpio));
	return 0;
}

static inline int gpio_direction_output(unsigned gpio, int value)
{
	writel(value?3:2, AVALON_GPIO_PORT(gpio));
	return 0;
}

static inline int gpio_get_value(unsigned gpio)
{
	return readl(AVALON_GPIO_PORT(gpio));
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	writel(value?3:2, AVALON_GPIO_PORT(gpio));
}

static inline int gpio_cansleep(unsigned gpio)
{
	return 0;
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

#undef AVALON_GPIO_PORT

#endif /* CONFIG_ALTERA_PIO_GPIO */

#endif /* _ASM_NIOS2_GPIO_H_ */
