#ifndef _ASM_NIOS2_GPIO_H
#define _ASM_NIOS2_GPIO_H

#ifdef CONFIG_GPIOLIB

#include <asm-generic/gpio.h>		/* cansleep wrappers */

static inline int gpio_get_value(unsigned int gpio)
{
	return __gpio_get_value(gpio);
}

static inline void gpio_set_value(unsigned int gpio, int value)
{
	__gpio_set_value(gpio, value);
}

static inline int gpio_cansleep(unsigned int gpio)
{
	return __gpio_cansleep(gpio);
}

static inline int gpio_to_irq(unsigned gpio)
{
	return __gpio_to_irq(gpio);
}

static inline int irq_to_gpio(unsigned int irq)
{
	return -EINVAL;
}

#else /* !CONFIG_GPIOLIB */

#ifdef CONFIG_ALTERA_PIO_GPIO

struct altera_pio_port {
	unsigned long mapbase;	/* Physical address base */
	unsigned char __iomem *membase;
	u32 data_reg;
	u32 dir_reg;
};
extern struct altera_pio_port altera_pio_ports[];

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

static inline int gpio_to_irq(unsigned gpio)
{
	return -EINVAL;
}

static inline int irq_to_gpio(unsigned irq)
{
	return -EINVAL;
}

#include <asm-generic/gpio.h>		/* cansleep wrappers */

#endif /* CONFIG_ALTERA_PIO_GPIO */

#ifdef CONFIG_NIOS2_GPIO

#include <asm/io.h>

extern resource_size_t nios2_gpio_mapbase;

#define AVALON_GPIO_PORT(p) (nios2_gpio_mapbase + ((p) << 2))

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

static inline int gpio_to_irq(unsigned gpio)
{
	return -EINVAL;
}

static inline int irq_to_gpio(unsigned irq)
{
	return -EINVAL;
}

#undef AVALON_GPIO_PORT

#include <asm-generic/gpio.h>		/* cansleep wrappers */

#endif /* CONFIG_NIOS_GPIO */

#endif	/* !CONFIG_GPIOLIB */

#endif /* _ASM_NIOS2_GPIO_H */
