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
#endif /* CONFIG_GPIOLIB */

#endif /* _ASM_NIOS2_GPIO_H */
