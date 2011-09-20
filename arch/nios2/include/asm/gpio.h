#ifndef _ASM_NIOS2_GPIO_H
#define _ASM_NIOS2_GPIO_H

#include <asm-generic/gpio.h>		/* cansleep wrappers */

#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#define gpio_cansleep	__gpio_cansleep

#endif /* _ASM_NIOS2_GPIO_H */
