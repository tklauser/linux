#include <linux/init.h>
#include <linux/of_platform.h>

extern int __init altera_gpio_init(void);
extern int __init oc_bit_gpio_init(void);

static struct of_device_id altera_of_bus_ids[] __initdata = {
    { .compatible = "simple-bus", },
    { .compatible = "altr,avalon", },
    {}
};

static int __init nios2_device_probe(void)
{
#ifdef CONFIG_GPIO_ALTERA
	altera_gpio_init();
#endif
#ifdef CONFIG_GPIO_OC_BIT
	oc_bit_gpio_init();
#endif
	of_platform_bus_probe(NULL, altera_of_bus_ids, NULL);
	return 0;
}

device_initcall(nios2_device_probe);
