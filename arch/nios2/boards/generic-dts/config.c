#include <linux/init.h>
#include <linux/of_platform.h>

static struct of_device_id altera_of_bus_ids[] __initdata = {
    { .compatible = "simple-bus", },
    { .compatible = "altera,avalon", },
    {}
};

static int __init nios2_device_probe(void)
{
	of_platform_bus_probe(NULL, altera_of_bus_ids, NULL);
	return 0;
}

device_initcall(nios2_device_probe);
