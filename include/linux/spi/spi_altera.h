#ifndef _LINUX_SPI_SPI_ALTERA_H
#define _LINUX_SPI_SPI_ALTERA_H

/*
 * struct altera_spi_platform_data - platform data of the Altera SPI
 * @interrupt:	use intrrupt driven data transfer.
 */
struct altera_spi_platform_data {
	int interrupt;
};

#endif /* _LINUX_SPI_SPI_ALTERA_H */
