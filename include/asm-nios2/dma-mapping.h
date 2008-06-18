#ifndef _NIOS2_DMA_MAPPING_H
#define _NIOS2_DMA_MAPPING_H

#ifdef CONFIG_PCI
#include <asm-generic/dma-mapping.h>
#else
#include <asm-generic/dma-mapping-broken.h>
#endif

#endif  /* _NIOS2_DMA_MAPPING_H */
