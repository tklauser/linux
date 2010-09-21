#include <asm-generic/dma.h>
void nios2_set_dma_waddr(unsigned int dmanr, unsigned int a);

void set_dma_count(unsigned int dmanr, unsigned int count);
void enable_dma(unsigned int dmanr);
void nios2_set_dma_waddr(unsigned int dmanr, unsigned int a);
void nios2_set_dma_handler(unsigned int dmanr, int (*handler)(void*, int), void* user);

void nios2_set_dma_data_width(unsigned int dmanr, unsigned int width);
void nios2_set_dma_rcon(unsigned int dmanr, unsigned int set);
void nios2_set_dma_wcon(unsigned int dmanr, unsigned int set);
void nios2_set_dma_raddr(unsigned int dmanr, unsigned int a);

void disable_dma(unsigned int dmanr);
