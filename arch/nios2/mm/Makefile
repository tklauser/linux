#
# Makefile for the Nios2-specific parts of the memory manager.
#

obj-y := init.o

obj-y += cacheflush$(MMU).o

obj-$(CONFIG_MMU) += pgtable.o tlb.o uaccess.o fault.o ioremap.o
obj-$(CONFIG_MMU) += extable.o mmu_context.o mmdump.o dma-noncoherent.o
