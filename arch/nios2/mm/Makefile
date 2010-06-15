# $Id: Makefile,v 1.1 2006/07/05 06:23:18 gerg Exp $
# Makefile for the linux nios2-specific parts of the memory manager.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#
# Note 2! The CFLAGS definition is now in the main makefile...

obj-y	:= init.o ioremap.o memory.o pgtable.o tlb.o uaccess.o fault.o
obj-y	+= dma-noncoherent.o
obj-y   += pgalloc.o
obj-y   += mmu_context.o
obj-y	+= mmdump.o extable.o cacheflush.o
