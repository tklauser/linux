#
# Makefile for the linux kernel.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#
# Note 2! The CFLAGS definitions are now in the main makefile...

extra-y := head.o vmlinux.lds

obj-y   := entry.o traps.o irq.o syscall.o syscalltable.o \
	    process.o signal.o setup.o sys_nios2.o \
	    usb.o  init_task.o \
	    time.o ptrace.o start.o nios2_ksyms.o insnemu.o

obj-$(CONFIG_MODULES) += module.o
obj-$(CONFIG_CONSOLE) += console.o
obj-$(CONFIG_AVALON_DMA) += dma.o
obj-$(CONFIG_EARLY_PRINTK) += early_printk.o
#obj-$(CONFIG_NIOS_DEBUG_DEV) += debugdev.o
