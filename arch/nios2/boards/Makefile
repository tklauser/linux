board-$(CONFIG_3C120) := 3c120
board-$(CONFIG_NEEK) := neek
BOARD := $(board-y)
export BOARD

ifneq ($(board-y),)
core-y   += arch/$(ARCH)/boards/$(BOARD)/
KBUILD_AFLAGS += -Iarch/$(ARCH)/boards/$(BOARD)/include
KBUILD_CPPFLAGS += -Iarch/$(ARCH)/boards/$(BOARD)/include
KBUILD_CFLAGS += -Iarch/$(ARCH)/boards/$(BOARD)/include
endif
