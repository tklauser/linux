#
# Note, no support for gzipped uImages
#
# Copied from arm/boot/Makfile

MKIMAGE		:= $(srctree)/scripts/mkuboot.sh

extra-y		:= vmlinux.bin

.PHONY:  FORCE

OBJCOPYFLAGS_vmlinux.bin := -O binary
$(obj)/vmlinux.bin: vmlinux FORCE
	$(call if_changed,objcopy)

quiet_cmd_uimage = UIMAGE  $@
      cmd_uimage = $(CONFIG_SHELL) $(MKIMAGE) -A nios2 -O linux -T kernel \
               -C none -a 0xd0000000 -e 0xd0000000 -n 'Linux-$(KERNELRELEASE)' \
               -d $< $@

quiet_cmd_flashimage = FLASHUIMAGE  $@
      cmd_flashimage = $(OBJCOPY) --change-addr=$(CONFIG_NIOS2_UIMAGE_FLASHADDR) -I binary -O srec $< $@ 

$(obj)/uImage: $(obj)/vmlinux.bin FORCE
	$(Q)rm -f $@
	$(call if_changed,uimage)
	@echo -n '  Image: $@ '
	@if [ -f $@ ]; then echo 'is ready' ; else echo 'not made'; fi

$(obj)/uImage.flash: $(obj)/uImage FORCE
	$(Q)rm -f $@
	$(call if_changed,flashimage)
	@echo -n 'uImage.flash @$(CONFIG_NIOS2_UIMAGE_FLASHADDR) '
	@if [ -f $@ ]; then echo 'is ready' ; else echo 'not made'; fi


# Files generated that shall be removed upon make clean
clean-files	:= vmlinux* uImage*
