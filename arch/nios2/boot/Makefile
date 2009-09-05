#
# arch/nios2/boot/Makefile
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#

MKIMAGE := $(srctree)/scripts/mkuboot.sh

targets := vmImage
extra-y += vmlinux.bin vmlinux.gz
subdir- := compressed

quiet_cmd_uimage = UIMAGE  $@
      cmd_uimage = $(CONFIG_SHELL) $(MKIMAGE) -A $(ARCH) -O linux -T kernel \
                   -C gzip -n 'Linux-$(KERNELRELEASE)' \
                   -a $(shell $(NM) vmlinux | awk '$$NF == "_stext" {print $$1}') \
                   -e $(shell $(NM) vmlinux | awk '$$NF == "_start" {print $$1}') \
                   -d $< $@

OBJCOPYFLAGS_vmlinux.bin := -O binary
$(obj)/vmlinux.bin: vmlinux FORCE
	$(call if_changed,objcopy)

$(obj)/vmlinux.gz: $(obj)/vmlinux.bin FORCE
	$(call if_changed,gzip)

$(obj)/vmImage: $(obj)/vmlinux.gz
	$(call if_changed,uimage)
	@$(kecho) 'Kernel: $@ is ready'

$(obj)/zImage: $(obj)/compressed/vmlinux FORCE
	$(call if_changed,objcopy)
	@$(kecho) 'Kernel: $@ is ready'

$(obj)/compressed/vmlinux: $(obj)/vmlinux.gz FORCE
	$(Q)$(MAKE) $(build)=$(obj)/compressed $@

install:
	sh $(srctree)/$(src)/install.sh $(KERNELRELEASE) $(BOOTIMAGE) System.map "$(INSTALL_PATH)"
