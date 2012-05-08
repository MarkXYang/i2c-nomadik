
# To build modules outside of the kernel tree, we run "make"
# in the kernel source tree; the Makefile these then includes this
# Makefile once again.
# This conditional selects whether we are being included from the
# kernel Makefile or not.
# called from kernel build system: just declare what our modules are

   # KERNELDIR environment variable is assumed to hold the kernel root directory
	# KERNELDIR = $(PRJROOT)/kernel/linux-2.6.38-rc8
	# The current directory is passed to sub-makes as argument
   PWD := $(shell pwd)

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

.PHONY: modules modules_install clean

obj-m := lis3lv02d-nhk8815.o


