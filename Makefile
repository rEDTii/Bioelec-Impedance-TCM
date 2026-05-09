KERNELDIR := /home/alientek/rk3568/SDK/linux/rk3568_linux_sdk/kernel/
CURRENT_PATH := $(shell pwd)
CROSS_COMPILE := /home/alientek/rk3568/SDK/linux/rk3568_linux_sdk/prebuilts/gcc/linux-x86/aarch64/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
ARCH := arm64

# Cross-compiler for user-space programs
CC := $(CROSS_COMPILE)gcc

# libiio static library (cross-compiled for aarch64)
LIBIIO_INSTALL := $(CURRENT_PATH)/libiio/install

# Multi-file module: use a name that differs from any source file
# to avoid kbuild circular dependency (ad5940_drv.o <- ad5940_drv.o)
ad5940-objs := ad5940_drv.o ad5940_core.o
obj-m := ad5940.o

build: kernel_modules user_daemon

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

user_daemon: ad5940_bia_daemon

ad5940_bia_daemon: ad5940_bia_daemon.c
	$(CC) -o $@ $< \
		-I$(LIBIIO_INSTALL)/include/iio \
		-L$(LIBIIO_INSTALL)/lib \
		-liio -lm -lpthread -lrt -static

clean:
	rm -f ad5940_bia_daemon
	rm -f ad5940_drv.o ad5940_core.o ad5940.o ad5940.ko ad5940.mod.o ad5940.mod.c
	rm -f modules.order Module.symvers .ad5940.o.cmd .ad5940.mod.o.cmd .ad5940.ko.cmd
	rm -f .ad5940_core.o.cmd .ad5940_drv.o.cmd
	rm -rf .tmp_versions/

distclean: clean
	$(MAKE) -C $(CURRENT_PATH)/libiio/build clean 2>/dev/null || true
	rm -rf $(CURRENT_PATH)/libiio/install
