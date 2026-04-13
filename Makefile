KERNELDIR := /home/alientek/rk3568/SDK/linux/rk3568_linux_sdk/kernel/
CURRENT_PATH := $(shell pwd)
CROSS_COMPILE := /home/alientek/rk3568/SDK/linux/rk3568_linux_sdk/prebuilts/gcc/linux-x86/aarch64/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
ARCH := arm64

# Multi-file module: use a name that differs from any source file
# to avoid kbuild circular dependency (ad5940_drv.o <- ad5940_drv.o)
ad5940-objs := ad5940_drv.o ad5940_core.o
obj-m := ad5940.o

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean
