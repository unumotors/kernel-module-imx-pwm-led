MODULE_NAME = imx_pwm_led
OUT = $(MODULE_NAME).ko

KERNEL_SRC ?= /usr/src/kernel

# FIXME: Remove -DDEBUG
ccflags-y += -Wno-unknown-pragmas -DDEBUG

obj-m += $(MODULE_NAME).o

.PHONY: all clean modules_install

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
