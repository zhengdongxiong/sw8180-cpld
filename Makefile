
DEBUG = n
CPLD_UDEV = n

ifeq ($(SW8180_UDEV), y)
	EXTRA_CFLAGS += -DSW8180_UDEV
endif

ifeq ($(DEBUG),y)
    EXTRA_CFLAGS += -DDEBUG
endif

#If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)
	obj-m += sw8180_cpld.o
# Otherwise we were called directly from the command
# line; invoke the kernel build system.
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	make -C $(KERNELDIR) M=$(PWD) clean
endif
