CURRENT=$(shell uname -r )
KDIR = /lib/modules/$(CURRENT)/build
PWD=$(shell pwd )

TARGET1 = 1ppsusbss-transmitter
TARGET2 = 1ppsusbss-receiver
TARGET3 = 1pps-tx

obj-m += $(TARGET1).o $(TARGET2).o $(TARGET3).o

ccflags-y = -Wno-declaration-after-statement -mpopcnt -fno-pie

default: all

all:
	make -C $(KDIR) M=$(PWD) modules

clean:
	@rm -f *.o .*.cmd .*.flags *.mod.c *.order
	@rm -f .*.*.cmd *~ *.*~ TODO.* 
	@rm -fR .tmp*
	@rm -rf .tmp_versions 
	@rm -f *.ko *.symvers

disclean: clean 
	@rm *.ko *.symvers
