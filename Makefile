ifeq ($(DEVICE),$(filter $(DEVICE), S922X RK3588))
	obj-m := rocknix-joypad.o
else ifeq ($(DEVICE),$(filter $(DEVICE), H700 RK3399))
	obj-m := rocknix-singleadc-joypad.o
else
	obj-m := rocknix-joypad.o rocknix-singleadc-joypad.o
endif
