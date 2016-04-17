KERNERLDIR  :=/home/linux/exynos4412/linux-3.14-fs4412
#KERNERLDIR :=/lib/modules/$(shell  uname  -r )/build
123:
	$(MAKE)  -C   $(KERNERLDIR)    M=$(shell  pwd)  modules
clean:
	rm  -rf  *.ko  *.o  modules*  order*  *mod*  Module*
	
	
obj-m=lcd.o
