# Out-of-tree build of the IMX296 tegracam sensor driver for a Jetson L4T kernel.
#   make -C <kernel build dir> M=$PWD NVOOT=<nvidia-oot dir> KBUILD_EXTRA_SYMBOLS=<nvidia-oot>/Module.symvers modules
# See build.sh.
obj-m := imx296.o
NVOOT ?= /usr/src/nvidia/nvidia-oot
ccflags-y += -I$(NVOOT)/include
