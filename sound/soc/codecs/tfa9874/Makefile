# make sure that the environment variables ARCH and CROSS_COMPILE
# are set for your architecture and cross compiler
#
# e.g. for Beaglebone black:
# export ARCH := arm
# export CROSS_COMPILE := arm-linux-gnueabihf-
#
# e.g. Samsung Galaxy S6:
# export ARCH := arm64
# export CROSS_COMPILE := aarch64-linux-android-

# In case of out of tree build, build as a module
# (when build inside kernel we will not enter this directory and this will have no effect)

# additional debugging
#TFA_DEBUG=1

# debugging support (also enables trace_printk)
EXTRA_CFLAGS += -DDEBUG -DTFA9874_NONDSP_STEREO

EXTRA_CFLAGS += -I$(src)/inc
EXTRA_CFLAGS += -Werror

snd-soc-tfa9874-objs += tfa98xx.o
snd-soc-tfa9874-objs += tfa_container.o
snd-soc-tfa9874-objs += tfa_dsp.o
snd-soc-tfa9874-objs += tfa_init.o

ifdef TFA_DEBUG
EXTRA_CFLAGS += -DTFA_DEBUG -DDEBUG
snd-soc-tfa9874-objs += tfa_debug.o
endif

obj-$(CONFIG_SND_SOC_TFA9874) := snd-soc-tfa9874.o
