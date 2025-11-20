CFLAGS_MCU += -mtune=cortex-m0plus -mcpu=cortex-m0plus -msoft-float
#for TinyUSB library
CFLAGS_MCU += -DCFG_TUSB_MCU=OPT_MCU_SAML21
#CPU compiler 
MPY_CROSS_MCU_ARCH = armv6m


#Call this if MP_CODESIZE equal 232k (FLASH size-Bootloader size. 256k-24k)
ifeq ($(MICROPY_HW_CODESIZE), 232K)
    FROZEN_MANIFEST ?= mcu/$(MCU_SERIES_LOWER)/manifest.py
endif

ifeq ($(MICROPY_HW_CODESIZE), 256K)
    FROZEN_MANIFEST ?= mcu/$(MCU_SERIES_LOWER)/manifest.py
endif
#VfsLfs1 function
MICROPY_VFS_LFS1 ?= 1

SRC_S += shared/runtime/gchelper_thumb1.s

#0x1851780a UF2 converter SAML21 family ID
UF2CONV_FLAGS += -f 0x1851780a

