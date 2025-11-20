MCU_SERIES = SAML21
CMSIS_MCU = SAML21J18B

ifeq ($(BOOTLOADER),1)
LD_FILES = boards/wlr089u0_flash_bootloader.ld sections.ld
#Application begins at 0x6000 because of SAMBA based Adafruit UF2 bootloader
TEXT0 = 0x6000 
#256k - first 0x6000 bytes = 232K
# The ?='s allow overriding in mpconfigboard.mk.
MICROPY_HW_CODESIZE ?= 232K

else ifeq ($(BOOTLOADER),0)
LD_FILES = boards/wlr089u0_flash.ld sections.ld
#Application begins at 0x0000 because we have no bootloader
TEXT0 = 0x0000 
MICROPY_HW_CODESIZE ?= 256K
endif