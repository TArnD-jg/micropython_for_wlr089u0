# Select the board to build for:
ifdef BOARD_DIR
# Custom board path - remove trailing slash and get the final component of
# the path as the board name.
BOARD ?= $(notdir $(BOARD_DIR:/=))
else
# If not given on the command line, then default to WLR089U0.
BOARD ?= WLR089U0
BOARD_DIR ?= boards/$(BOARD)
endif

ifeq ($(wildcard $(BOARD_DIR)/.),)
$(error Invalid BOARD specified: $(BOARD_DIR))
endif

#If EXTRAS  hasn't been defined then it will compile as 0
ifndef EXTRAS
EXTRAS=0
CFLAGS += -DEXTRAS=$(EXTRAS)
endif
#If BOOTLOADER hasn't been defined then it will compile as 0
ifndef BOOTLOADER
BOOTLOADER=0
endif

#defining EXTRAS must give 0 or 1 see mpconfigmcu.h
ifneq ($(EXTRAS),)
CFLAGS += -DEXTRAS=$(EXTRAS)
endif

#Checking if BOOTLOADER or EXTRAS has only 1 or 0 as value if not then error will be raised
ifneq ($(BOOTLOADER),0)
    ifneq ($(BOOTLOADER),1)
        $(error BOOTLOADER can only be 0 or 1 (Most: $(BOOTLOADER)))
    endif
endif
ifneq ($(EXTRAS),0)
    ifneq ($(EXTRAS),1)
        $(error EXTRAS can only be 0 or 1 (Most: $(BOOTLOADER)))
    endif
endif

#Name of firmware's dir
BUILD ?= build-$(BOARD)-B$(BOOTLOADER)-E$(EXTRAS)

CROSS_COMPILE ?= arm-none-eabi-
UF2CONV ?= $(TOP)/tools/uf2conv.py

MCU_SERIES_LOWER = $(shell echo $(MCU_SERIES) | tr '[:upper:]' '[:lower:]')

include ../../py/mkenv.mk
include $(BOARD_DIR)/mpconfigboard.mk
include mcu/$(MCU_SERIES_LOWER)/mpconfigmcu.mk

# Qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h
QSTR_GLOBAL_DEPENDENCIES = $(BOARD_DIR)/mpconfigboard.h mcu/$(MCU_SERIES_LOWER)/mpconfigmcu.h

FROZEN_MANIFEST ?= boards/manifest.py

# Include py core make definitions
include $(TOP)/py/py.mk
include $(TOP)/extmod/extmod.mk

GIT_SUBMODULES += lib/tinyusb

INC += -I.
INC += -I$(TOP)
INC += -I$(BUILD)
INC += -I$(BOARD_DIR)
INC += -Imcu/$(MCU_SERIES_LOWER)
INC += -Iasf4/$(MCU_SERIES_LOWER)/CMSIS/Core/Include
INC += -Iasf4/$(MCU_SERIES_LOWER)/hal/include
INC += -Iasf4/$(MCU_SERIES_LOWER)/hal/utils/include
INC += -Iasf4/$(MCU_SERIES_LOWER)/config
INC += -Iasf4/$(MCU_SERIES_LOWER)/hri
INC += -Iasf4/$(MCU_SERIES_LOWER)/hpl/core
INC += -Iasf4/$(MCU_SERIES_LOWER)/hpl/gclk
INC += -Iasf4/$(MCU_SERIES_LOWER)/hpl/pm
INC += -Iasf4/$(MCU_SERIES_LOWER)/hpl/port
INC += -Iasf4/$(MCU_SERIES_LOWER)/hpl/rtc
INC += -Iasf4/$(MCU_SERIES_LOWER)/hpl/tc
INC += -Iasf4/$(MCU_SERIES_LOWER)/include
INC += -Iasf4/$(MCU_SERIES_LOWER)/include/pio
INC += -I$(TOP)/lib/tinyusb/src
INC += -I$(TOP)/shared/tinyusb/

MAKE_PINS = boards/make-pins.py
BOARD_PINS = $(BOARD_DIR)/pins.csv
PREFIX_FILE = boards/pins_prefix.c

ifeq ($(BOARD),WLR089U0)
AF_FILE = mcu/$(MCU_SERIES_LOWER)/pin-af-table_WLR089U0.csv
else
AF_FILE = mcu/$(MCU_SERIES_LOWER)/pin-af-table.csv
endif
GEN_PINS_SRC = $(BUILD)/pins_$(BOARD).c
GEN_PINS_HDR = $(HEADER_BUILD)/pins.h

CFLAGS += $(INC) -Wall -Werror -std=c99 -nostdlib -mthumb $(CFLAGS_MCU) -fsingle-precision-constant -Wdouble-promotion
CFLAGS += -DMCU_$(MCU_SERIES) -D__$(CMSIS_MCU)__
CFLAGS += $(CFLAGS_EXTRA)

# Strip the letter 'K' from MICROPY_HW_CODESIZE for use by C code.
CFLAGS += -DMICROPY_HW_CODESIZE=$(strip $(subst K,' ', $(MICROPY_HW_CODESIZE)))

LDFLAGS += -nostdlib $(addprefix -T,$(LD_FILES)) -Map=$@.map --cref
LDFLAGS += --defsym=_codesize=$(MICROPY_HW_CODESIZE)

LIBS += $(shell $(CC) $(CFLAGS) -print-libgcc-file-name)

# Tune for Debugging or Optimization
CFLAGS += -g  # always include debug info in the ELF file
ifeq ($(DEBUG),1)
CFLAGS += -O0
else
CFLAGS += -Os -DNDEBUG
LDFLAGS += --gc-sections --print-memory-usage
CFLAGS += -fdata-sections -ffunction-sections
endif

# Flags for optional C++ source code
CXXFLAGS += $(filter-out -std=c99,$(CFLAGS))

# TODO make this common -- shouldn't be using these "private" vars from py.mk
ifneq ($(SRC_CXX)$(SRC_USERMOD_CXX)$(SRC_USERMOD_LIB_CXX),)
LIBSTDCPP_FILE_NAME = "$(shell $(CXX) $(CXXFLAGS) -print-file-name=libstdc++.a)"
LDFLAGS += -L"$(shell dirname $(LIBSTDCPP_FILE_NAME))"
endif

LDFLAGS += --wrap=dcd_event_handler

MPY_CROSS_FLAGS += -march=$(MPY_CROSS_MCU_ARCH)

SRC_C += \
	mcu/$(MCU_SERIES_LOWER)/clock_config.c \
	help.c \
	machine_i2c.c \
	machine_pin.c \
	machine_rtc.c \
	machine_spi.c \
	main.c \
	modsamd.c \
	mphalport.c \
	pendsv.c \
	pin_af.c \
	samd_flash.c \
	samd_isr.c \
	samd_soc.c \
	samd_spiflash.c \
	usbd.c\
	machine_aes.c\
	machine_bitstream.c \

ifneq ($(BOARD),WLR089U0)
SRC_C += \
	machine_dac.c 
endif

SHARED_SRC_C += \
	shared/runtime/mpirq.c \
	shared/libc/printf.c \
	shared/libc/string0.c \
	shared/readline/readline.c \
	shared/runtime/gchelper_native.c \
	shared/runtime/interrupt_char.c \
	shared/runtime/pyexec.c \
	shared/runtime/softtimer.c \
	shared/runtime/stdout_helpers.c \
	shared/runtime/sys_stdio_mphal.c \
	shared/timeutils/timeutils.c \
	shared/tinyusb/mp_usbd.c \
	shared/tinyusb/mp_usbd_cdc.c \
	shared/tinyusb/mp_usbd_descriptor.c \
	shared/tinyusb/mp_usbd_runtime.c 
	
ASF4_SRC_C += $(addprefix asf4/$(MCU_SERIES_LOWER)/,\
	hal/src/hal_atomic.c \
	hal/src/hal_flash.c \
	hpl/nvmctrl/hpl_nvmctrl.c \
	hpl/aes/hpl_aes.c\
	hal/src/hal_aes_sync.c\
	)

LIBM_SRC_C += $(SRC_LIB_LIBM_C)
LIBM_SRC_C += $(SRC_LIB_LIBM_SQRT_SW_C)

TINYUSB_SRC_C += $(addprefix lib/tinyusb/src/,\
	class/cdc/cdc_device.c \
	common/tusb_fifo.c \
	device/usbd.c \
	device/usbd_control.c \
	portable/microchip/samd/dcd_samd.c \
	tusb.c \
	)

DRIVERS_SRC_C += \
	drivers/memory/spiflash.c\
	drivers/bus/softspi.c \

# List of sources for qstr extraction
SRC_QSTR += $(SRC_C) $(SHARED_SRC_C) $(SRC_CXX) $(GEN_PINS_SRC)

OBJ += $(PY_O)
OBJ += $(addprefix $(BUILD)/, $(SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(SRC_CXX:.cpp=.o))
OBJ += $(addprefix $(BUILD)/, $(SRC_S:.s=.o))
OBJ += $(addprefix $(BUILD)/, $(SHARED_SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(ASF4_SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(LIBM_SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(DRIVERS_SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(TINYUSB_SRC_C:.c=.o))
OBJ += $(GEN_PINS_SRC:.c=.o)

all: $(BUILD)/$(BOARD)_B$(BOOTLOADER)_E$(EXTRAS).uf2\
     $(BUILD)/$(BOARD)_B$(BOOTLOADER)_E$(EXTRAS).hex
     
# Convert the production ELF file to a HEX file.
$(BUILD)/$(BOARD)_B$(BOOTLOADER)_E$(EXTRAS).hex: $(BUILD)/$(BOARD)_B$(BOOTLOADER)_E$(EXTRAS).elf
	$(ECHO) "Creating HEX from ELF: $@"
	$(Q)$(CROSS_COMPILE)objcopy -O ihex $^ $@
	$(ECHO) "Production HEX file created at: $@"
	
$(BUILD)/$(BOARD)_B$(BOOTLOADER)_E$(EXTRAS).elf: $(OBJ)
	$(ECHO) "LINK $@"
	$(Q)$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)

$(BUILD)/$(BOARD)_B$(BOOTLOADER)_E$(EXTRAS).bin: $(BUILD)/$(BOARD)_B$(BOOTLOADER)_E$(EXTRAS).elf
	$(Q)$(OBJCOPY) -O binary -j .isr_vector -j .text -j .data $^ $(BUILD)/$(BOARD)_B$(BOOTLOADER)_E$(EXTRAS).bin

$(BUILD)/$(BOARD)_B$(BOOTLOADER)_E$(EXTRAS).uf2: $(BUILD)/$(BOARD)_B$(BOOTLOADER)_E$(EXTRAS).bin
	$(Q)$(PYTHON) $(UF2CONV) $(UF2CONV_FLAGS) -b $(TEXT0) -c -o $@ $<

# pin_af.c: $(BUILD)/$(GEN_PIN_AF) | $(HEADER_BUILD)

# Use a pattern rule here so that make will only call make-pins.py once to make
# both pins_$(BOARD).c and pins.h
$(BUILD)/%_$(BOARD).c $(HEADER_BUILD)/%.h: $(BOARD_DIR)/%.csv $(MAKE_PINS) $(AF_FILE) $(PREFIX_FILE) | $(HEADER_BUILD)
	$(ECHO) "GEN $@"
	$(Q)$(PYTHON) $(MAKE_PINS) --board-csv $(BOARD_PINS) --af-csv $(AF_FILE) --prefix $(PREFIX_FILE) \
	    --output-source $(GEN_PINS_SRC) --output-header $(GEN_PINS_HDR) \
	    --mcu $(MCU_SERIES)

include $(TOP)/py/mkrules.mk
