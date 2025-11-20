MicroPython 1.24 - Port for Microchip SAML21 (WLR089U0)
=======================================================

Non-official, experimental MicroPython port for Microchip WLR089U0 module
(based on ATSAML21 / ATSAMR34 with SX127x LoRa transceiver).

> **7.1 Cortex M0+ Processor** 
> The SAM R34/R35 contains an ATSAML21J18B ARM Cortex-M0+ processor, based on the ARMv6 Architecture and Thumb®-2 ISA. The Cortex-M0+ is 100% instruction set compatible with its predecessor, the Cortex-M0 core, and upward compatible to Cortex-M3 and M4 cores. The implemented ARM Cortex-M0+ is revision r0p1.
>
> — *SAM R34/R35 Low Power LoRa® Sub-GHz SiP Datasheet*, DS70005356B, p. 21, Microchip Technology Inc.

-------------------------------------------------------
Overview
--------

This port brings basic MicroPython functionality to the WLR089U0,
a module combining an ATSAML21 ARM Cortex-M0+ MCU with a LoRa SX127x radio.

Note: Only ATSAMR34-based WLR089U0 modules are supported
because USB hardware support is required.

This port is derived from the SAMD21 port due to the similar architecture.

-------------------------------------------------------
Documentation
-------------

SAM L21 Family Datasheet (DS60001477C):
https://ww1.microchip.com/downloads/en/DeviceDoc/SAM_L21_Family_DataSheet_DS60001477C.pdf

WLR089U0 Module Datasheet (70005435B):
https://ww1.microchip.com/downloads/aemDocuments/documents/WSG/ProductDocuments/DataSheets/70005435B.pdf

-------------------------------------------------------
Features
--------

- Experimental LoRa P2P communication via SX127x driver
- Compatible with MicroPython 1.24/1.25
- UF2 bootloader support for easy firmware flashing
- Works with ATSAMR34 (HW USB)
- Example MicroPython scripts in: boards/WLR089U0/examples/
Port of MicroPython to Microchip WLR089U0(SAML21/ATSAMR34/35 MCUs)
- Cryptolib based HW AES  (ECB,CBC,CTR)

-------------------------------------------------------
Connections of HW
-------------------------------------------------------

A simplified connection diagram can be found here:
`boards/WLR089U0/WLR089U0_Connections.png`

### 1. Flash Memory (Winbond W25Q16JV-IQ)
| Signal Name | Pin |
| :--- | :--- |
| **FLASH_MOSI** | PB22 |
| **FLASH_MISO** | PA22 |
| **FLASH_SCK** | PB23 |
| **FLASH_CS** | PA23 |

### 2. USB Interface
| Signal Name | Pin |
| :--- | :--- |
| **USB_DM** | PA24 |
| **USB_DP** | PA25 |

### 3. User Interface
* **Push Button (Reset):** RST Pin

### 4. LoRa Module (SX127x)

**SPI Interface (SPI-COM4):**
| Signal Name | Pin |
| :--- | :--- |
| **LORA_MOSI** | PB30 |
| **LORA_CS** | PB31 |
| **LORA_SCK** | PC18 |
| **LORA_MISO** | PC19 |

**DIO & Control Pins:**
| Signal Name | Pin | Description |
| :--- | :--- | :--- |
| **LORA_DIO0** | PB16 | Packet Received / Tx Done |
| **LORA_DIO1** | PA11 | Rx Timeout |
| **LORA_DIO2** | PA12 | Freq. Hop |
| **LORA_DIO3** | PB17 | - |
| **LORA_DIO4** | PA10 | - |
| **LORA_DIO5** | PB00 | - |
| **LORA_nRST** | PB15 | Reset |
| **LORA_PWR** | PA09 | RF Switch Power |

-------------------------------------------------------
Build Instructions
------------------

Clone MicroPython 1.24 release:

    git clone --branch v1.24-release --single-branch https://github.com/micropython/micropython.git

Copy this port into:
    micropython/ports/saml21/

1. Build the MicroPython cross-compiler
   (from the MicroPython root directory):

    make -C mpy-cross

2. Build submodules (for WLR089U0):

    cd ports/saml21
    make submodules

Note: lib/asf4 is not required - it resides in ports/saml21/asf4/.

3. Build firmware, examples:

    make BOOTLOADER=0 EXTRAS=0
    make BOOTLOADER=1 EXTRAS=0
    make BOARD=WLR089U0 BOOTLOADER=1 EXTRAS=0

Default board is WLR089U0, so BOARD is optional.
Note: make BOARD=saml21 should work, the firmware haven't been tested on ATSAML21 board. ATSAMR34/35 has no DAC perip.

Resulting binaries appear in:
    build-WLR089U0/

-------------------------------------------------------
Flashing the Firmware
---------------------

The WLR089U0 has no built-in UF2 bootloader.
Precompiled UF2 bootloader binaries are located at:

    ports/saml21/Firmwares

Original UF2 base:
    https://github.com/ladyada/uf2-samd21

Modified version for ATSAML21/ATSAMR34 (WLR089U0):
    https://github.com/TArnD-jg/uf2-samd21_bootloader_for_wlr089u0

Steps:

1. Flash the bootloader using MPLAB IPE v6.05 and MPLAB SNAP.
   - IPE: https://www.microchip.com/en-us/tools-resources/production/mplab-integrated-programming-environment
   - SNAP: https://www.microchip.com/en-us/development-tool/pg164100

2. Double-tap the reset button to enter UF2 bootloader mode.
   A USB drive (WLR089U0) will appear on your PC.

3. Copy the generated firmware.uf2 file to that drive.
   The board will automatically reboot once flashing is complete.

-------------------------------------------------------
Notes
-----

- Build tested with:
      gcc-arm-none-eabi-7-2018-q2-update

- P2P LoRa networking example in:
      boards/WLR089U0/examples/

Planned features:
- improved board documentation

Removed features:
- DHT driver

Features that won't work on WLR089U0, but can be re-enabled, see mpconfigmcu.h:
- AsyncIO
- Onewire
- Framebuf


-------------------------------------------------------
Disclaimer
----------

This is an experimental, non-official build.
It may contain bugs or incomplete features. Use at your own risk.

-------------------------------------------------------

License
-------

Licensed under the MIT License.
Based on the MicroPython project:
https://github.com/micropython/micropython
