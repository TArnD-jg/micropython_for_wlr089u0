-------------------------------------------------------
Firmware Variants & Memory Layout
-------------------------------------------------------

The bootloader occupies the first **24kB** of flash memory.
Therefore, the UF2 application **MUST start from 0x6000**.

### Build Configuration Matrix

**Crucial Note:** The `E0` (Extras Disabled) builds are **minimalistic builds**.
They disable standard MicroPython modules like `json`, `re`, `binascii`, `zlib`, and `random` to save space.
**All variants include the LoRa drivers.**

| Variant | Bootloader Offset | Feature Set | Start Address | Description |
| :--- | :--- | :--- | :--- | :--- |
| **B1_E1** | **Yes** | **Standard** | `0x6000` | **Recommended.** Full MicroPython (JSON, RE, etc.) + LoRa drivers. |
| **B1_E0** | Yes | Minimal | `0x6000` | Stripped build. **No JSON, RE, or ZLIB.** Includes LoRa drivers. |
| **B0_E1** | No | Standard | `0x0000` | For direct programmer flashing. Full MicroPython + LoRa drivers. |
| **B0_E0** | No | Minimal | `0x0000` | For direct programmer flashing. Minimal features + LoRa drivers. |

* **B1** = Bootloader enabled (Offset 0x6000)
* **B0** = Bootloader disabled (Offset 0x0)
* **E1** = **Standard Features enabled** (JSON, RE, Binascii, Random, Deflate, etc.)
* **E0** = **Minimal Features** (Core interpreter only, LoRa supported, but standard libraries removed)

### Memory Usage

The WLR089U0 has 256KB Flash. The `B1_E1` build is very close to the limit (~96% usage).
Users who need more space for their own `.py` files:
1. Use the **B1_E0** variant (minimal features).
2. Remove the pre-included LoRa drivers from the `modules` directory before compiling.
3. Customize the feature set by modifying `saml21/mcu/saml21/mpconfigmcu.h` and compile it.

| Build | Flash Used | Free Flash (approx) | RAM Used |
| :--- | :--- | :--- | :--- |
| **B1_E1** | 229,216 B (96.5%) | ~8 KB | 2,364 B |
| **B1_E0** | 199,728 B (84.1%) | ~38 KB | 2,356 B |

*(Note: B1 builds have 232KB available flash region due to the 24KB reserved for bootloader)*

-------------------------------------------------------
Directory Structure
-------------------------------------------------------

After compiling, the `Firmwares` directory will contain the following structure:

Firmwares
├── BOOTLOADER
│   ├── bootloader-WLR089U0-xxx.bin  (The bootloader itself)
│   └── ...
├── build-WLR089U0-B1-E1             (Target for UF2 Flashing)
│   ├── WLR089U0_B1_E1.uf2           <-- COPY THIS TO USB DRIVE
│   ├── WLR089U0_B1_E1.hex
│   └── ...
├── build-WLR089U0-B1-E0             (Target for UF2 with more free space)
│   └── ...
└── ... (B0 variants for SWD flashing)
