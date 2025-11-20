Lora/modules/
Contains the WLR089U0-specific SX127x driver and _boot.py.
Copy the entire directory into the root of the saml21 port before compiling.

Lora_rx.py 
Lora_tx.py 
Simple example scripts demonstrating how to create a 1:1 LoRa P2P link using cryptolib.

Warning:
This is a modified and experimental version of the MicroPython LoRa library.
It is designed specifically for the WLR089U0 and may not work on other boards.
Use at your own risk.
Ensure that LoRa transmission parameters comply with local radio regulations.

LoRa SPI Interface (SPI-COM4):
LORA_MOSI  PB30
LORA_CS    PB31
LORA_SCK   PC18
LORA_MISO  PC19

LoRa DIO Pins:
LORA_DIO0  PB16
LORA_DIO1  PA11
LORA_DIO2  PA12
LORA_DIO3  PB17
LORA_DIO4  PA10
LORA_DIO5  PB00

Other Pins:
LORA_nRST  PB15
LORA_PWR   PA09

