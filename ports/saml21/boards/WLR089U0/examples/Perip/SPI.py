# Import Serial Peripheral Interface (SPI) module and GPIO library
from machine import Pin, SPI

# WLR089U0 - LoRa TxRx, Initialize CS (Chip Select) pin as output
cs = Pin('LORA_CS', Pin.OUT)

# Set CS pin HIGH (Deselect the chip)
cs.value(1)

# Initialize SPI object (constructor)
# Note: Baudrate is set to 100kHz
spi = SPI(4, sck=Pin("LORA_SCK"), mosi=Pin("LORA_MOSI"), miso=Pin("LORA_MISO"), baudrate=100000)

# Loop to read registers from address 0x00 up to 0x09
for i in range(0, 10):
    # Set CS pin LOW (Select the chip to start communication)
    cs.value(0)
    
    # Send register address to read
    # (Bitwise AND with 0x7F ensures the MSB is 0, indicating a read operation)
    spi.write(bytearray([i & 0x7F]))
    
    # Receive 1 byte of data from the register
    data = spi.read(1)
    
    # Convert received bytes to hex string format
    hex_data = ' '.join(['%02X' % byte for byte in data])
    
    # Set CS pin HIGH (Deselect the chip to end communication)
    cs.value(1)
    
    # Print register address and read value
    print("Register address: " + str(hex(i)) + " value: 0x" + hex_data)