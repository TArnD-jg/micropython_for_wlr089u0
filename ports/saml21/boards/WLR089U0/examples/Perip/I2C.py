# Import I2C and Pin libraries
from machine import I2C, Pin

# Initialize I2C peripheral
# Hardware I2C ID: 1, SCL: PA17, SDA: PA16, Frequency: 400kHz
i2c = I2C(1, scl=Pin("PA17"), sda=Pin("PA16"), freq=400_000)

# Scan for connected I2C devices
print("Scanning I2C bus...")
devices = i2c.scan()

# Print the list of found addresses in Hexadecimal format
print("Devices found:", list(map(hex, devices)))