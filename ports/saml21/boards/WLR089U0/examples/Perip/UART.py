from machine import UART, Pin
import time

# Initialize UART
# ID: 0, TX pin: PA04, RX pin: PA05, Baudrate: 115200
uart = UART(0, tx=Pin('PA04'), rx=Pin('PA05'), baudrate=115200)

# Send a welcome message
print("Sending 'hello' via UART...")
uart.write('hello\n\r')

while True:
    # Check if there is any data waiting in the buffer
    if uart.any():
        # Read up to 5 bytes (or whatever is available)
        data = uart.read(5)
        
        if data:
            print(f"Received: {data}")
            
    time.sleep(1)