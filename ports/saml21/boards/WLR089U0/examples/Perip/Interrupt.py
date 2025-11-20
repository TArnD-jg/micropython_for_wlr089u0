from machine import Pin
import time

# Global counter variable
counter = 0

# Define the Interrupt Service Routine (ISR) callback function
# Ideally, software debouncing should be handled here or in the main loop.
def button_pressed(pin):
    global counter
    counter += 1
    print(f"\nHello World! Count: {counter}")

# Initialize push button on pin PA18
# Set as Input with internal Pull-Up resistor enabled
# (Button connects PA18 to GND)
button = Pin("PA18", Pin.IN, Pin.PULL_UP)

# Initialize Interrupt
# Trigger event: Falling edge (High -> Low transition when pressed)
button.irq(trigger=Pin.IRQ_FALLING, handler=button_pressed)

# Main loop to keep the script running
print("Waiting for button press on PA18...")

while True:
    # Do nothing, just sleep to save power while waiting for interrupt
    time.sleep(1)