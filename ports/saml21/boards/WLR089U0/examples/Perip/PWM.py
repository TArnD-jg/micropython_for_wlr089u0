# Import I/O and PWM peripheral libraries
from machine import Pin, PWM
import time

# PWM initialization on pin PA05
# Frequency: 200Hz, Initial Duty Cycle: 0
pwm = PWM(Pin('PA05'), freq=200, duty_u16=0)

# Helper variable for duty cycle
i = 0

while True:
    # Set duty cycle (16-bit integer: 0-65535)
    pwm.duty_u16(i)
    
    # Increment duty cycle
    i = i + 1000
    
    # Reset if counter exceeds the 16-bit range limit (approx 65000)
    if i > 65000:
        i = 0
        
    # Small delay to make the fading visible
    time.sleep(0.01)