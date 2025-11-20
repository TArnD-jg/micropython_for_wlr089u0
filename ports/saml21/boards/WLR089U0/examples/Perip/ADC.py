# Import ADC and GPIO libraries
from machine import ADC, Pin
import time

# Initialize ADC on pin PA08
adc0 = ADC(Pin('PA08'))

print("ADC reading started on PA08...")

while True:
    # Read 16-bit value (0 - 65535)
    # Note: The hardware resolution is 12-bit, but MicroPython scales it to 16-bit.
    value = adc0.read_u16()
    print(f"Raw value: {value}")
    
    # Delay 1 second
    time.sleep(1)

# ---------------------------------------------------------
# TECHNICAL NOTES (Architecture Differences):
# ---------------------------------------------------------
# There are significant differences between ATSAML21 and ATSAMD21/D51 families
# regarding ADC Voltage Reference (VREF).
#
# Consult:
# https://docs.micropython.org/en/latest/samd/quickref.html#adc-analog-to-digital-conversion
#
# SAM L21 Family Data Sheet p33. and p972.
# Internal mapping from C-source (adc_vref_table):
#   0: INTREF (1.0V)
#   1: INTVCC0 (1/1.48 VDDANA)
#   2: INTVCC1 (1/2 VDDANA)
#   3: AREFA (External)
#   4: AREFB (External)
#   5: INTVCC2 (VDDANA) <--- DEFAULT_ADC_VREF (5)
#
# By default, VREF is set to VDDANA (usually 3.3V).
# ---------------------------------------------------------