from machine import WDT, Pin
import time

StartTime = time.time()
#Init LED Pin
LED = Pin("PA18", Pin.OUT)
#Safety delay before initializing WDT
time.sleep(3)
try:
    #WDT init
    #Change accordingly to the table
    #wdt = WDT(timeout=4095)
    wdt = WDT(timeout=4096)
except Exception as e:
    raise e
while True:
    #Feed WDT
    wdt.feed()
    LED.toggle()
    #Change accordingly to the table
    time.sleep_ms(3000)
    #time.sleep(4)
    PassedTime = time.time() - StartTime
    print(f"{PassedTime}")

# WLR089U0 Watchdog Timer (WDT) Timeout Mapping
# -------------------------------------------------------
# The SAML21 hardware WDT rounds down to the nearest power of 2.
# See at machine_wdt.c
# Always leave a safety margin for clock drift and interpreter overhead.
# 
# | Python Timeout (ms) | Actual Hardware WDT   | Safe time.sleep() |
# | :------------------ | :-------------------- | :---------------- |
# | 1000 - 2047         | 1 sec  (1024 ms)      | max 0.8 sec       |
# | 2048 - 4095         | 2 sec  (2048 ms)      | max 1.5 sec       |
# | 4096 - 8191         | 4 sec  (4096 ms)      | max 3.0 sec       |
# | 8192 - 16383        | 8 sec  (8192 ms)      | max 6.0 sec       |
# | >= 16384            | 16 sec (16384 ms)     | max 14.0 sec      |
# 
# NOTE: Hardware cap is 16384ms. Values >16384 are clamped to 16s.
# Change WDT and/or time.sleep values to check the validity of the table