import time

a=time.ticks_ms()
time.sleep_ms(1000)
b=time.ticks_ms()
c=b-a
print("Sleep 1000ms: "+ str(c) + " ms")

a=time.ticks_us()
time.sleep(1)
b=time.ticks_us()
c=b-a
print("Sleep 1s: "+ str(c) + " us")

a=time.ticks_us()
time.sleep_us(1000_000)
b=time.ticks_us()
c=b-a
print("Sleep 1*10^6us: "+ str(c) + " us")
# There are small differences between the measured times, but they are negligible.
# These variations exist because the sleep modes rely on different 
# internal timers and mechanisms.
# Sleep 1000ms: 997 ms
# Sleep 1s: 996391 us
# Sleep 1*10^6us: 1000077 us
# Sleep 1000ms: 997 ms
# Sleep 1s: 996399 us
# Sleep 1*10^6us: 1000078 us