import machine
import ubinascii

WORD_0_ADDR = 0x0080A00C
WORD_1_ADDR = 0x0080A040
WORD_2_ADDR = 0x0080A044
WORD_3_ADDR = 0x0080A048

word_0 = machine.mem32[WORD_0_ADDR]
word_1 = machine.mem32[WORD_1_ADDR]
word_2 = machine.mem32[WORD_2_ADDR]
word_3 = machine.mem32[WORD_3_ADDR]

print(f"#Word 0: {word_0:08X}")
print(f"#Word 1: {word_1:08X}")
print(f"#Word 2: {word_2:08X}")
print(f"#Word 3: {word_3:08X}")

serial_number = (word_0 << 96) | (word_1 << 64) | (word_2 << 32) | word_3
print(f"#Serial Number: {serial_number:032X}")
unique_id_hex = ubinascii.hexlify(machine.unique_id()).decode()
print(f"#Unique ID: {unique_id_hex}")