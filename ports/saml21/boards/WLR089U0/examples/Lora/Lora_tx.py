import time
from machine import Pin, SPI
import gc
import cryptolib
import ubinascii

gc.collect()
# Turn on LoRa see ATSAMR34/35 DS.
RFSW_PWR = Pin('LORA_PWR', Pin.OUT)
RFSW_PWR.on()


KEY = b"321ztrgr3213asfg"    
IV  = b"1234567890ABCDEF"    



def pad_data(data, block_size=16):
    #PKCS7 padding
    padding_len = block_size - (len(data) % block_size)
    return data + bytes([padding_len] * padding_len)

def unpad_data(data):
    #PKCS7 unpadding
    pad_len = data[-1]
    if pad_len < 1 or pad_len > 16:
        raise ValueError("Invalid padding length")
    if data[-pad_len:] != bytes([pad_len] * pad_len):
        raise ValueError("Invalid padding bytes")
    return data[:-pad_len]

def encrypt_cbc(data):
    padded_data = pad_data(data)
    aes = cryptolib.aes(KEY, 2, IV)
    encrypted = aes.encrypt(padded_data)
    return encrypted

def decrypt_cbc(data):
    aes = cryptolib.aes(KEY, 2, IV)
    decrypted = aes.decrypt(data)
    return unpad_data(decrypted)

def encrypt_ecb(data):
    padded = pad_data(data)
    aes = cryptolib.aes(KEY, 1) 
    encrypted = aes.encrypt(padded)
    return encrypted

def decrypt_ecb(data):
    aes = cryptolib.aes(KEY, 1)
    decrypted = aes.decrypt(data)
    return unpad_data(decrypted)

def encrypt_ctr(data):
    aes = cryptolib.aes(KEY, 6, IV)
    encrypted = aes.encrypt(data)
    return encrypted

def decrypt_ctr(data):
    aes = cryptolib.aes(KEY, 6, IV)
    decrypted = aes.decrypt(data)
    return decrypted

def safe_decode(data):
    try:
        return data.decode()
    except:
        return ''.join(chr(b) if 32 <= b <= 126 else '.' for b in data)

def get_modem():
    from lora import SX1276
    lora_cfg = {
        "freq_khz": 868100,
        "sf": 9,
        "bw": "125",
        "coding_rate": 5,
        "preamble_len": 8,
        "output_power": 14,
        "implicit_header": False,
        "crc_en": True,
    }

    spi = SPI(4, baudrate=2_000_000, polarity=0, phase=0,
              sck=Pin("LORA_SCK"), mosi=Pin("LORA_MOSI"), miso=Pin("LORA_MISO"))

    modem = SX1276(spi=spi, cs=Pin("LORA_CS", Pin.OUT),
                   reset=Pin("LORA_nRST", Pin.OUT),
                   dio0=Pin("LORA_DIO0", Pin.IN),
                   lora_cfg=lora_cfg)
    return modem

def main():
    modem = get_modem()
    while True:
        msg = b"Hello World!"
        encrypted = encrypt_cbc(msg) 
        print(f"Encrypted (hex): {ubinascii.hexlify(encrypted)}")
        modem.send(encrypted)
        time.sleep(3600)

if __name__ == "__main__":
    main()


