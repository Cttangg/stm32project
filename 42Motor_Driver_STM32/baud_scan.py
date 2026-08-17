"""COM port baud rate scanner — extended low range"""
import serial
import time

targets = [600,1200,1800,2400,3600,4800,7200,9600,14400,19200,28800,38400,57600,115200]

print(f"{'Baud':>8}  {'Bytes':>5}  Data")
print("-" * 60)

for baud in targets:
    try:
        s = serial.Serial('COM8', baudrate=baud, timeout=1.2)
        time.sleep(0.3)
        s.reset_input_buffer()
        data = s.read(128)
        s.close()
        # Find printable patterns
        printable = ''.join(chr(b) if 32 <= b < 127 else '.' for b in data)
        # Also check for 'H' or 'e' or 'l'
        has_hello = any(b in data for b in b'Hello')
        marker = ' <-- CLEAN?' if has_hello else ''
        print(f"{baud:>8}  {len(data):>5}  {printable[:50]}{marker}")
    except Exception as e:
        print(f"{baud:>8}  {0:>5}  ERR: {e}")
