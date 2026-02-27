import serial
import time

# ---------------- CONFIG ----------------
PORT = "COM15"
BAUDRATE = 57600
TIMEOUT = 0.1

# ---------------- INIT UART (8N2) ----------------
try:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,     # 8 data bits
        parity=serial.PARITY_NONE,     # No parity
        stopbits=serial.STOPBITS_TWO,  # 2 stop bits  ← 8N2
        timeout=TIMEOUT
    )
    print(f"\nCR95HF EMULATOR Started on {PORT} @ {BAUDRATE} (8N2)\n")
except Exception as e:
    print("UART Open Error:", e)
    exit()

# ---------------- RESPONSE LOGIC ----------------
def get_cr95hf_response(rx):

    # ECHO
    if rx == b'\x55':
        return b'\x55'

    # PROTOCOL SELECT ISO14443A
    elif rx.startswith(b'\x02'):
        return b'\x00\x00'

    # RF ON
    elif rx.startswith(b'\x09'):
        return b'\x82\x00'

    # REQA
    elif b'\x26' in rx:
        return b'\x80\x04\x00\x04'

    # ANTICOLLISION CL1
    elif b'\x93\x20' in rx:
        return b'\x80\x08\xDE\xAD\xBE\xEF\x12'

    # SELECT CL1
    elif b'\x93\x70' in rx:
        return b'\x80\x04'

    # ANTICOLLISION CL2
    elif b'\x95\x20' in rx:
        return b'\x80\x08\x11\x22\x33\x44\x55'

    # SELECT CL2
    elif b'\x95\x70' in rx:
        return b'\x80\x00'

    # AUTH (Key A)
    elif b'\x60' in rx:
        return b'\x80\x00'

    # READ BLOCK
    elif b'\x30' in rx:
        return b'\x80' + bytes([0x11]*16)

    else:
        return b'\x00'

# ---------------- MAIN LOOP ----------------
while True:
    try:
        rx = ser.read(ser.in_waiting or 1)

        if rx:
            print("RX from STM32 :", rx.hex(' ').upper())

            response = get_cr95hf_response(rx)

            time.sleep(0.05)
            ser.write(response)

            print("TX to STM32   :", response.hex(' ').upper())
            print("-" * 50)

    except KeyboardInterrupt:
        print("\nStopping Emulator...")
        ser.close()
        break