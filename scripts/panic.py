import serial
import time

PORT = '/dev/ttyUSB1'
BAUDRATE = 921600
NUMBER_OF_BYTES = 2048
FAST_PANIC = False

def main():
    with serial.Serial(PORT, BAUDRATE, rtscts=True) as ser:
        if FAST_PANIC:
            print(f"Writing {NUMBER_OF_BYTES} bytes")
            for i in range(0, NUMBER_OF_BYTES):
                ser.write(bytes([i % 256]))

        else: 
            print("Writing 256 bytes")
            for i in range(0, 256):
                ser.write(bytes([i % 256]))

            for i in range(0, NUMBER_OF_BYTES):
                print(f"Sleep and write byte {i}")
                time.sleep(1)
                ser.write(bytes([i % 256]))

        

if __name__ == "__main__":
    main()