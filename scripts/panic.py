import serial

PORT = '/dev/ttyUSB1'
BAUDRATE = 921600
NUMBER_OF_BYTES = 2048

def main():
    with serial.Serial(PORT, BAUDRATE, rtscts=True) as ser:
        for i in range(0, NUMBER_OF_BYTES):
            ser.write(bytes([i % 256]))

if __name__ == "__main__":
    main()