import serial

# Open the serial port
ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0)

# Check if the serial port is open
if ser.is_open:
    print("Serial port is open")

try:
    while True:
        # Read a line from the serial port
        line = ser.readline()
        if line:
            print(f"Received: {line}")
except KeyboardInterrupt:
    print("Exiting...")

finally:
    # Close the serial port
    ser.close()
    print("Serial port is closed")

