import serial
from rscp_2 import receiver

ser = serial.Serial("/dev/pts/6", 115200, timeout=None)
# ser1 = serial.Serial("/dev/pts/5", 9600, timeout=None)

tx=bytearray()
if ser.is_open:
    print(f"Connected to {ser.name}")
    

try:
    hi=receiver(ser)
    # writedata=b'~\t\t@I\x0eVgreen,+'
    # ser1.write(writedata)
    while True:
        # Read data from the COM port with a timeout
        data = ser.read()  
        if data:
            print(f"Received: {data}")
            hi.test_parser(data)
        else:
            print("No data received within the timeout period.")
        print(tx)
except KeyboardInterrupt:
    print("Exiting...")

# Close the port
ser.close()
