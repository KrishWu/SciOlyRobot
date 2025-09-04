import serial
import time

# Attempt to connect to the serial port
connected = False
while not connected:
    try:
        ser = serial.Serial('/dev/cu.usbmodem101', 115200)  # Replace 'COM3' with your Arduino's port
        connected = True
    except serial.SerialException:
        print("Failed to connect to serial port. Retrying...")

while True:
    # ser.flushInput()  # Clear the serial input buffer
    # time.sleep(0.001)
    line = ser.readline().decode('utf-8').strip()
    print("Serial Data:", line)  # Debugging print
