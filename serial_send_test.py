import serial
import time

SerialObj = serial.Serial('COM20', baudrate=115200, bytesize=8, parity='N', stopbits=1)

while True:
    num = input("Enter a number: ")
    SerialObj.write(bytes(num, 'utf-8'))  # Transmit input to Arduino
    time.sleep(0.1)  # Optional delay to ensure data is sent before proceeding

# Close the serial port when done (this line will not be reached in the current script)
SerialObj.close()