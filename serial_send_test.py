import serial
import time

# arduino = serial.Serial(port='COM19',  baudrate=115200, timeout=.1)


# def write_read(x):
#     arduino.write(bytes(x,  'utf-8'))
#     time.sleep(0.05)
#     data = arduino.readline()
#     return  data

SerialObj = serial.Serial('COM20') # COMxx  format on Windows
                  # ttyUSBx format on Linux
SerialObj.baudrate = 115200  # set Baud rate to 9600
SerialObj.bytesize = 8   # Number of data bits = 8
SerialObj.parity  ='N'   # No parity
SerialObj.stopbits = 1   # Number of Stop bits = 1

while True
    time.sleep(1000)
    SerialObj.write(b'42')    #transmit 'A' (8bit) to micro/Arduino
    SerialObj.close()      # Close the port
# while True:
#     num = input("Enter a number: ")
#     value  = write_read(num)
#     print(value)



