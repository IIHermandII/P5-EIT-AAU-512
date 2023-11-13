import time 
import serial

SerialObj = serial.Serial('COM20', baudrate=115200, bytesize=8, parity='N', stopbits=1)
SerialObj.close()

for i in range(1,10):
    SerialObj = serial.Serial('COM20', baudrate=115200, bytesize=8, parity='N', stopbits=1)
    
    # input()
    time.sleep(2)
    SerialObj.write(bytes(str(4), "utf-8"))  # Transmit input to Arduino
    print("just sendt a 4")
    time.sleep(2)  # Optional delay to ensure data is sent before proceeding
    SerialObj.close()

# Close the serial port when done (this line will not be reached in the current script)
