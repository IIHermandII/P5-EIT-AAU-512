import time 
import serial

SerialObj = serial.Serial('COM19', baudrate=115200, bytesize=8, parity='N', stopbits=1)
time.sleep(2)

while True:
    input_string=input("Please input an int between -90 to 90 (int)")
    SerialObj.write(bytes(str(input_string), "utf-8"))  # Transmit input to Arduino
    print("just sendt a: ", input_string)
    if input_string == "B":
        break

print("breake!")
# Close the serial port when done (this line will not be reached in the current script)
