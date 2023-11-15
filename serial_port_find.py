import sys
import glob
import serial


def serial_ports():
    # ONLY for windows, and only 1 port expect
    if sys.platform.startswith('win'):  # windows
        ports = ['COM%s' % (i + 1) for i in range(256)]
    else:
        raise EnvironmentError('NOT WINDOWS PLATFORM')

    for port in ports:  # i in COM1...COM256
        try:
            s = serial.Serial(port)
            s.close()
            result = port
        except (OSError, serial.SerialException):
            pass
    return result

if __name__ == '__main__':
    print(serial_ports())