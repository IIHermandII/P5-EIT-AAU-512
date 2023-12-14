"""
Embedded Python Blocks:

Each time this file is saved, GRC will instantiate the first class it finds
to get ports and parameters of your block. The arguments to init  will
be the parameters. All of them are required to have default values!
"""
from ctypes import cdll, c_int
import numpy as np
from gnuradio import gr
import uhd
import time
import serial
import sys
import pathlib
import os


class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block
    """Embedded Python Block example - a simple multiply const"""

    def serial_ports(self):
        # ONLY for windows, and only 1 port expect
        if sys.platform.startswith('win'):  # windows
            ports = ["COM%s" % (i + 1) for i in range(256)]
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

    def __init__(self, SampleRate=1, SignalFreq=1, Gain=1, Rx1_Phase_Cal=1, Rx2_Phase_Cal=1, Rx3_Phase_Cal=1, Rx4_Phase_Cal=1):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='Angle sweeper',  # will show up in GRC
            in_sig=[],
            out_sig=[np.complex64, np.float32]
        )
        # Initialize arduino communication
        # self.SerialObj = serial.Serial(self.serial_ports(), baudrate=115200, bytesize=8, parity='N', stopbits=1)

        # Setup off phaseshifters
        self.path = str(pathlib.Path().resolve()) + "\VNX_dps64.dll"
        print("--- Path ---")
        print(self.path)
        self.vnx = cdll.LoadLibrary(os.path.normpath(self.path))

        self.vnx.fnLPS_SetTestMode(False)  # Use actual devices
        self.DeviceIDArray = c_int * 20
        self.Devices = self.DeviceIDArray()  # This array will hold the list of device handles returned by the DLL

        print('------- Initialize Lab Bricks -------')
        self.numDevices = self.vnx.fnLPS_GetNumDevices()  # GetNumDevices will determine how many LPS devices are availible
        print(str(self.numDevices), ' device(s) found')

        # GetDevInfo generates a list, stored in the devices array, of
        # every availible LPS device attached to the system
        # GetDevInfo will return the number of device handles in the array
        dev_info = self.vnx.fnLPS_GetDevInfo(self.Devices)
        print('GetDevInfo returned', str(dev_info))
        self.ser_num = []
        # GetSerialNumber will return the devices serial number
        for i in range(self.numDevices):
            self.ser_num.append(self.vnx.fnLPS_GetSerialNumber(self.Devices[i]))
            print('Serial number:', str(self.ser_num[i]))
            init_dev = self.vnx.fnLPS_InitDevice(self.Devices[i])  # InitDevice will prepare the device for operation
            print('InitDevice returned', str(init_dev))
        print('---------- End initialize -----------')

        self.c = 3e8
        self.SignalFreq = SignalFreq  # input
        self.d = 1/2*(self.c/self.SignalFreq)  # d = (1/2) * (3e8/2.4e9)
        self.PhaseStep = 4
        self.PhaseValues = np.arange(-180, 184, self.PhaseStep)  # finds number of steps

        for i in range(self.numDevices):
            self.vnx.fnLPS_SetWorkingFrequency(self.Devices[i], int(self.SignalFreq / 100000))

        self.lookUp = {}

        for i in range(self.numDevices):
            match self.ser_num[i]:
                case 24780:
                    self.lookUp.update({0: self.Devices[i]})
                case 24781:
                    self.lookUp.update({1: self.Devices[i]})
                case 24783:
                    self.lookUp.update({2: self.Devices[i]})
                case 24784:
                    self.lookUp.update({3: self.Devices[i]})

        self.sdr = uhd.usrp.MultiUSRP("type=b200")
        # self.sdr.set_rx_agc(True, 0) # 0 for channel 0, i.e. the first channel of the USRP

        f = open("data.txt", "w")
        f.write("Data output:" + '\n')
        f.close()

        # Variables
        self.Rx1_Phase_Cal = Rx1_Phase_Cal  # phase off set
        self.Rx2_Phase_Cal = Rx2_Phase_Cal  # phase off set
        self.Rx3_Phase_Cal = Rx3_Phase_Cal  # phase off set
        self.Rx4_Phase_Cal = Rx4_Phase_Cal  # phase off set
        self.SampleRate = SampleRate  # input
        self.numSamples = 1024
        self.Gain = Gain  # input
        self.slice_value = 1
        self.threshold = -25

    def find_local_maxima(self, arr):
        local_maxima = []
        avg_arr = self.moving_average(arr, 3)
        for i in range(2, len(avg_arr) - 2):
            if avg_arr[i][0] > avg_arr[i - 1][0] and avg_arr[i][0] > avg_arr[i + 1][0]:
                local_maxima.append(avg_arr[i][1])
        return local_maxima

    def moving_average(self, arr, window_size):
        i = 0
        moving_averages = []
        while i < len(arr) - window_size + 1:
            window = []
            for j in range(window_size):
                window.append(arr[i+j][0])

            window_average = round(sum(window) / window_size, 2)  # Calculate the average of current window

            moving_averages.append([window_average, arr[i][1]])
            i += 1
        return moving_averages

    def set_phase_shifters(self, PhDelta):
        RxPhase1 = (PhDelta*0+self.Rx1_Phase_Cal) % 360
        RxPhase2 = (PhDelta*1+self.Rx2_Phase_Cal) % 360
        RxPhase3 = (PhDelta*2+self.Rx3_Phase_Cal) % 360
        RxPhase4 = (PhDelta*3+self.Rx4_Phase_Cal) % 360  # the phase shift + offset % 360
        self.vnx.fnLPS_SetPhaseAngle(self.lookUp[0], int(RxPhase1))
        self.vnx.fnLPS_SetPhaseAngle(self.lookUp[1], int(RxPhase2))
        self.vnx.fnLPS_SetPhaseAngle(self.lookUp[2], int(RxPhase3))
        self.vnx.fnLPS_SetPhaseAngle(self.lookUp[3], int(RxPhase4))  # phaseshifter, phase

    def calculate_stearing_angle(self, PhDelta):
        value1 = (self.c * np.radians(np.abs(PhDelta)))/(2*np.pi*self.SignalFreq*self.d)
        clamped_value1 = max(min(1, value1), -1)
        theta = np.degrees(np.arcsin(clamped_value1))

        error_func = 0.0000002045*theta**4+0.00295*theta**2+2  # the theta does not macth theta irl for that error_func
        theta -= error_func
        if PhDelta >= 0:
            Steer_angle = theta   # positive PhaseDelta covers 0deg to 90 deg
        else:
            Steer_angle = -theta   # negative phase delta covers 0 deg to -90 deg
        return Steer_angle

    def fft(self, win):
        # Get samples of data
        data = self.sdr.recv_num_samps(
            self.numSamples,  # Number of samples
            self.SignalFreq,  # Frequency in Hz
            self.SampleRate,  # Sampling rate
            [0],  # Receive on channel 0
            self.Gain,  # dB of RX gain
        )

        y = data * win
        sp = np.absolute(np.fft.fft(y, self.numSamples))
        return sp

    def multiple_sorces(self, sp_shifted):
        index = 0
        peaks = []
        for spec in sp_shifted:
            index += 1
            if 20*np.log10(spec) > self.threshold:
                # freq=((index+slice_value)*(self.SampleRate/self.numSamples)-self.SampleRate/2+self.SignalFreq)/pow(10,9)
                amp = round(20*np.log10(spec), 2)
                peaks.append([amp, index])

        local_max = self.find_local_maxima(peaks)
        return local_max

    def work(self, input_items, output_items):
        max_signal = -10000
        PhaseStepNumber = 0
        win = np.hamming(self.numSamples)  # window function
        tSweep = time.time()
        collected_max = []

        for PhDelta in self.PhaseValues:  # every tick in the -180 -> 184
            self.set_phase_shifters(PhDelta)
            steer_angle = self.calculate_stearing_angle(PhDelta)
            sp = self.fft(win)
            PeakValue = 20*np.log10(max(sp[0]))
            sp_shifted = np.fft.fftshift(sp[0][self.slice_value:-self.slice_value])
            local_max = self.multiple_sorces(sp_shifted)
            if len(local_max) != 0:
                collected_max.append([PhaseStepNumber, local_max])

            if PeakValue > max_signal:
                max_signal = PeakValue
                max_angle = steer_angle

            output_items[0][PhaseStepNumber] = ((1)*steer_angle + (1j * PeakValue))
            PhaseStepNumber = PhaseStepNumber + 1

        f = open("data.txt", "a")
        f.write("Collected_max: " + str(collected_max) + '\n')
        f.write("Sweep time: " + str((time.time()-tSweep)*1000) + "ms" + '\n')
        f.close()

        print("Sweep time: ", (time.time()-tSweep)*1000, "ms")

        # self.SerialObj.write(bytes(str(round(max_angle)), "utf-8"))  # Transmit input to Arduino

        output_items[0] = output_items[0][0:PhaseStepNumber]
        output_items[1][:] = max_angle
        return len(output_items[0])
