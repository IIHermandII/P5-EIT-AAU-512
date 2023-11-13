"""
Embedded Python Blocks:

Each time this file is saved, GRC will instantiate the first class it finds
to get ports and parameters of your block. The arguments to _init_  will
be the parameters. All of them are required to have default values!
"""
from ctypes import *
import numpy as np
from gnuradio import gr
import uhd
import time 
import serial
"""
TO TRY:
-Ny USB HUB / gav 1ms
-Skrive til en enkelt enhed / stadig langsomt
-Undersøge om der er et usb handle som fucker up - det virker jo til set frequency og så ikke til set phase - og phase er endda en mindre besked end frequency / PAS
-Sende besked sammen med sæt working frequency / gav 10ms delay pr. besked
-Sæt working frequency sammen i main loopet / 40ms delay - 10 pr enhed
-Prøv at lade streamhandle ligge i Multi klassen og kun generere et nyt hvis det et None
-Evt. skær ned på buffer for at det ikke tager længere tid end højst nødvendigt i Recv_num
"""

class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block
    """Embedded Python Block example - a simple multiply const"""

    def __init__(self, SampleRate = 1, SignalFreq=1, Gain=1, Rx1_Phase_Cal=1, Rx2_Phase_Cal=1, Rx3_Phase_Cal=1, Rx4_Phase_Cal=1):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='Angle sweeper',  # will show up in GRC
            in_sig=[],
            out_sig=[np.complex64, np.float32]
        )
        # if an attribute with the same name as a parameter is found,
        # a callback is registered (properties work, too).
        self.vnx = cdll.LoadLibrary(r"C:\Users\asbjo\OneDrive\Dokumenter\AAU\Asbjørns_gnu\VNX_dps64.dll")
        self.vnx.fnLPS_SetTestMode(False)  # Use actual devices
        self.DeviceIDArray = c_int * 20
        self.Devices = self.DeviceIDArray()  # This array will hold the list of device handles returned by the DLL

        print('------- Initialize Lab Bricks -------')
        self.numDevices = self.vnx.fnLPS_GetNumDevices() # GetNumDevices will determine how many LPS devices are availible
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
            init_dev = self.vnx.fnLPS_InitDevice(self.Devices[i])  #InitDevice will prepare the device for operation
            print('InitDevice returned', str(init_dev))
        print('---------- End initialize -----------')
        
        self.SampleRate = SampleRate #input
        self.numSamples = 1024
        self.Gain = Gain #input

        self.c = 3e8
        self.SignalFreq = SignalFreq #input
        self.d = 1/2*(self.c/self.SignalFreq) # d = (1/2) * (3e8/2.4e9)

        self.sdr = uhd.usrp.MultiUSRP("type=b200")

        self.Rx1_Phase_Cal = Rx1_Phase_Cal #phase off set
        self.Rx2_Phase_Cal = Rx2_Phase_Cal #phase off set
        self.Rx3_Phase_Cal = Rx3_Phase_Cal #phase off set
        self.Rx4_Phase_Cal = Rx4_Phase_Cal #phase off set

        self.PhaseStep = 4
        self.PhaseValues = np.arange(-180, 184, self.PhaseStep) #finds number of steps

        for i in range(self.numDevices):
            #HER ER DER 2ms
            #print("For loop", time.time())
            self.vnx.fnLPS_SetWorkingFrequency(self.Devices[i], int(self.SignalFreq/ 100000))
        
        self.lookUp = {}
        
        for i in range(self.numDevices):
            match self.ser_num[i]:
                case 24780:
                    self.lookUp.update({0 : self.Devices[i]})
                case 24781:
                    self.lookUp.update({1 : self.Devices[i]})
                case 24783:
                    self.lookUp.update({2 : self.Devices[i]})  
                case 24784: 
                    self.lookUp.update({3 : self.Devices[i]})
        
        #self.data = np.zeros((100),dtype=np.complex64)

    def work(self, input_items, output_items):
        """example: multiply with constant"""        
        max_signal = -10000
        max_angle = 0

        win = np.hamming(self.numSamples) #window function

        PhaseStepNumber = 0 
        tSweep = time.time()
        

        for PhDelta in self.PhaseValues: #evry tick in the -180 -> 184

            # RxPhase1 = (PhDelta*0+self.Rx1_Phase_Cal) % 360
            # RxPhase2 = (PhDelta*1+self.Rx2_Phase_Cal) % 360
            # RxPhase3 = (PhDelta*2+self.Rx3_Phase_Cal) % 360
            RxPhase4 = (PhDelta*3+self.Rx4_Phase_Cal) % 360 #the phase shift + offset % 360
            # self.vnx.fnLPS_SetPhaseAngle(self.lookUp[0], int(RxPhase1))
            # self.vnx.fnLPS_SetPhaseAngle(self.lookUp[1], int(RxPhase2))
            # self.vnx.fnLPS_SetPhaseAngle(self.lookUp[2], int(RxPhase3))
            tPhases = time.time()
            self.vnx.fnLPS_SetPhaseAngle(self.lookUp[3], int(RxPhase4)) #phaseshifter, phase
            # print("PHASESHIFTER USB time: ",(time.time()-tPhases)*1000, "ms")
            
            value1 = (self.c * np.radians(np.abs(PhDelta)))/(2*np.pi*self.SignalFreq*self.d) 
            clamped_value1 = max(min(1, value1), -1)     
            theta = np.degrees(np.arcsin(clamped_value1))

            error_func=0.0000002045*theta**4+0.00295*theta**2+2 #the theta does not macth theta irl for that error_func
            theta -= error_func
            if PhDelta>=0:
                SteerAngle = theta   # positive PhaseDelta covers 0deg to 90 deg
            else:
                SteerAngle = -theta   # negative phase delta covers 0 deg to -90 deg

            #print("Pre sample: ",time.time())

            tSample = time.time()
            #Get samples of data
            data = self.sdr.recv_num_samps(
            self.numSamples, # Number of samples
            self.SignalFreq, # Frequency in Hz
            self.SampleRate, # Sampling rate
            [0], # Receive on channel 0
            self.Gain, # dB of RX gain
            )
            
            # print("SDR TOTAL Sample time: ",(time.time()-tSample)*1000, "ms")

            y = data * win
            sp = np.absolute(np.fft.fft(y,1024))
            PeakValue = 20*np.log10(max(sp[0]))

            if PeakValue > max_signal:
                max_signal = PeakValue
                max_angle = SteerAngle

            output_items[0][PhaseStepNumber] = ((1)*SteerAngle + (1j * PeakValue))
            PhaseStepNumber = PhaseStepNumber + 1
                 
        print("Sweep time: ",(time.time()-tSweep)*1000, "ms")  
        # serial_arduino = serial.Serial('COM20',baudrate=115200,bytesize=8,parity='N',stopbits=1)
        # time.sleep(2)
        # serial_arduino.write(bytes(str(4),"utf-8")) #needs "" insted of '' do to python mecanics

        SerialObj = serial.Serial('COM20', baudrate=115200, bytesize=8, parity='N', stopbits=1)
    
        # input()
        time.sleep(1.6)
        SerialObj.write(bytes(str(180), "utf-8"))  # Transmit input to Arduino
        print("just sendt a 4")
        time.sleep(0.4)  # Optional delay to ensure data is sent before proceeding
        # SerialObj.close()


        output_items[0] = output_items[0][0:PhaseStepNumber]
        output_items[1][:] = max_angle
        return len(output_items[0])