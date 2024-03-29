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
import sys
import pathlib
import os
from datetime import datetime 
"""
TO TRY:
-Ny USB HUB / gav 1ms
-Skrive til en enkelt enhed / stadig langsomt
-Undersøge om der er et usb handle som fucker up - det virker jo til set frequency og så ikke til set phase - og phase er endda en mindre besked end frequency / PAS
-Sende besked sammen med sæt working frequency / gav 10ms delay pr. besked
-Sæt working frequency sammen i main loopet / 40ms delay - 10 pr enhed
-Prøv at lade streamhandle ligge i Multi klassen og kun generere et nyt hvis det et None
-Evt. skær ned på buffer for at det ikke tager længere tid end højst nødvendigt i Recv_num
m
"""

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
    
    def __init__(self, SampleRate = 1, SignalFreq=1, Gain=1, Rx1_Phase_Cal=1, Rx2_Phase_Cal=1, Rx3_Phase_Cal=1, Rx4_Phase_Cal=1,Bartlett_Threshold=1):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='Angle sweeper',  # will show up in GRC
            in_sig=[],
            out_sig=[np.complex64, np.float32,np.complex64,np.float32,np.complex64,np.float32]
        )
        # if an attribute with the same name as a parameter is found,
        # a callback is registered (properties work, too).
        # self.vnx = cdll.LoadLibrary(r"C:\Users\asbjo\OneDrive\Dokumenter\AAU\Asbjørns_gnu\VNX_dps64.dll")
        
        # self.SerialObj = serial.Serial(self.serial_ports(), baudrate=115200, bytesize=8, parity='N', stopbits=1)
        
        self.numSamples = 51
        self.N=self.numSamples
        

        self.path = str(pathlib.Path().resolve())+"\VNX_dps64.dll"
        #print("#####################path")
        #print(self.path)
        #r"C:\Users\emill\OneDrive - Aalborg Universitet\GIT2\P5-EIT-AAU-512\SDR_testing\VNX_dps64.dll"
        self.vnx = cdll.LoadLibrary(r'C:\Users\Jacob Elberg Nielsen\Aalborg Universitet\EI3 gruppe 1 - Dokumenter\5. semester\GNU\Manual phase control\VNX_dps64.dll') #
        
        
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
        self.Gain = Gain #input

        self.c = 3e8
        self.SignalFreq = SignalFreq #input
        self.d = 1/2*(self.c/self.SignalFreq) # d = (1/2) * (3e8/2.4e9)

        self.sdr = uhd.usrp.MultiUSRP("type=b200")

        self.Rx1_Phase_Cal = Rx1_Phase_Cal #phase off set
        self.Rx2_Phase_Cal = Rx2_Phase_Cal #phase off set
        self.Rx3_Phase_Cal = Rx3_Phase_Cal #phase off set
        self.Rx4_Phase_Cal = Rx4_Phase_Cal #phase off set

        self.Bartlett_Threshold = Bartlett_Threshold #The threshold for whether a signal is present

        self.SteerStep = 2
        self.SteeringAngles = np.arange(-50,51,self.SteerStep)# array of [-50,-48....48,50] with lenght 51
        self.PhaseValues= (np.pi * np.sin((self.SteeringAngles*2*np.pi)/(360))*360)/(2*np.pi) # same size array, but now the necessary linear phaseshift as a decimal number

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
        
        self.freqVal = np.linspace(-self.SampleRate/2+self.SampleRate/self.numSamples,self.SampleRate/2,self.numSamples) 
        self.SliceValue=1
        self.MVF1 = int(69/2) #Most Valuable Frequency, just a random number for the first time the code runs
        self.MVF2 = int(69/4)
        self.averageNumber = 5
        self.averageArray = np.zeros(self.averageNumber)
        self.SignalNumbers = 0 #default amount of signals

        #Write to file
        self.angles = []
        # Making .txt file in data_folder with name = Steering_angle + date and time
        current_folder = os.getcwd()
        text_file_position = os.path.join(current_folder, "data_folder")
        if not os.path.exists(text_file_position):
            os.makedirs(text_file_position)
        date_time = datetime.now().strftime("%Y %m %d %H %M %S")
        self.text_file = os.path.join(text_file_position, f"Steering_angles_{date_time}.txt")
        print(self.text_file)
        with open(os.path.normpath(self.text_file), "w") as f:
            f.write("")
        f.close()
        self.first_run = True
        self.doa_text_file = os.path.join(text_file_position,f"DOA_{date_time}.txt")
        with open(os.path.normpath(self.doa_text_file), "w") as f:
            f.write("")
        f.close()

        #Set phase for phaseshifter 1, as it does not change
        #RxPhase1 = 0
        #self.vnx.fnLPS_SetPhaseAngle(self.lookUp[0], int(RxPhase1))

        


    


    def work(self, input_items, output_items):
        """example: multiply with constant"""        
        max_signal1 = -10000
        max_angle1 = 0
        max_signal2 = -10000
        max_angle2 = 0

        win = np.hamming(self.numSamples) #window function

        SteerStepNumber = 0 
        tSweep = time.time()
        bartlett = np.zeros(self.numSamples,dtype=np.float32)
        sp = np.zeros(self.numSamples,dtype=np.float32)
        

        for PhDelta in self.PhaseValues: #evry tick in the steering angles
            RxPhase2 = (np.round(PhDelta*1)+self.Rx2_Phase_Cal) % 360
            RxPhase3 = (np.round(PhDelta*2)+self.Rx3_Phase_Cal) % 360
            RxPhase4 = (np.round(PhDelta*3)+self.Rx4_Phase_Cal) % 360 #the phase shift + offset % 360
            self.vnx.fnLPS_SetPhaseAngle(self.lookUp[1], int(RxPhase2))
            self.vnx.fnLPS_SetPhaseAngle(self.lookUp[2], int(RxPhase3))
            self.vnx.fnLPS_SetPhaseAngle(self.lookUp[3], int(RxPhase4)) #phaseshifter, phase
            #tPhases = time.time()
            #print("PHASESHIFTER USB time: ",(time.time()-tPhases)*1000, "ms")
            

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
            
            sp = np.absolute(np.fft.fft(y,self.N))
            #print(sp[0])
            for iteration in range(self.SliceValue):
                sp[0][int(iteration)]=1e-9
                if iteration >= 0:
                    sp[0][int((self.numSamples-1)-iteration)]=1e-9
            
            sp[0]=np.fft.fftshift(sp[0])

            bartlett = np.add(bartlett,sp[0]) #sum each fft

            PeakValue1 = 20*np.log10(sp[0][self.MVF1])
            PeakValue2 = 20*np.log10(sp[0][self.MVF2])

            if PeakValue1 > max_signal1:
                    max_signal1 = PeakValue1
                    #theta = 0.9425*theta + 0.154 calibrated correction
                    max_angle1 = 0.9425*self.SteeringAngles[SteerStepNumber]+0.154

            if PeakValue2 > max_signal2:
                    max_signal2 = PeakValue2
                    #theta = 0.9425*theta + 0.154 calibrated correction
                    max_angle2 = 0.9425*self.SteeringAngles[SteerStepNumber]+0.154

            if self.SignalNumbers<1:
                PeakValue1=1e-9
                PeakValue2=1e-9
            elif self.SignalNumbers == 1:
                PeakValue2=1e-9
            

            output_items[0][SteerStepNumber] = ((1)*self.SteeringAngles[SteerStepNumber] + (1j * PeakValue1))
            output_items[4][SteerStepNumber] = ((1)*self.SteeringAngles[SteerStepNumber] + (1j * PeakValue2))
            SteerStepNumber = SteerStepNumber + 1

                

            #end of phaseshift  for-loop

        

        #self.MVF1 = np.argmax(bartlett) #finds the index of max value
        # Find the index of the next biggest value
        # next_biggest_index = 0 # Default value if not found

        # for i, x in enumerate(bartlett):
        #     if x >= bartlett[self.MVF1]:
        #         continue  # Skip values greater than or equal to the maximum
        #     if self.MVF2 == 0 or x > bartlett[self.MVF2]:
        #         self.MVF2 = i

        BartlettAverage = np.mean(bartlett)
        #print("Average is " + str(BartlettAverage))
        afterpeak = 0
        self.MVF1=0
        self.MVF2=0
        OverAverage=0
        #bartlett = np.fft.fftshift(bartlett)
        for i, x in enumerate(bartlett):
            if x >= BartlettAverage:
                OverAverage=1
            
            if OverAverage>0: #it has been over average
                if afterpeak > 0:
                    if self.MVF2 == 0 or x > bartlett[self.MVF2]:
                        self.MVF2 = i
                        #print("hooo " + str(i)+"  "+ str(x))
                elif self.MVF1 == 0 or x > bartlett[self.MVF1]:
                    self.MVF1 = i
                    #print("heiii " + str(i) +"  "+ str(x))
                elif x < BartlettAverage:
                    afterpeak=1
                    #print("afterpeak happens at " + str(i)+"  "+ str(x))

        #Threshold analysis of the signals present
        if 20*np.log10(bartlett[self.MVF1]) >= self.Bartlett_Threshold:
            if 20*np.log10(bartlett[self.MVF2]) >= self.Bartlett_Threshold:
                self.SignalNumbers = 2
            else:
                self.SignalNumbers = 1
        else:
            self.SignalNumbers = 0

        


        
        FreqOfInterest1=round((self.SignalFreq + self.freqVal[self.MVF1])/1e6)
        FreqOfInterest2=round((self.SignalFreq + self.freqVal[self.MVF2])/1e6)

        output_items[2][0:self.numSamples] = self.freqVal+1j*20*np.log10(bartlett)
        output_items[2]=output_items[2][0:self.numSamples]
        #print(output_items[2])
        


        print("Sweep time: ",(time.time()-tSweep)*1000, "ms")  
       
        self.averageArray = np.append(self.averageArray[-self.averageNumber:], max_angle1)
        
        #time.sleep(2)
        print("just sendt " + str(round(np.mean(self.averageArray))))
        #self.SerialObj.write(bytes(str(round(max_angle1)), "utf-8"))  # Transmit input to Arduino
        
        #time.sleep(2)

        #Write to file
        self.angles.append(max_angle1)
        with open(self.doa_text_file,"w") as f:
            f.write("__DOA__\n")
            for angles in self.angles:
                f.write(f"{angles}, ")
        

        



        output_items[0] = output_items[0][0:SteerStepNumber]
        output_items[1][:] = max_angle1
        output_items[3][:] = FreqOfInterest1
        output_items[5][:] = FreqOfInterest2
        output_items[4] = output_items[0][0:SteerStepNumber]
        return len(output_items[0])