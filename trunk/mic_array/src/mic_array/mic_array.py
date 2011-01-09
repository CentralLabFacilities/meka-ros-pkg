#! /usr/bin/python
import time
import os
import roslib; roslib.load_manifest('mic_array')
import rospy
from mic_array.msg import MicArray
from mic_array.srv import MicArrayParam
import u3
from time import sleep
from datetime import datetime
import struct
import math
import sys
from scipy.signal import convolve, remez
from scipy import array
from threading import Thread

class RingBuffer:
    def __init__(self, size):
        self.data = [0.0]*size
        self.size = size

    def append(self, x):
        self.data.pop(0)
        self.data.append(x)

    def get(self):
        return self.data
    
class mic_array_thread(Thread):
    def __init__(self):
        Thread.__init__(self)        
        self.num_chan = 6
        self.gain = [2.2,0.8,1.0,1.8,1.0,0.5]
        self.dac_map = [2,1,6,5,4,3]
        self.thresh = 100000 #150
        self.mic_array_done = False
        self.pub = rospy.Publisher('mic_array', MicArray)
        samp_freq=int(math.floor(10000/self.num_chan))
        self.filt = remez(numtaps=40, bands=[0, 70, 71, 270, 271, math.floor(samp_freq/2)], desired=[0, 1, 0], Hz = samp_freq)
        
        self.p = 2
        self.scale = 100.0
        
        self.d = u3.U3()
        self.d.configIO( FIOAnalog = 0x7E )
            
        self.d.streamConfig(NumChannels = 6, PChannels = [1,2,3,4,5,6], NChannels = [ 31 ]*6, Resolution = 1, SampleFrequency = samp_freq)
        
        #print "samps per pkt:", self.d.streamSamplesPerPacket
        #print "pkts per req:", self.d.packetsPerRequest
        
        self.buf_size = int(math.floor((self.d.streamSamplesPerPacket * self.d.packetsPerRequest)/self.num_chan))
        self.samp_time = float(self.buf_size) / float(samp_freq)
        self.mem_size = int(math.floor(self.buf_size * 1.5))
        
        window_time = 0.1
        self.window_size = int(math.floor(window_time*samp_freq))
        
        if self.mem_size < int(math.floor(self.window_size * 1.5)):
            self.mem_size = int(math.floor(self.window_size * 1.5))
        
        #print "buff size:", buf_size
        #print "mem size:", mem_size
        #print "win size:", window_size
        self.buf_filt = [None]*self.num_chan
        self.buf = [None]*self.num_chan
        for i in range(self.num_chan):
            self.buf[i] = RingBuffer(self.mem_size)
                
        self.energy_mic = [0.0] * self.num_chan
        self.energy = [0.0] * self.num_chan
        
        self.xd = [0.0]*self.num_chan
        self.yd = [0.0]*self.num_chan            
        for i in range(self.num_chan):
            t = (2*math.pi/self.num_chan)*i
            self.xd[i] = math.sin(t)
            self.yd[i] = math.cos(t)
    
                
    def stop(self):
        self.mic_array_done = True

    def run(self):	
        try:    
            self.d.streamStart()
            missed = 0
            start = datetime.now()
            dataCount = 0

            for r in self.d.streamData():
                if self.mic_array_done:
                    break
                
                if r is not None:
                    if r['errors'] != 0:
                        print "Error: %s ; " % r['errors'], datetime.now()
        
                    if r['numPackets'] != self.d.packetsPerRequest:
                        print "----- UNDERFLOW : %s : " % r['numPackets'], datetime.now()
        
                    if r['missed'] != 0:
                        missed += r['missed']
                        print "+++ Missed ", r['missed']
        
                    if len(r['AIN'+str(self.dac_map[0])]) != self.buf_size:
                        print "-----  err buf_size != rx.   len(AIN1):", len(r['AIN1'])
                        
                    dataCount += 1                    
                    
                    for j in range(self.num_chan):
                        for i in range(len(r['AIN'+str(self.dac_map[j])])):                                        
                            self.buf[j].append(((r['AIN'+str(self.dac_map[j])][i])*self.scale*self.gain[j]))                        
                        self.buf_filt[j] = convolve(self.filt, self.buf[j].get())
                        for i in range(len(self.buf_filt[j])):
                            self.buf_filt[j][i] = self.buf_filt[j][i] ** self.p                        
                        self.energy_mic[j] = sum(self.buf_filt[j])/len(self.buf_filt[j])

                    xd_p = []
                    yd_p = []
                    for i in range(self.num_chan):
                        xd_p.append(self.xd[i] * self.energy_mic[i])
                        yd_p.append(self.yd[i] * self.energy_mic[i])
                    xd_src = sum(xd_p)
                    yd_src = sum(yd_p)
                    angle_src = math.atan2(yd_src,xd_src)
                    mag_src = math.sqrt(xd_src**2 + yd_src**2)

                    self.pub.publish(self.energy_mic, angle_src, mag_src)
                    
                    if False:
                        T_all = []
                        Max_E_all = []
                        Max_I_all = []
                        for i in range(self.mem_size-self.window_size):
                            if i % 2 == 0:
                                for j in range(self.num_chan):
                                    self.energy[j] = sum(self.buf_filt[j][i:(i+self.window_size)])/self.window_size
                                D = 0.0
                                B = 0.0
                                A = 0.0
                                
                                baseline = 99999999.0
                                for i in range(len(self.energy)):
                                    if self.energy[i] < baseline:
                                        self.baseline = self.energy[i]   
                                for i in range(len(self.energy)):
                                    self.energy[i] -= baseline
        
                                if sum(self.energy) > self.thresh:  # Okay we've detected an event.  Now locate which 2 mic's it is between
                                    e_sum = [None]*self.num_chan
                                    for i in range(self.num_chan):                                
                                        if i == self.num_chan - 1:
                                            e_sum[i] = self.energy[i] + self.energy[0]
                                        else:
                                            e_sum[i] = self.energy[i] + self.energy[i + 1]
                                    max_e = -1
                                    max_i = 0
                                    for i in range(self.num_chan):
                                        if e_sum[i] > max_e:
                                            max_i = i
                                            max_e = e_sum[i]
                                    
                                    if  max_i == 0:
                                        D = 30.0
                                        A = self.energy[0]
                                        B = self.energy[1]
                                    elif max_i == 1:
                                        D = 90.0
                                        A = self.energy[1]
                                        B = self.energy[2]
                                    elif max_i == 2:
                                        D = 150.0
                                        A = self.energy[2]
                                        B = self.energy[3]
                                    elif max_i == 3:
                                        D = -150.0
                                        A = self.energy[3]
                                        B = self.energy[4]
                                    elif max_i == 4:
                                        D = -90.0
                                        A = self.energy[4]
                                        B = self.energy[5]
                                    elif max_i == 5:
                                        D = -30.0
                                        A = self.energy[5]
                                        B = self.energy[0]
                                    
                                    
                                    T = D + 30.0*(B**2-A**2)/(B**2+A**2)
                                    #print "Detected: T:", T, "A:", A, "B:", B, "D:", D, "i:", max_i 
                                    #print "Energy:", max_e
                                    T_all.append(T)
                                    Max_E_all.append(max_e)
                                    Max_I_all.append(max_i)
                        max_e_window = -1
                        max_i_window = 0
                        for i in range(len(Max_E_all)):
                            if Max_E_all[i] > max_e_window:
                                max_e_window = Max_E_all[i]
                                max_i_window = i
                        '''if len(T_all) > 0:
                            print "Max Detected: T:", T_all[max_i_window], "i:", Max_I_all[max_i_window]
                            print "Max Energy:", max_e_window'''

        finally:
            print 'Stopping LabJack capture.'
            stop = datetime.now()
            self.d.streamStop()
            self.d.close()
            
###################################################################

rospy.init_node('mic_array', anonymous=True)

mic = mic_array_thread()
mic.start()

try:
    while True:
        rospy.sleep(0.1)
except:
    pass

mic.stop()