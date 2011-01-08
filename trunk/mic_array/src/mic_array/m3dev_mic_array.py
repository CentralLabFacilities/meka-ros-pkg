#! /usr/bin/python
from pylab import *
import Gnuplot
import m3.toolbox as m3t
import u3
from time import sleep
from datetime import datetime
import struct
import math
#import numpy
import sys
from scipy.signal import convolve, remez
from scipy import array
# MAX_REQUESTS is the number of packets to be read.
# At high frequencies ( >5 kHz), the number of samples will be MAX_REQUESTS times 48 (packets per request) times 25 (samples per packet)

class RingBuffer:
    def __init__(self, size):
        self.data = [0.0]*size
        self.size = size

    def append(self, x):
        self.data.pop(0)
        self.data.append(x)

    def get(self):
        return self.data
    
num_chan=6
gain = [2.2,0.8,1.0,1.8,1.0,0.5]
dac_map = [2,1,6,5,4,3]
thresh = 150
#thresh = 10000

samp_freq=math.floor(10000/num_chan)
filt = remez(numtaps=40, bands=[0, 70, 71, 270, 271, math.floor(samp_freq/2)], desired=[0, 1, 0], Hz = samp_freq)

p = 2
scale = 100.0

d = u3.U3()
d.configIO( FIOAnalog = 0x7E )
    
d.streamConfig(NumChannels = 6, PChannels = [1,2,3,4,5,6], NChannels = [ 31 ]*6, Resolution = 1, SampleFrequency = samp_freq)

print "samps per pkt:", d.streamSamplesPerPacket
print "pkts per req:", d.packetsPerRequest

buf_size = (d.streamSamplesPerPacket * d.packetsPerRequest)/num_chan

mem_size = int(math.floor(buf_size * 1.5))

window_time = 0.1
window_size = int(math.floor(window_time*samp_freq))

if mem_size < int(math.floor(window_size * 1.5)):
    mem_size = int(math.floor(window_size * 1.5))

print "buff size:", buf_size
print "mem size:", mem_size
print "win size:", window_size

buf = [None]*num_chan
for i in range(num_chan):
    buf[i] = RingBuffer(mem_size)

x=range(mem_size)

y=[0.0]*len(x)
y_lim=[0.0, 1500.0]
#y_lim=[0.0, 150.0]

g = [None]*num_chan

for i in range(num_chan):
    g[i] = m3t.gplot(y,x,g[i],y_lim,persist_in=0)
    g[i].title('mic ' + str(i))

try:    
    d.streamStart()
    
    missed = 0
    start = datetime.now()
    dataCount = 0
    
    while True:
        for r in d.streamData():
            if r is not None:
                if r['errors'] != 0:
                    print "Error: %s ; " % r['errors'], datetime.now()
    
                if r['numPackets'] != d.packetsPerRequest:
                    print "----- UNDERFLOW : %s : " % r['numPackets'], datetime.now()
    
                if r['missed'] != 0:
                    missed += r['missed']
                    print "+++ Missed ", r['missed']
    
                if len(r['AIN'+str(dac_map[0])]) != buf_size:
                    print "-----  err buf_size != rx.   len(AIN1):", len(r['AIN1'])
                    
                dataCount += 1
                buf_filt = [None]*num_chan
                
                for j in range(num_chan):
                    for i in range(len(r['AIN'+str(dac_map[j])])):                                        
                        buf[j].append(((r['AIN'+str(dac_map[j])][i])*scale*gain[j]))
                        
                    
                    #g[j] = m3t.gplot(buf[j].get(),x,g[j],y_lim,persist_in=1)
                    buf_filt[j] = convolve(filt, buf[j].get())
                    for i in range(len(buf_filt[j])):
                        buf_filt[j][i] = buf_filt[j][i] ** p
                    g[j] = m3t.gplot(buf_filt[j],x,g[j],y_lim,persist_in=0)
                    

                T_all = []
                Max_E_all = []
                Max_I_all = []
                for i in range(mem_size-window_size):
                    if i % 2 == 0:
                        energy = [0.0] * num_chan
                        for j in range(num_chan):
                            energy[j] = sum(buf_filt[j][i:(i+window_size)])/window_size
                        D = 0.0
                        B = 0.0
                        A = 0.0
                        
                        baseline = 99999999.0
                        for i in range(len(energy)):
                            if energy[i] < baseline:
                                baseline = energy[i]   
                        for i in range(len(energy)):
                            energy[i] -= baseline

                        if sum(energy) > thresh:  # Okay we've detected an event.  Now locate which 2 mic's it is between
                            e_sum = [None]*num_chan
                            for i in range(num_chan):                                
                                if i == num_chan - 1:
                                    e_sum[i] = energy[i] + energy[0]
                                else:
                                    e_sum[i] = energy[i] + energy[i + 1]
                            max_e = -1
                            max_i = 0
                            for i in range(num_chan):
                                if e_sum[i] > max_e:
                                    max_i = i
                                    max_e = e_sum[i]
                            
                            if  max_i == 0:
                                D = 30.0
                                A = energy[0]
                                B = energy[1]
                            elif max_i == 1:
                                D = 90.0
                                A = energy[1]
                                B = energy[2]
                            elif max_i == 2:
                                D = 150.0
                                A = energy[2]
                                B = energy[3]
                            elif max_i == 3:
                                D = -150.0
                                A = energy[3]
                                B = energy[4]
                            elif max_i == 4:
                                D = -90.0
                                A = energy[4]
                                B = energy[5]
                            elif max_i == 5:
                                D = -30.0
                                A = energy[5]
                                B = energy[0]
                            
                            
                            T = D + 30.0*(B**2-A**2)/(B**2+A**2)
                            print "Detected: T:", T, "A:", A, "B:", B, "D:", D, "i:", max_i 
                            print "Energy:", max_e
                            T_all.append(T)
                            Max_E_all.append(max_e)
                            Max_I_all.append(max_i)
                max_e_window = -1
                max_i_window = 0
                for i in range(len(Max_E_all)):
                    if Max_E_all[i] > max_e_window:
                        max_e_window = Max_E_all[i]
                        max_i_window = i
                if len(T_all) > 0:
                    print "Max Detected: T:", T_all[max_i_window], "i:", Max_I_all[max_i_window]
                    print "Max Energy:", max_e_window

except (KeyboardInterrupt,EOFError):
    pass

stop = datetime.now()
d.streamStop()
d.close()
for i in range(num_chan):
    g[i].close()


total = dataCount * d.packetsPerRequest * d.streamSamplesPerPacket
print "%s requests with %s packets per request with %s samples per request = %s samples total." % ( dataCount, d.packetsPerRequest, d.streamSamplesPerPacket, total )
print "%s samples were lost due to errors." % missed
total -= missed
print "Adjusted number of samples = %s" % total

runTime = (stop-start).seconds + float((stop-start).microseconds)/1000000
print "The experiment took %s seconds." % runTime
print "%s samples / %s seconds = %s Hz" % ( total, runTime, float(total)/runTime )
    
    
