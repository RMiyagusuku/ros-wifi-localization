from __future__ import print_function
import numpy as np
import sys
import os
import time
import subprocess
import argparse
import signal


def eprint(*args, **kwargs):                    #prints errors/warnings to stderr
    print(*args, file=sys.stderr, **kwargs)

def signal_handler(signal, frame):
    global interrupted
    interrupted = True

class Device:
    """
    Class wirelessDevice
    Wireless lan device handler

    Attributes
    
    iface Name of the wireless device
    tempf Name of the temporal file were measurements are stored
    ready Flag for acquiring/reading state  0:Measuring 1:Measurement ready
    """

    def __init__(self,**kwargs):
            
        self.iface = kwargs.get('iface','eth1')
        self.logf = self.iface+".log" 
        self.buffer = kwargs.get('buffer',1)
        self.channels = kwargs.get('channels',(1,6,11))
        self.ts = kwargs.get('sample_time',1)
        self.filter = kwargs.get('filter',True)

        self.filter_process = None
        self.read_process = None
        self.chopper_process = None
        self.proc_out = None

        # Calculating time to sample with tcpdump
        #TODO: compute this time, self adjust after each reading 
        self.dump_time  = 0.1 # time in sec to dump the tcp file into the txt     
        
        #NOTE: For scan, the mode can not be monitor
        os.system("sudo iwconfig " + self.iface + " channel 1")      #set channel to 1st
        os.system("sudo ifconfig " + self.iface + " down")           #turn interface off
        os.system("sudo iwconfig " + self.iface + " mode monitor")   #change to monitor mode
        os.system("sudo ifconfig " + self.iface + " up")             #turn interface on
    
    def read_start(self):
        """
        iface must be initialized before with init(iface)
        """
        eprint('Read RSS starting')
        cmd = 'sudo tcpdump -npeqi {} -f type mgt subtype beacon'.format(self.iface)
        #scan command = "sudo iwlist " + self.wname + " scan | egrep 'Address|Signal' > " + self.tempf
        self.read_process = subprocess.Popen(cmd.split(),stdout=subprocess.PIPE)
        self.proc_out = self.read_process.stdout
        
        if self.filter:
            cmd = 'filter_rss.py'
            self.filter_process = subprocess.Popen(cmd,stdin=self.read_process.stdout,stdout=subprocess.PIPE)
            self.proc_out = self.filter_process.stdout
            eprint('Using Filter')

    def chopper_start(self):
        cmd = "chopper.py -i {} -t {} -ch {}".format(self.iface,self.ts," ".join(str(ch) for ch in self.channels))
        self.chopper_process = subprocess.Popen(cmd.split(),stdout=subprocess.PIPE)

    def __str__(self):the 
        """
        Overloading __str__ to make print statement meaningful
        
        <format>
        Device                      :
        Temp file                   :
        Sample time                 :
        Buffer                      :
        """
        to_print = '{}  : {}\n'.format('Device'.ljust(34),self.wname)
        to_print = to_print + '{}  : {}\n'.format('Temp file'.ljust(34),tempf)
        
        return to_print

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='RSSI measurement node: sets wireless IFACE to monitor mode and hoppes between CHANNELS; and uses tcpdump to sense acquire only beacon information from routers and outputs the readings (filtered or not (-nf)) into stdout or logs.')
    parser.add_argument('-i',dest='iface',default='eth1',help='wlan interface')
    parser.add_argument('-t',dest='ts',type=int,default=1,help='Total sample time of a hoop')
    parser.add_argument('-ch',dest='channels',type=int,nargs='+',default=(1,4,11,),help='Channels to hop')
    parser.add_argument('-b',dest='buffer',type=int,default=5,help='Number of buffered log files')
    parser.add_argument('-nf',dest='filter',default=True,action='store_false',help='Turn filtering of tcpdump output off')
        
    args = parser.parse_args()
    device = Device(**vars(args))                   #device init
    device.chopper_start()                          #channel hopper start

    while True:                                     #wait for channel hopper
        line = device.chopper_process.stdout.readline()[:-1]
        if line == 'ready':
            break

    eprint('Streaming starting')
    device.read_start()                         #reading wireless start
#    sys.stdout = device.proc_out#.readline()[:-1]

    global interrupted
    interrupted = False
    signal.signal(signal.SIGINT, signal_handler) #to handle signals

    while True:
        line = device.proc_out.readline()[:-1]
        print(line)

        if interrupted:
            eprint('Measurement acquisition stoped')
            break

class Measurement:
    """
    Class measurement

    Attributes
    mean: Mean value of RSSI measurement
    std : Standard deviation of RSSI measurement
    n   : Number of RSSI data points taken during measurement
    z   : How old the measurement is, being 0 the measurement taken in the last step
    """

    def __init__(self, mean, std, n):
        self.mean = mean
        self.std  = std
        self.n    = n
        self.z    = 0               # how old the meassurement is, 0: new

    def new(self,filename):
        pass
        # process a new meassurement from filename
        # add some counter to see how old meassurement is
        # replace new RSSI values found for existing address
        # create new entry for first time found address

    def view(self):
        print('RSSI: '+str(self.mean)+" +/- "+str(self.std)+" ("+str(self.n)+","+str(self.z)+")")


    def help(self):
        print("""
Class measurement

Attributes
mean: Mean value of RSSI measurement
std : Standard deviation of RSSI measurement
n   : Number of RSSI data points taken during measurement
z   : How old the measurement is, being 0 the measurement taken in the last step
""")


##############################################################################
################################### Functions ################################
##############################################################################
# robot1@robot1-CF-W4:~$ sudo tcpdump -l -npi eth1 -c 100 >  exampletcpdump

def decodeScan(FileName):
    """ 
    Function to extract RSSI information stored in a .txt file    
 
    Input 
    FileName  Name of .txt file, input as string including file type termination    
    
    Output
    addressList Dictionary with address names and index where they are stored
    RSSI measurement class containing RSSI information

    Example
    aL, RSSI = decode('File.txt')
    """

    File = open(FileName,'r')
    Data = File.readlines()
    File.close()
    addressList = {}
    RSSI = list()
    index = 0
    for datum in Data:
        addressIndex = datum.find('Address:')        
        if addressIndex != -1:                  #if found
            #address is the text after a space (split separates string by spaces)
            address = datum[addressIndex:-1].split()[1]
            addressList.update({address:index})
            index += 1
        else:
            RSSIIndex = datum.find('Signal level')
            #RSSI value is the text that begins with a '-', then we drop all after a space
            value = -float(datum[RSSIIndex:-1].split('-')[1].split()[0])
            RSSI.append(Measurement(value,0,1))

    return (addressList, RSSI)
#####     #####     #####     #####     #####     #####     #####     #####

def decodeTcpdump(FileName,**options):
    """ 
    Function to extract RSSI information stored in a .txt file    
    
    Input 
    FileName  Name of .txt file, input as string including file type termination    
    
    Output
    addressList Dictionary with address names and index where they are stored
    RSSI measurement class containing RSSI information

    Example
    aL, RSSI = decode('File.txt') or aL, RSSI = decode('File.txt',raw=1) 

    aL{'mac:address'} = i
    RSSI[i].mean, RSSI[i].std RSSI[i].n RSSI[i].z  (Check Measurement class)
    """

    File = open(FileName,'r')   # Just reading
    Data = File.readlines()     # Data to memory
    File.close()                # Close file
    addressList = {}

    RSSI_raw = [[]]

# Obtaining RSSI information from FileName 
    for datum in Data:
        addInd = datum.find('SA:')        # Find address index xx:xx:xx:xx:xx:xx whith x being any letter or numeral
        if addInd != -1:                    # if found (-1 when not found)    
            address = datum[addInd+3:addInd+20]
            if address in addressList:
                index = addressList[address]    # Recover index from dict
            else:
                index = len(addressList)        # Assign new index

                addressList.update({address:index})      # Add address to dict inverseAddressList.update({index:address})
                if index > 0:   # To avoid adding unnecesary empty entries      
                    RSSI_raw.append([])                  # Add RSSI new entry
    
            indexDBM  = datum.find('dB')   # find the string dBm consider the past 10 strings from dBm where the RSSI value should be, use split to separate the value from other text, it should be separated by a comma, take the last string splitted transform to int as RSSI values are integers (no decimal points)
                        
            try:
                rssiValue = int(datum[indexDBM-5:indexDBM].split()[-1])  # find rssi value
                RSSI_raw[index].append(rssiValue)   # append to output array
#            time[index].append(timeTemp)
            except Exception:
                pass


    if options.get("raw"):
        return (addressList, RSSI_raw)

    else:
    # Processing obatained RSSI Data
        RSSI = list()
        for rssi in RSSI_raw:
            if len(rssi) > 0:
                RSSI.append(Measurement(np.mean(rssi),np.std(rssi),len(rssi)))
            else:
                RSSI.append(Measurement(-100,0,0))
            
        return (addressList, RSSI)
#####     #####     #####     #####     #####     #####     #####     #####


