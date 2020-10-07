#!/usr/bin/python

from __future__ import print_function
import time
import signal
import sys
import os
import argparse

def eprint(*args):                    #prints errors/warnings to stderr
    print(*args, file=sys.stderr)

def oprint(*args):                    #prints to stdout
    print(*args,file=sys.stdout)
    sys.stdout.flush()

def signal_handler(signal, frame):
    global interrupted
    interrupted = True
    
def channel_hopper(**kwargs):
    channels    = kwargs.pop('channels',(1,4,11,))
    sample_time = kwargs.pop('ts',1)    
    iface       = kwargs.pop('iface','eth1')
    eprint('Channel Hopper')
    eprint('Channels {}'.format(channels))

    interval    = 1.*sample_time/len(channels)
    if interval < 0.1:
        eprint('Warning, set ts so at least 100 ms are available for each channel')
    else:
        eprint('Interval for each channel {} s'.format(interval))
    global interrupted
    interrupted = False
    signal.signal(signal.SIGINT, signal_handler) #to handle signals
    
    eprint('Hopping channels starting')
    s = 0    
    while True: 
        for ch in channels:
            command = "sudo -S iwconfig {} channel {}".format(iface,ch)
            os.system(command)
            time.sleep(interval) 
            if s==0:
                oprint('ready') 
                s = 1

        if interrupted:
            eprint('Closing channel hopper')
            break

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Wireless channel hopper.')
    parser.add_argument('-i',dest='iface',default='eth1',help='wlan interface')
    parser.add_argument('-t',dest='ts',type=int,default=1,help='Total sample time of a hoop')
    parser.add_argument('-ch',dest='channels',type=int,nargs='+',default=(1,4,11,),help='Channels to hop')
    args = parser.parse_args()

    channel_hopper(**vars(args))
