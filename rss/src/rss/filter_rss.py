#!/usr/bin/python

from __future__ import print_function
import sys
import os
import argparse
import signal

def eprint(*args, **kwargs):                    #prints errors/warnings to stderr
    print(*args, file=sys.stderr, **kwargs)
    sys.stderr.flush()

def oprint(*args):                    #prints to stdout
    print(*args, file=sys.stdout)
    sys.stdout.flush()

def signal_handler(signal, frame):
    global interrupted
    interrupted = True

if __name__=="__main__":

    global interrupted
    interrupted = False
    signal.signal(signal.SIGINT, signal_handler) #to handle signals

    warn_flag = True

    while True:
        line = sys.stdin.readline()
        signal_pos  = line.find('signal')   #finds 'signal'
        bssid_pos   = line.find('BSSID')
        channel_pos = line.find('MHz')
        
        if (signal_pos != -1) and (bssid_pos != -1) and (channel_pos != -1):            
            dB = line[:signal_pos].split()[-1][:-2]  # db is block before 'signal' [:-2] so dB is not printed
            channel = line[:channel_pos].split()[-1] #channel is the block before 'MHz'
            BSSID = line[bssid_pos:].split()[0][6:] # BSSID: begins with 'BSSID' [6:] so BSSID is not printed
            oprint(str(dB)+' '+str(BSSID)+' '+str(channel))
        else:
            if warn_flag:
                warn_flag = False
                eprint('Warning: one metric (RSS,BSSID,CHANNEL) not found')

        if interrupted:
            eprint('Closing filter')
            break
