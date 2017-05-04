import numpy as np
import seaborn as sns
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D

import re
import numpy
import seaborn as sns


sns.set_style("white")
sns.set_style("ticks")

mpl.rc('xtick', labelsize=26) 
mpl.rc('ytick', labelsize=26)
mpl.rcParams.update({'font.size': 26})
Greens = sns.color_palette("Greens")
Blues  = sns.color_palette("Blues")

sns.set_palette("colorblind")

def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.

    Format specification: http://netpbm.sourceforge.net/doc/pgm.html

    """
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    image = numpy.frombuffer(buffer, dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height), offset=len(header)
                            ).reshape((int(height), int(width)))

    #make unexplored areas gray clear
    image.flags['WRITEABLE'] = True
    image[image == 205] = 220
    image.flags['WRITEABLE'] = False
    
    return image


def read_yaml(filename):
    map_yaml = dict()
    with open(filename,'rb') as f:
        lines = f.readlines()
        for line in lines:   
            try:
                key = line.split(':')[0]
                val = line.split(':')[1][:-1]
                if key == 'origin':
                    val = val.split('[')[1]
                    val = val.split(']')[0]
                    val = np.array(val.split(','),dtype=float)
                else:
                    try:
                        val = float(val)
                    except:
                        val = val.replace(' ','') #eliminate white spaces
                map_yaml[key]=val
            except:
                pass
    return map_yaml


class mapviz():
    def __init__(self,root='/home/renato/catkin_ws/src/tests/maps/b2map'):
        fmap  = root+'.pgm'
        fyaml = root+'.yaml'
        
        self.image = read_pgm(fmap, byteorder='<')
        self.map_yaml = read_yaml(fyaml)
        span_x = len(self.image)*self.map_yaml['resolution']
        span_y = len(self.image[0])*self.map_yaml['resolution']
        self.extent = [self.map_yaml['origin'][0],self.map_yaml['origin'][0]+span_x,self.map_yaml['origin'][1],self.map_yaml['origin'][1]+span_y]
        
    def plot(self,f=None,ax=None):
        if f is None:
            f, ax = plt.subplots()
        ax.imshow(self.image.T,plt.cm.gray,origin='lower',extent=self.extent)
        return f,ax

