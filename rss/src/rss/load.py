from Localization.sensormodel.hybrid import hGP
from Localization.sensormodel.gp import GP
import os

def load_data(**kwargs):
    """
    Load a previously pickled data from file_path/file_name.
    Optional Parameters [default]
        file_name ['b3test1']: Core name of the files to get data from -> <file_name>rss.p and <file_name>poses.p
        file_path ['~/catkin_ws/src/tests/bags/processed_data/']: Path where to find files   
    """

    file_name = kwargs.get('file_name','b3test1')
    #TODO:change file path
    file_path = kwargs.get('file_path',None)
    if file_path is None:
        file_path=os.path.expanduser('~')+'/catkin_ws/src/tests/bags/processed_data/'
    
    f=file_path+file_name

    try: 
      import cPickle as pickle
    except ImportError:
      import pickle
  
    file1 = f+'rss.p'
    file2 = f+'poses.p'
    file3 = f+'odom.p'
  
    with open(file1, 'rb') as f:
      rss = pickle.load(f)
    with open(file2, 'rb') as f:
      poses = pickle.load(f)
    with open(file3, 'rb') as f:
      odom = pickle.load(f)
    
    return (rss,poses,odom)


def load_model(**kwargs):
    """
    Load a previously pickled data from file_path/file_name.
    Optional Parameters [default]
        file_name ['b3test1']: Core name of the files to get data from -> <file_name>rss.p and <file_name>poses.p
        file_path ['~/catkin_ws/src/tests/bags/processed_data/']: Path where to find files   
    """

    file_name = kwargs.get('file_name','b3_hgp')
    file_path = kwargs.get('file_path',None)
    if file_path is None:
        file_path=os.path.expanduser('~')+'/catkin_ws/src/tests/bags/processed_data/'

    f=file_path+file_name+'.p'

    # Load model
    data_empty = {'X':np.asarray([[1,1]]),'Y':np.asarray([1]),'Var':np.asarray([1])}
    h = hGP(data_empty,verbose=True)
    h.load(filepath=f)    
    
    return h

    

