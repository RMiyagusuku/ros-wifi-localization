#!/usr/bin/env python
from __future__ import print_function
#standard libraries
import sys
import numpy as np
import rospy

#ros messages
from geometry_msgs.msg import PoseArray, Pose
from rss.msg import RssData, ProbArray

#Services
from rss.srv import Localization

#personal libraries
from rss.classes import ProcessedData
from util.others import mesh
from Localization.sensormodel.sensormodel import SensorModel

debug = 0

def eprint(*args, **kwargs):                    #prints errors/warnings to stderr
    print(*args, file=sys.stderr, **kwargs)


def pose_from_array(array):
    pose = Pose()
    pose.position.x = array.flatten()[0]
    pose.position.y = array.flatten()[1]
    return pose

def loc_server_handle(model,raw_rss,nsamples,pose_array, **kwargs): 
    """
    Handle for the location server service
    
    Inputs    
        model           [GP based model]
        raw_rss         [RssData] Received RSS message (rss.msg RssData)
        nsamples        [Int] with the number of desired samples, 
                        if nsamples==1 the most likely sample is returned
        pose_array      [PoseArray] if empty, the loc_server samples the model and 
                        returns a PoseArray made of those samples. 
                        If not, the samples in the PoseArray are used
    Optional inputs
        x_min,x_max,    [Floats] limits used for the sampling algorithm and to calculate 
        y_min,y_max,spn the most likely sample

    Returns
        pose_array      Original pose_array if provided, or pose_array made of samples drawn 
                        from model  
        weights         Weights (likelihood) of pose_array for the model given the RssData
                        provided
    """    
    
    ns = kwargs.pop('ns','')
    limits = rospy.get_param(ns+'/map_limits')
    
    x_min = limits['x_min']
    x_max = limits['x_max']
    y_min = limits['y_min']
    y_max = limits['y_max']
    spn   = limits['spn']

    frame = kwargs.pop('parent_frame','map')

    if isinstance(raw_rss,list):
        rss_list = raw_rss 
    else:
        rss_list = (raw_rss,)       

    if not isinstance(model,SensorModel):
        eprint('[WARN] Provided model is not a valid SensorModel')
        return [pose_array, list(), list()]

    nrss = len(rss_list)
    rss  = ProcessedData(rss_list,other_mac_dict=model.all_mac_dict,
            filters_on=False,warn_pose_off=True,flag_fuse_measurements=True,
            filter_fuse_measurements=nrss)
    

    if debug:
        eprint(type(rss))
        for rss_datum in rss_list:
            eprint(str(rss.data['Y'].shape[1])+' acccess points out of '+str(len(rss_datum.mac_address))+' used')
    # use train mac, filters off by default, no warning due to poses missing, 
    # fuse measurements on, fuse all measurements available
        
    temp_x  = np.linspace(x_min,x_max,spn)
    temp_y  = np.linspace(y_min,y_max,spn)
    Xtest   = mesh(temp_x,temp_y)
    prob_mesh = model.jointpdf(Xtest,rss.data['Y'])
    
    if debug:
        eprint('Request')
        eprint('nsamples:       \t'+str(nsamples))
        eprint('pose_array len: \t'+str(len(pose_array.poses)))
    if nsamples == 1: #if single sample is requested, send the one with max probability
        if debug:
            eprint('Single sample, max probability loc_srv sent')
        pose_array.header.stamp = rospy.get_rostime()
        index = np.argmax(prob_mesh)        
        pose_array.poses = list()        
        pose  = pose_from_array(Xtest[index,:])
        pose_array.poses.append(pose)
        
        weights = list()
        weights.append(pX[index])

        return [pose_array, weights, prob_mesh.flatten()]

    #if more than one sample is requested

    # if nsamples differs from pose_array size - e.g., empty pose_array nsamples are drawn from the current model's posterior given the rss message
    if nsamples != len(pose_array.poses): 
        if debug:
            eprint('Sampling from model, loc_srv sent')
        pose_array = PoseArray()
        #pose_array.header.seq = self.seq 
        #self.seq += 1
        pose_array.header.stamp = rospy.get_rostime()
        pose_array.header.frame_id = frame
        pose_array.poses = list()

        try:
            measurement = rss.data['Y']
            samples     = model.sample(measurement,span=(x_min,x_max,y_min,y_max),
                            nsamples=nsamples,K=50,batch=1)#,alpha=alpha[0])
            pose_array.poses = [pose_from_array(x) for x in samples]
        except:
            e = sys.exc_info()[0]
            eprint('[ERROR] Sampling failed: '+str(e))
            return None  

    # if nsamples equals the number of poses in pose_array use those poses and compute weights
    else: 
        if debug:
            eprint('Evaluating samples likelihiood loc_srv sent')
        samples = [[p.position.x, p.position.y] for p in pose_array.poses]
        samples = np.asarray(samples)
    
    # compute the weights as the likelihood of the samples
    weights = model.jointpdf(samples,rss.data['Y'])
    weights.tolist()
    prob_mesh.flatten().tolist()

    return [pose_array, weights, prob_mesh]

def loc_server_client(raw_rss,nsamples,pose_array=None,ns=''):
    """
    Client for sending localization requrests
    Inputs
        raw_rss     RssData or list(RssData) if list is sent, only the first one is used 
                    %maybe change
        nsamples    Number of sampels
   
    Optional inputs
        pose_array  PoseArray, default:None, If None, an empty PoseArray is sent
        ns          String, default:''. Namespace used for renaming the service as ns/loc_server
    """
    if pose_array is None:
        pose_array = PoseArray()

    rss_copy = raw_rss
    if isinstance(rss_copy,RssData):
        rss = (rss_copy,)
    else:   #if not RssData, check if its a list of RssData
        if isinstance(rss_copy[0],RssData):
            rss = rss_copy
        else:
            eprint("Invalid rss argument")
            return None 

    if debug:
        eprint("Sending the request")
    try:
        rospy.wait_for_service(ns+'/loc_server',timeout=2)
    except:
        eprint(ns+'/loc_server not available')
        return None

    loc_server = rospy.ServiceProxy(ns+'/loc_server', Localization)
    try:
        loc_response = loc_server(rss,int(nsamples),pose_array) 
        #make sure first argument is not a list, but a RssData
    except:
        e = sys.exc_info()[0]
        eprint('[ERROR] Loc server client: Localization failed: '+str(e))  
        return None

    if debug:
        eprint('Max probability: '+str(np.max(loc_response.weights))) #max weigth
        eprint('Avg probability: '+str(np.mean(loc_response.weights))) #max weigth


    return loc_response


