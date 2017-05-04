from __future__ import print_function
import sys
#standard libraries
import numpy as np
import pickle
import copy
#Data types
from rss.msg import RssData
from geometry_msgs.msg import PoseWithCovarianceStamped as Pose

def eprint(*args, **kwargs):                    #prints errors/warnings to stderr
    print(*args, file=sys.stderr, **kwargs)



#class Odom():
#    def __init__(self,poses,**kwargs):
#        """
#        Class around odometry data. Provides functions for analysing
#        
#        Input
#            poses: ROS         
#        """

#        kwargs.get('initial_pos',[0,0,0])
        
        

############################################## MEASUREMENT DATA CLASS ##############################################
class Measurement():
    def __init__(self,mdata,**kwargs):
        """
        Class around RssData message. Provides functions for analysing and filtering the messages. 

        Input
            mdata: ROS RssData message
        
        Optional Parameters [default]
            poses [None]: ROS Trajectory message
            filter_rss_factor [0.3]: Factor used to filter rss
            min_repetition [False] : Flag to activate filtering by minimum number of rss data acquisitions per position
            min_repetition_rss [5] : Minimum number of rss data acquisitions per position
            min_rss [-100] : Minimum rss value assigned when no measurement is found
            prior_var [5]  : Prior variances assigned when no measurement is found
        
        Functions
            filter_rss
            find_pose
            pose_distance
            rss_mean
            rss_var
            rss_len
            transform2vector
        """
        assert type(mdata) is RssData

        poses = kwargs.get('poses',None)
        #TODO: make this automatic
        self.poses_type = kwargs.get('poses_type','PoseWithCovarianceStamped')        

        #filter params
        self.filter_factor       = kwargs.get('filter_rss_factor',0.3) #30% lower than mode
        self.min_repetition_flag = kwargs.get('min_repetition',False)
        self.min_repetition_rss  = kwargs.get('min_repetition_rss',5)
    
        #measurement param
        self.min_rss   = kwargs.get('min_rss',-100)
        self.prior_var = kwargs.get('prior_var',5)
                
        #retrieve mdata
        self.time = long(mdata.time_start_ns+long(mdata.duration_ms*1000))
        self.nap = len(mdata.mac_address)
        self.mac_dict = {mdata.mac_address[i]:i for i in np.arange(self.nap)}
        self.freq = list(mdata.freq)
        self.rss  = [list(data.rss) for data in mdata.data]
        
        #Filter
        self.filter_rss()
        
        #Find measurement pose from poses
        if poses is not None:
            try:
                self.find_pose(poses)
            except:
                self.pose = None
        else:
            self.pose = None
    
    def filter_rss(self,**kwargs):
        """
        Filters rss data by finding the mode and eliminating rss values which have less than filter_rss_factor*mode of repetitions
        If the min_repetition flag is activated rss data points with less than min_repetition_rss points are also filtered
        
        Parameters [default]
            filter_rss_factor [class filter_factor]
            min_repetition [class min_repetition_flag]
            min_repetition_rss [class min_repetition_rss]
        """
        
        filter_factor       = kwargs.get('filter_rss_factor',self.filter_factor) #30% lower than mode
        min_repetition_flag = kwargs.get('min_repetition',self.min_repetition_flag)
        min_repetition_rss  = kwargs.get('min_repetition_rss',self.min_repetition_rss)
        verbose = kwargs.get('verbose',False)    
            
        self.filtered_rss   = list()
        
        for rss in self.rss:
            if (min_repetition_flag) and (len(rss) < min_repetition_rss):
                if verbose:
                    print(len(rss))
                filtered_rss = [self.min_rss,]
            else:
                unique_rss = list(set(rss)) #list of unique rss values 
                count_rss = [rss.count(u_rss) for u_rss in unique_rss] # number of times each unique rss appears
                # filter those that appearh less than factor times the mode
                mode_rss = np.max(count_rss)
                filtered_rss = [r for r,c in zip(unique_rss,count_rss) for i in range(c) if c > filter_factor*mode_rss] 
            self.filtered_rss.append(filtered_rss)
            

           
    def find_pose(self,poses):
        """
        Given a ROS Trajectory message, incorporates x,y,angle position information
        Input
            poses: ROS Trajectory message
        """
        #TODO: assert poses is list of type(Pose)
        #TODO: when called from ProcessedData, speed up by passing poses_time and not recomputing each time

        poses_time   = [long(p.header.stamp.secs*1e9+p.header.stamp.nsecs) for p in poses]
        index_after  = poses_time.index(next(x for x in poses_time if x > self.time))
        index_before = index_after-1


        if self.poses_type == 'PoseWithCovarianceStamped':
            xa = poses[index_after].pose.pose.position.x
            ya = poses[index_after].pose.pose.position.y
            aa = 2*np.arctan2(poses[index_after].pose.pose.orientation.z,
                              poses[index_after].pose.pose.orientation.w)
        
            xb = poses[index_before].pose.pose.position.x
            yb = poses[index_before].pose.pose.position.y
            ab = 2*np.arctan2(poses[index_before].pose.pose.orientation.z,
                              poses[index_before].pose.pose.orientation.w)

        elif self.poses_type == 'Path':
            xa = poses[index_after].pose.position.x
            ya = poses[index_after].pose.position.y
            aa = 2*np.arctan2(poses[index_after].pose.orientation.z,
                              poses[index_after].pose.orientation.w)
        
            xb = poses[index_before].pose.position.x
            yb = poses[index_before].pose.position.y
            ab = 2*np.arctan2(poses[index_before].pose.orientation.z,
                              poses[index_before].pose.orientation.w)

        #interpolation
        dt = float(poses_time[index_after] - poses_time[index_before])
        dta = float((poses_time[index_after] - self.time)/dt)
        dtb = float((self.time - poses_time[index_before])/dt)
        self.pose = [dta*xb+dtb*xa,dta*yb+dtb*ya,(ab+dtb*((aa-ab+np.pi)%(2*np.pi)-np.pi)+np.pi)%(2*np.pi)-np.pi]
        #(aa-ab+np.pi)%(2*np.pi)-np.pi ensure that aa-ab uses the shortest arc from aa to ab 
        #e.g., 1deg - 359deg -> (1-359+180)%(360)-180 = 2deg instead of 1-359 = -358 deg  
        #dta*ab+dtb*aa -> dta*ab+dtb*(aa-ab+ab) -> dta*ab+dtb*((aa-ab+np.pi)%(2*np.pi)-np.pi +ab)
        #dta*ab+dtb*ab+dtb*((aa-ab+np.pi)%(2*np.pi)-np.pi) =>[dta+dtb=1]=> ab+dtb*((aa-ab+np.pi)%(2*np.pi)-np.pi)
        
    def pose_distance(self,other):
        """
        Checks the distance (linear) between its pose and another measurement's pose
        """
        assert type(other) is type(self)
        if (self.pose is None) or (self.pose is None):
            return np.inf#, -1
        else:
            d_lin = ((self.pose[0]-other.pose[0])**2 + (self.pose[1]-other.pose[1])**2)**.5
            #d_ang = np.abs((self.pose[2]-other.pose[2]+np.pi)%(2*np.pi)-np.pi)
            return d_lin#, d_ang
        
    def rss_mean(self,index='all',filtered=True):
        """
        Outputs mean of rss measurements
        index='all' computes the mean value for all measurements
        """
        if filtered:
            rss = self.filtered_rss
        else:
            rss = self.rss
                
        if index=='all':
            selected_rss = rss
        else:
            selected_rss = [rss[i] for i in index]

        return np.asarray([np.mean(s) for s in selected_rss])

    def rss_var(self,index='all',filtered=True):
        """
        Outputs variance of rss measurements
        index='all' computes the mean value for all measurements
        """
        if filtered:
            rss = self.filtered_rss
        else:
            rss = self.rss
                
        if index=='all':
            selected_rss = rss
        else:
            selected_rss = [rss[i] for i in index]
        
        #output rss variance if len(rss) > 1 if not output the default variance 
        #var = np.asarray([np.var(s) if (len(s)>1) else self.prior_var for s in selected_rss])
        #if each sample is considered to be taken from a noisy function with gaussian noise of the same variance,
        #then the variance of a sequence of samples becomes prior_noise_variace/n_samples        
        #if rss_len is zero, the prior variance is assigned hence, in practice the min_rss_len = 1        
        var = self.prior_var/np.clip(self.rss_len(index=index,filtered=filtered),1,np.inf)        

        return var #np.clip(var,2e-5,np.inf)

    def rss_len(self,index='all',filtered=True):
        """
        Outputs variance of rss measurements
        index='all' computes the mean value for all measurements
        """
        if filtered:
            rss = self.filtered_rss
        else:
            rss = self.rss
                
        if index=='all':
            selected_rss = rss
        else:
            selected_rss = [rss[i] for i in index]
        
        #output len(rss) if it is higher than 1, if 1 output 0 -> in the filtering process, instead of eliminating entries
        #a single min_rss value is put, so len=1 should be len=0
        return np.asarray([len(s) if len(s) > 1 else 0 for s in selected_rss])
    
    
    
    def transform2vector(self,general_dict,fun):
        """
        Outputs a vector for training and prediction, using a mac_dict of all sensed vectors in the area
        """
        if fun=='mean':
            rss_fun = self.rss_mean(index='all')
            default_val = self.min_rss
        elif fun=='var':
            rss_fun = self.rss_var(index='all')
            default_val = self.prior_var
        elif fun=='len':
            rss_fun = self.rss_len(index='all')
            default_val = 0
        else:
            print('Incorrect key: ', str(fun))
            
        vector = default_val*np.ones(len(general_dict))
        for mac,index in self.mac_dict.iteritems(): #for all macs in self
            try:
                vector[general_dict[mac]] = rss_fun[index] #put rss value in the adequate index from general_dict
            except:
                pass
            
        return vector
    
    #overloading addition
    def __add__(self,other):
        """
        Overloading __add__ to allow the addition of measurements
        When two measurements are added, all their signals are convined. 
        The time is set to the average of both measurements.
        If the data comes from different access points, the new* access points are added to the dictionary
        If the data comes from already sensed access points, the new measurements are appended to current data
        """
        assert type(other) is type(self)
        result = copy.deepcopy(self)
        
        result.time = long(result.time*.5)+long(other.time*.5)
        
        #finding mac_addresses not sensed in self
        to_add_keys = [mac for mac in other.mac_dict if mac not in result.mac_dict]
        old_index   = [other.mac_dict[key] for key in to_add_keys]
        to_add_freq = [other.freq[ind] for ind in old_index]
        
        #updating
        result.mac_dict.update({to_add_keys[i]:(i+result.nap) for i in range(len(to_add_keys))})
        result.freq = result.freq + to_add_freq 
        result.nap = len(result.freq)
        
        result.rss = result.rss + [[] for i in range(len(to_add_keys))]
        for other_mac,other_index in other.mac_dict.iteritems():
            index = result.mac_dict[other_mac]
            result.rss[index] = result.rss[index]+other.rss[other_index]
            
        # if position is known


        if result.pose is None:
            if other.pose is None:
                pass #No measure knows its position
            else:
                result.pose = other.pose #self.pose is not known but other is known
        else:
            if other.pose is None:
                pass #pose is knonw and other.pose is not, so pose remains the same
            else:
                result.pose[0] = .5*result.pose[0]+.5*other.pose[0]
                result.pose[1] = .5*result.pose[1]+.5*other.pose[1]
                result.pose[2] = (result.pose[2]+.5*((other.pose[2]-result.pose[2]+np.pi)%(2*np.pi)-np.pi)+np.pi)%(2*np.pi)-np.pi

        return result
    
    def __str__(self):
        """
        Overloading __str__ to make print statement meaningful
        
        <format>
        Mac Frequency RSS
        """
        to_print = 'Time: {} ns\n'.format(self.time)
        if self.pose is not None:
            to_print+='Pose: {}\n'.format(str(self.pose))
        to_print += '{}  {}  {}\n'.format('mac address'.ljust(17),'Freq'.ljust(4),'RSS')
        for mac,index in self.mac_dict.iteritems():
            to_print += '{}  {}  {}\n'.format(mac.ljust(17),self.freq[index],self.rss[index])       

        return to_print


############################################## PROCESSED DATA CLASS ##############################################
class ProcessedData():
    def __init__(self,raw_measurements,**kwargs):
        """
        Class that groups all Measurement taken in an environment and provides further filtering and processing
        Input
            raw_measurements: list of ROS RssData measurements
        
        Optional Parameters [default]
            poses [None]          : ROS Trajectory message with the positions where rss data was taken
            other_mac_dict [None] : If available, compute_data is performed using this dict instead of self generated mac dict
            filter_min_distance [0.05]   : Minimum distance in meter between data points. If two measurements are too close
                                           they are fused together
            filter_fuse_measurements [5] : Fuses measurements together (performed after filter_min_distance, so effective 
                                           min distance is filter_fuse_measurement*filter_min_distance)       
            filter_min_points_per_AP [10]: Minimum number of different data points where an Access Point must be available
             
        Functions
            filter_refilter
            filter_distance
            filter_fuse
            filter_macs
            create_all_mac_dict
            super_macs
            compute_data
 
        """
        self.poses      = kwargs.get('poses',None)
        poses_type      = kwargs.get('poses_type','PoseWithCovarianceStamped')        
        other_mac_dict  = kwargs.get('other_mac_dict',None)
        all_filters_on  = kwargs.get('all_filters_on',True) #changes the default flag for filters on
        warn_pose_off   = kwargs.get('warn_pose_off',False)
     
        self.measurements = [Measurement(m,poses=self.poses,poses_type=poses_type) for m in raw_measurements]

        #Filter managemente
        flag_min_distance       = kwargs.get('flag_min_distance',all_filters_on)
        flag_fuse_measurements  = kwargs.get('flag_fuse_measurements',all_filters_on)
        flag_min_points_per_AP  = kwargs.get('flag_fuse_min_points_per_AP',all_filters_on)
        flag_pose_distance      = kwargs.get('flag_pose_distance',all_filters_on)

        #MAYBE TODO: do not turn all filters_of
        #filter_distance
        if flag_min_distance:
            if not self.poses is None:
                self.filter_min_distance      = kwargs.get('filter_min_distance',0.05) #min distance between points in [m]
                #filter_fuse 
            else:
                if not warn_pose_off:
                    eprint('[WARN] No pose will be calculated')

        if flag_fuse_measurements:
            self.filter_fuse_measurements = kwargs.get('filter_fuse_measurements',5) #number of consecutive measurements to fuse
            #filter_macs
            self.filter_min_points_per_AP = kwargs.get('min_points_per_AP',10) #min number of points each AP must have
        
            #Filters
        if flag_pose_distance:
            if self.poses is not None:
                filter_flag = self.filter_distance() #Filter1: If pose of measurements is too close, fuse measurements
            
            filter_flag = self.filter_fuse()     #Filter2: Join measurements to not have too few measurements per point
            filter_flag = self.filter_refilter() #Re-Filter fused measurements
        
        self.nm = len(self.measurements)     #number of valid measurements
        
        if other_mac_dict is None:
            self.create_all_mac_dict()           #create dict of all macs that have appeared
            filter_flag = self.filter_macs()     #Filter3: Filter mac if it does not appear in a minimum number of locations
            #TODO: fuse macs which are similar
        else:
            self.all_mac_dict = other_mac_dict

        self.compute_data()

    #FILTERS
    def filter_refilter(self):
        """
        Executes Measurement class filters again, after measurements have been fused
        """
        for measurement in self.measurements:
            measurement.filter_rss(min_repetition=True)
        return True
    
    def filter_distance(self,**kwargs):
        """
        Fuses measurements so there is at least min_distance between consecutive measurements
        Optional Paramteres [default]
            min_distance [class filter_min_distance]: Minimum distance enforced between consecutive points   
        """        
        min_distance = kwargs.get('min_distance',self.filter_min_distance)
        
        m_out = list()
        for m in self.measurements:
            distance = np.inf
            if bool(m_out): #checks M is not empty
                distance = m.pose_distance(m_out[-1])
    
            if distance < min_distance: #if distance between samples is smaller than min_distance, fuse points
                m_out[-1] = m_out[-1]+m
                
            else: #else add the measurement to the list if its pose is known
                if m.pose is not None:
                    m_out.append(m)
        self.measurements = m_out
        return True
        
    def filter_fuse(self,**kwargs):
        """
        Fuses fuse_measurements consecutive measurements
       
        Optional Parameters [default]:
            fuse_measurements [class filter_fuse_measurements] : Number of consecutive measurements to filter        
        """        
        fuse_measurements = kwargs.get('fuse_measurements',self.filter_fuse_measurements)
        m_out = list()
        for i in range(len(self.measurements)):
            if i%fuse_measurements == 0:
                m_out.append(self.measurements[i])
            else:
                m_out[-1] = m_out[-1] + self.measurements[i]
        self.measurements = m_out
        return True
       
    def create_all_mac_dict(self):
        """
        Creates a single mac_dict for all measurements
        """
        temp = [mac for i in range(self.nm) for mac in self.measurements[i].mac_dict]
        mac_keys = list(set(temp)) #eliminates mac duplicates
        self.all_mac_dict = {mac_keys[i]:i for i in range(len(mac_keys))} #create dictionary
    
    def filter_macs(self,**kwargs):
        """
        Filters macs which do not have a value in least min_points_per_AP locations

        Optional Parameters [default]:
            min_points_per_AP [class filter_min_points_per_AP]: Minimum number of locations a valid mac should be heard
        """
        min_points_per_AP = kwargs.get('min_points_per_AP',self.filter_min_points_per_AP)
        
        self.compute_data()
        del_list = list() #list with macs to delete
        n = self.data['Y'].shape[1]
        
        for mac,index in self.all_mac_dict.iteritems():
            count = np.count_nonzero(self.data['Y'][:,index])
            if count < min_points_per_AP: 
                del_list.append(mac)
        
        filtered_macs = [mac for mac in self.all_mac_dict.keys() if not mac in del_list]
        self.all_mac_dict = {mac:i for i,mac in enumerate(filtered_macs)}
        
        return True
    
    def super_macs(self,**kwargs):
        pass
        
    def compute_data(self):
        """
        Computes the data output of the class. 
        Output
            data: Dictionary with keys:
                X: [px2] Locations (x-y)
                Y: [pxm] Rss measurements mean
                Var: [pxm] Rss measurements variance 
                N: [pxm] Number of rss measurements 
                *All outputs are 2d arrays with [] the dimension p:#positions, m:#access points  
        """
        if not self.poses is None:     
            dataX   = np.asarray([m.pose for m in self.measurements])
        else:
            dataX = None
        
        dataY   = np.asarray([m.transform2vector(self.all_mac_dict,'mean') for m in self.measurements])/100+1. #scaled Y
        dataVar = np.asarray([m.transform2vector(self.all_mac_dict,'var') for m in self.measurements])/100**2. #scaled Var
        dataN   = np.asarray([m.transform2vector(self.all_mac_dict,'len') for m in self.measurements])
        if dataX is None:
            self.data = {'X':None, 'Y':dataY,'Var':dataVar,'n':dataN}
        else:        
            self.data = {'X':dataX[:,:2],'Y':dataY,'Var':dataVar,'n':dataN}


    def save(self,filepath='last_data.p'):
        """
        Pickle current model. 
        If no filepath is given, it is saved at <currentpath>/last_data.p
        """
        try: 
            import cPickle as pickle
        except ImportError:
            import pickle

        with open(filepath, 'wb') as f:
            pickle.dump(self.__dict__,f,2)
        
        return True

    def load(self,filepath='last_data.p'):
        """
        Load a previously pickled model from filepath. 
        If no filepath is given, <currentpath>/lastmodel
        """
        try: 
            import cPickle as pickle
        except ImportError:
            import pickle

        with open(filepath, 'rb') as f:
            tmp_dict = pickle.load(f)
        
        self.__dict__.update(tmp_dict)        
        
        return True



    #TODO: Add to ProccessedData
    """ 
    def super_macs(pd,**kwargs):
        #TODO: improve code
        min_common_points = kwargs.get('min_common_points',10)
        max_avg_distance  = kwargs.get('max_avg_distance',1/100.)
        max_max_distance  = kwargs.get('max_max_distance',5/100.)

        n = pd.data['Y'].shape[1]

        #First condition = points in common between 2 AP must be higher than min_common_points
        #Second condition similarity between common points (avg difference, max difference)
        CORR = np.zeros((n,n)) #Upper triangular matrix with the number of max difference between vectors Y
        
        for i in range(n):
            for j in range(i,n):
                di = pd.data['Y'][:,i]
                dj = pd.data['Y'][:,j]
                
                index = np.where(di*dj!=0)[0]
                if index.shape[0] >= min_common_points: #First condition
                    e  = np.abs(di[index]-dj[index])
                    da = np.mean(e)
                    dm = np.max(e)
                    #d = np.sum(di[index]*dj[index])/(0.5*np.sum(di[index]**2+dj[index]**2)) #Correlation index
                else:
                    da = 10
                    dm = 10
                if da<max_avg_distance and dm<max_max_distance:
                    CORR[i,j] = 1
                
        ## recreate the mac
        labels = -np.ones(CORR.shape[0]).astype(int)
        next_label = 0

        for index,c in enumerate(CORR):
            c_index = np.nonzero(c)[0]
            
            #if c_index.shape[0]==0 it means the ap does not have min number of point even with itself
            #delete mac by leaving -1
            if c_index.shape[0] > 0: 
                possible_labels = list(set(labels[c_index])) #unique elements list
                #print('  possible labels: ',possible_labels)
                try:
                    possible_labels.remove(-1) #if -1 (no label) is present, remove
                except:
                    pass
        
                if len(possible_labels)==0: #no possible labels found, assign new label
                    labels[c_index] = next_label
            
                    #print('  ',index, next_label)
                    next_label += 1
                    #print('xxxx',next_label)
                elif len(possible_labels)==1:
                    labels[index] = possible_labels[0]
                    #print('  ',index, possible_labels[0])
                else:
                    min_label = min(possible_labels)
                    labels[index] = min_label
                    possible_labels.remove(min_label)
                    #print('  ',index, min_label, 'others: ',possible_labels)
                    for other_label in possible_labels:
                        #print('****',other_labels,' to ',min_label)
                        labels[labels==other_label] = min_label #force the other labels to be the same as min_label

        #for l in set(labels):
        #    print(l,[i for i,x in enumerate(labels) if labels[i]==l])  
        
        return labels
    """
