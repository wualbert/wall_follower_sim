#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):
        #init publisher
        rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        #init subscriber
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback)
        return

    def scan_callback(self,laser_scan_data):
        #process data to obtain line to follow
        m,b = self.scan_to_line(laser_scan_data)
        print(m,b)
        self.compute_control(m,b)
        return
    
    def scan_to_line(self, laser_scan_data):
        #least squares approximation of a line
        ro_points = self.partition_points(laser_scan_data)
        cartesian_points = self.range_to_cartesian(ro_points)
        m,b = self.cartesian_to_line(cartesian_points)
        return m,b

    def partition_points(self,laser_scan_data):
        '''
        #drop unnecessary scan data
        useful_range = np.asarray(laser_scan_data.ranges)
        range_data_count = useful_range.shape[0]
        split_point_low = int(np.floor(range_data_count/2))
        split_point_high = int(np.ceil(range_data_count/2))
        cartesian_points = np.zeros((split_point_low,2))
        angles = np.arange()
        assert(abs(self.SIDE)==1)

        #transform to Cartesian in laser frame
        if self.SIDE == 1:
        #keep left side data
            useful_range = useful_range[split_point_high:]
            for i, r in enumerate(useful_range):
                #calculate x
                cartesian_points[i,0]=r*np.cos(laser_scan_data.angle_min+\
                        (i+split_point_high)*laser_scan_data.angle_increment)
                #calculate y
                cartesian_points[i,1]=r*np.sin(laser_scan_data.angle_min+\
                        (i+split_point_high)*laser_scan_data.angle_increment)
        elif self.SIDE == -1:
        #keep right side data
            useful_range = useful_range[:split_point_low]
            for i, r in enumerate(useful_range):
                #calculate x
                cartesian_points[i,0]=r*np.cos(laser_scan_data.angle_min+\
                        i*laser_scan_data.angle_increment)
                #calculate y
                cartesian_points[i,1]=r*np.sin(laser_scan_data.angle_min+\
                        i*laser_scan_data.angle_increment)
        '''
        range_orientation_points = np.asarray(laser_scan_data.ranges)
        orientation_points = np.arange(laser_scan_data.angle_min, \
                                        laser_scan_data.angle_max+laser_scan_data.angle_increment/2,\
                                        laser_scan_data.angle_increment)
        range_orientation_points = np.vstack([range_orientation_points, orientation_points]).T
        return range_orientation_points

       
    def range_to_cartesian(self,range_orientation_points):
        num_of_points = range_orientation_points.shape[0]
        cartesian_points = np.zeros([num_of_points,2])
        for i, r in enumerate(range_orientation_points[:,0]):
                #calculate x
                cartesian_points[i,0]=r*np.cos(range_orientation_points[i,1])
                #calculate y
                cartesian_points[i,1]=r*np.sin(range_orientation_points[i,1])
        print(cartesian_points)
        return cartesian_points

    def cartesian_to_line(self,cartesian_points):
        x = cartesian_points[:,0]
        x = np.vstack([x, np.ones(len(x))]).T
        y = cartesian_points[:,1]
        return np.linalg.lstsq(x,y)[0]

    def compute_control(self,m,b):
       pass 
if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
