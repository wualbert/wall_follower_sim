#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from utilities import *

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    MARKER_TOPIC = rospy.get_param("wall_follower/marker_topic")
    STEERING_ANGLE = rospy.get_param("wall_follower/max_wheel_angle")
    def __init__(self):
        #init publisher
        self.marker_pub = rospy.Publisher(self.MARKER_TOPIC,Marker,queue_size = 10)
        self.control_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        #init subscriber
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback)
        return

    def scan_callback(self,laser_scan_data):
        #process data to obtain line to follow
        lines= self.scan_to_lines(laser_scan_data)
        if lines is None:
            #no line detected
            self.publish_control(lines, zero=True)
            return
        #print(m,b)
        self.publish_control(lines)
        #self.publish_lines(lines)
        return
    
    def scan_to_lines(self, laser_scan_data):
        ro_points = self.partition_points(laser_scan_data)
        cartesian_points = self.range_to_cartesian(ro_points)
        lines = self.cartesian_to_lines(cartesian_points)
        return lines

    def partition_points(self,laser_scan_data):
        range_orientation_points = np.asarray(laser_scan_data.ranges)
        orientation_points = np.arange(laser_scan_data.angle_min, \
                                        laser_scan_data.angle_max+laser_scan_data.angle_increment/2,\
                                        laser_scan_data.angle_increment)
        range_orientation_points = np.vstack([range_orientation_points, orientation_points]).T
        return self.preprocess_points(laser_scan_data, range_orientation_points)    

       
    def range_to_cartesian(self,range_orientation_points):
        num_of_points = range_orientation_points.shape[0]
        cartesian_points = np.zeros([num_of_points,2])
        for i, r in enumerate(range_orientation_points[:,0]):
                #calculate x
                cartesian_points[i,0]=r*np.cos(range_orientation_points[i,1])
                #calculate y
                cartesian_points[i,1]=r*np.sin(range_orientation_points[i,1])
        return cartesian_points
    
    def preprocess_points(self, laser_scan_data,range_orientation_points):
        #prune impossible points
        new_rop = []
        for rop in range_orientation_points:
            if self.SIDE==1:
                #only look to the left
                if rop[1]<=min(np.pi, laser_scan_data.angle_max) and \
                    rop[1] >= 0:
                        new_rop.append(rop)
            elif self.SIDE == -1:
                if rop[1]>=max(-np.pi, laser_scan_data.angle_min) and \
                    rop[1]<=0:
                        new_rop.append(rop)
        return np.asarray(new_rop)

    def cartesian_to_lines(self,cartesian_points):
        lines = hough_line_detection(cartesian_points)
        if lines is None:
            #no line detected
            return lines
        #returns closest line
        return lines
   
    def publish_control(self,wall_lines, zero=False):
        if zero:
            control_action = AckermannDriveStamped()
            #set header
            control_action.header.stamp = rospy.rostime.Time().now()
            #fixed velocity and heading
            control_action.drive.speed = self.VELOCITY
            self.control_pub.publish(control_action)
            return True
        #choose first of all lines
        wall_line = wall_lines[0,:]
        assert(len(wall_line)==2)    #mx+b form
        control_action = AckermannDriveStamped()
        #set header
        control_action.header.stamp = rospy.rostime.Time().now()
        #calculate control action
        control_action.drive.steering_angle_velocity = 1
        control_action.drive.speed = self.VELOCITY
        reference_line = self.compute_reference_line(wall_line)
        self.publish_lines([reference_line], color=[1,0,1])
        #simple P steering
        control_action.drive.steering_angle = self.compute_pd_action(reference_line)
        #publish
        self.control_pub.publish(control_action)
        #print(control_action)
        return True
    
    def publish_lines(self, lines, frame='laser', color = [1,1,0]):
        for line in lines:
            marker = Marker()
            marker.header.stamp = rospy.rostime.Time().now()
            marker.header.frame_id = frame
            marker.type = Marker.LINE_STRIP
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.
            #compute line
            p1 = Point()
            p1.x = 0.
            p1.y = line[1]
            p2 = Point()
            p2.x = -line[1]/line[0]
            p2.y = 0.
            marker.points.append(p1)
            marker.points.append(p2)

            self.marker_pub.publish(marker)

    def compute_reference_line(self, wall_line):
        #given a wall, compute the reference line to follow
        assert(len(wall_line)==2)   #mx+b form
        if self.SIDE == 1:
            #wall is on the left
            return np.asarray([wall_line[0], \
                    wall_line[1]-self.DESIRED_DISTANCE*np.sqrt(1+wall_line[0]**2)])
        else:
            #wall is on the right
            return np.asarray([wall_line[0], \
                    wall_line[1]+self.DESIRED_DISTANCE*np.sqrt(1+wall_line[0]**2)])
    
    def compute_pd_action(self, wall_line):
        steering_angle = 0
        dist = line_to_point_distance(wall_line)
        dist_err = dist-self.DESIRED_DISTANCE*self.SIDE
        p_gain = 1
        #print('wl',wall_line)
        #print(dist, dist_err)
        steering_angle += p_gain*dist_err
        steering_angle = max(-self.STEERING_ANGLE,min(self.STEERING_ANGLE,steering_angle))
        return steering_angle

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
