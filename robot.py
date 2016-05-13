#!/usr/bin/env python

import rospy
import math
import random
from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
from read_config import read_config
from nav_msgs.msg import OccupancyGrid
import helper_functions
from map_utils import Map

from sklearn.neighbors import KDTree

PARTICLE_NUM = 800

class Robot():
    def __init__(self):
        """Read config file and setup ROS things"""
        self.config = read_config()         
        rospy.init_node("robot")

        #Subscribers for map and laser scans
        self.map_topic = rospy.Subscriber(
            "/map",
            OccupancyGrid,
            self.map_callback
        )
        self.laser_scan = rospy.Subscriber(
            "/base_scan",
            LaserScan,
            self.laser_callback
        )

        #Publishers
        self.cloud_topic = rospy.Publisher(
            "/particlecloud",
            PoseArray,
            queue_size = 10
        )
        self.likelihood_topic = rospy.Publisher(
            "/likelihood_field",
            OccupancyGrid,
            queue_size = 10,
            latch = True
        )
        self.result_update = rospy.Publisher(
            "/result_update",
            Bool,
            queue_size = 10
        )
        self.sim_complete = rospy.Publisher(
            "/sim_complete",
            Bool,
            queue_size = 10
        )

    def map_callback(self, data):
        self.map = Map(data)
        print "map called"

        #Initialize particles
        self.pose_array = PoseArray()
        self.pose_array.header.stamp = rospy.Time.now()
        self.pose_array.header.frame_id = 'map'
        self.pose_array.poses = []
        for n in range(PARTICLE_NUM):
            randX = random.randint(0, self.map.width)
            randY = random.randint(0, self.map.height)
            randTheta = random.uniform(0, 2 * math.pi)
            self.pose_array.poses.append(helper_functions.get_pose(randX, randY, randTheta))
        #Publish initial particles
        self.cloud_topic.publish(self.pose_array)

        #Construct likelihood field
        
        kdt = KDTree()

    def laser_callback(self, data):
        #this is just for now
        self.cloud_topic.publish(self.pose_array)


if __name__ == '__main__':
    rob = Robot()
    rospy.spin();
