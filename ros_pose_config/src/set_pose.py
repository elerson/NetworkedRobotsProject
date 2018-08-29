#!/usr/bin/python

import rospy
from network_utils.network import Network
from network_utils.Routing import Routing
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion 
from enum import IntEnum
import numpy as np
import tf
import os
import subprocess as sub


import yaml
class COMMANDS(IntEnum):    
    SETINITALPOSE         = 0
    STARTDEPLOYMENT       = 1
    EXECCOMMAND           = 2
    SETGOAL               = 3


class Robot:
    def __init__(self):
        
        home = os.path.expanduser("~")
        self.config_data = self.readConfig(home+'/NetworkedRobotsProject/configs/data_real.yaml')

        self.map_resolution = 0.05
        self.height         = 618


        self.network              = Network(id=-1, broadcast_addr = self.config_data['broadcast_address'], port = self.config_data['configuration_port'])
        self.network.addCommandCallback(self.receiveCommand)
        self.setpose_pub          = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.setgoal_pub          = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.getGoal)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.getPose)
        self.routing = Routing('teste4', home+'/NetworkedRobotsProject/configs/data_real.yaml', 'ra0')
        self.id = self.routing.getID()
        
        self.position            = {}
        self.position['id']      = self.id
        self.position['position']= (0.0, 0.0, 0.0)
        self.position['cov']     = (0.1, 0.0, 0.0, 0.1)

        rospy.Timer(rospy.Duration(1.5), self.pose_callback)


    def getPose(self, Pose):

        orientation = (
            Pose.pose.pose.orientation.x,
            Pose.pose.pose.orientation.y,
            Pose.pose.pose.orientation.z,
            Pose.pose.pose.orientation.w)
        orientation_euler = tf.transformations.euler_from_quaternion(orientation)

        #the variance for the kalman filter
        xx = Pose.pose.covariance[0]
        xy = Pose.pose.covariance[1]
        yx = Pose.pose.covariance[6]
        yy = Pose.pose.covariance[7]

        self.position['position'] = (Pose.pose.pose.position.x/self.map_resolution, self.height- Pose.pose.pose.position.y/self.map_resolution, orientation_euler[2])
        self.position['cov']      = (xx,xy,yx,yy)



    def pose_callback(self, param):
        self.network.sendMessage(self.position)
        #print ('send pose')

    def readConfig(self, config_file):
        with open(config_file, 'r') as stream:
            return yaml.load(stream)['configs']

    def getGoal(self, msg):
        print(msg)
        pass

    def receiveCommand(self, command):

        print(command['command'] == COMMANDS.SETINITALPOSE, command['robot_id'] == self.id, self.id, command['command'], command['robot_id'])
        if (command['command'] == COMMANDS.SETINITALPOSE):
            if (command['robot_id'] == self.id):
                initial_pose = command['initial_pose']
                angle        = command['direction']

                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = "map"
                pose.pose.pose.position.x = initial_pose[0]
                pose.pose.pose.position.y = initial_pose[1]


                q = tf.transformations.quaternion_from_euler(0, 0, angle)
                pose.pose.pose.orientation = Quaternion(*q)
                cov_ = np.array([0.0]*36)
                cov_[0]  = 0.10
                cov_[7]  = 0.10
                cov_[35] = 0.068
                print(cov_)
                p_cov = cov_.reshape(6,6)

                pose.pose.covariance = tuple(p_cov.ravel().tolist())

                self.setpose_pub.publish(pose)

                print(command)

        
        if (command['command'] == COMMANDS.SETGOAL):
            if (command['robot_id'] == self.id):
                goal = command['goal']
                angle = command['direction']

                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = goal[0]
                pose.pose.position.y = goal[1]


                q = tf.transformations.quaternion_from_euler(0, 0, angle)
                pose.pose.orientation = Quaternion(*q)
                self.setgoal_pub.publish(pose)

                print(command)
        return



if __name__ == "__main__":
    rospy.init_node('set_pose', anonymous=True)
    robot = Robot()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()

    sub.Popen(('kill', '-9', str(os.getpid())))
