#!/usr/bin/python

import rospy
from network_utils.network import Network
from network_utils.network import Routing
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion 
from enum import IntEnum
import numpy as np
import tf

class COMMANDS(IntEnum):    
    SETINITALPOSE         = 0


class Robot:
    def __init__(self):
        
        home = os.path.expanduser("~")
        self.config_data = self.readConfig(home+'/NetworkedRobotsProject/configs/data.yaml')

        self.network              = Network(id=-1, broadcast_addr = self.config_data['broadcast_address'], port = self.config_data['configuration_port'])
        self.network.addCommandCallback(self.receiveCommand)
        self.setpose_pub          = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.getPose)
        self.routing = Routing('teste4', home+'/NetworkedRobotsProject/configs/data.yaml')
        self.id = self.routing.getID()



    
    def readConfig(self, config_file):
        with open(config_file, 'r') as stream:
            return yaml.load(stream)['configs']

    def getPose(self, msg):
        print(msg)
        pass

    def receiveCommand(self, command):

        if (command['command_id'] == COMMANDS.SETINITALPOSE):
            if (command['robot_id'] != self.id):
                initial_pose = command['initial_pose']
                angle        = command['direction']

                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = "/odom"
                pose.pose.pose.position.x = initial_pose[0]
                pose.pose.pose.position.y = initial_pose[1]


                q = tf.transformations.quaternion_from_euler(0, 0, angle)
                pose.pose.pose.orientation = Quaternion(*q)

                p_cov = np.array([0.0]*36).reshape(6,6)
                pose.pose.covariance = tuple(p_cov.ravel().tolist())

                self.setpose_pub.publish(pose)

        print(command)
        return



if __name__ == "__main__":
    rospy.init_node('set_pose', anonymous=True)
    robot = Robot()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
