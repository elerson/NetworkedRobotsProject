#!/usr/bin/python
import os
import rospy
from network import Network
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion 
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import MapMetaData

import datetime


rss_command = ''' cat /proc/net/wireless | awk 'END { print $4 }'| sed 's/\.$//' '''
class Robot:
    def __init__(self):

        rospy.init_node('robot_data_gettering', anonymous=True)
        #print("ROBOT")
        self.map_resolution = 0.025

        self.network = Network()
        id = 0 #rospy.get_param("~id")

        self.id = int(id)
        self.position = {}
        self.position['id'] = self.id
        self.position['position'] = (0.0, 0.0, 0.0)
        self.position['covariance'] = (0.0, 0.0)
        self.height = 0

        self.send_position_time_diff = rospy.get_param("~pose_send_time", 0.05)
        self.send_position_time = rospy.get_time() 


        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.getPose)
        rospy.Subscriber("/map_metadata", MapMetaData, self.getMap)
        

    def getMap(self, MapData):

        self.map_resolution = MapData.resolution
        self.height = MapData.height


    def getPose(self, Pose):
        orientation = (
            Pose.pose.pose.orientation.x,
            Pose.pose.pose.orientation.y,
            Pose.pose.pose.orientation.z,
            Pose.pose.pose.orientation.w)
        orientation_euler = tf.transformations.euler_from_quaternion(orientation)

        self.position['position'] = (Pose.pose.pose.position.x/self.map_resolution, self.height- Pose.pose.pose.position.y/self.map_resolution, orientation_euler[2])
        self.position['covariance'] = (Pose.pose.covariance[0], Pose.pose.covariance[7])

        #print(self.position['position'])
        #avoid to flood the network with messages
        #if(rospy.get_time() - self.send_position_time > self.send_position_time_diff or not self.initialized):
        #    self.network.sendMessage(self.position)
        #    self.send_position_time__ = rospy.get_time()

        self.initialized = True


    def getRSS(self):
        return os.popen(rss_command).read()[0:3]


if __name__ == "__main__":
    robot = Robot()
    rate = rospy.Rate(25.0)
    with open("/tmp/output.txt", "w") as text_file:

        while not rospy.is_shutdown():
            rss = robot.getRSS()
            data = str(datetime.datetime.now().time()) + ';'
            data +=  str(robot.position['position'])+ ";" + str(robot.position['covariance']) + "\n"
            print(data)
            text_file.write(data)
            rate.sleep()
       