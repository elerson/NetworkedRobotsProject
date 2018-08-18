#!/usr/bin/python
import os
import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion 
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import MapMetaData
from network_utils.rssmeasure_v2 import RSSMeasure
from network_utils.rssi_kalman import RSSIKalmanFilter
import numpy as np

import datetime

config_file = "/home/elerson/NetworkedRobotsProject/configs/data.yaml"
measurement_id = 2

class Robot:
    def __init__(self):

        rospy.init_node('robot_data_gettering', anonymous=True)



        #print("ROBOT")
        self.map_resolution = 0.025

        #self.network = Network()
        id = 0 #rospy.get_param("~id")

        self.id = int(id)
        self.position = {}
        self.position['id'] = self.id
        self.position['position'] = (0.0, 0.0, 0.0)
        self.position['covariance'] = (0.0, 0.0)
        self.height = 0
        self.metric_kalmam      =  RSSIKalmanFilter(3, [-40.0, 3.5], 10.0, 4.0, True)
        self.covariance         = np.matrix([[0.0, 0.0], [0.0, 0.0]])

        self.send_position_time_diff = rospy.get_param("~pose_send_time", 0.05)
        self.send_position_time = rospy.get_time()
        self.config_file        = config_file
        self.rss_measure        = RSSMeasure('teste4', self.config_file)
        self.config             = self.readConfig(config_file)

        self.tree_file          = self.config['configs']['treefile']
        self.tree               = Tree(self.tree_file)


        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.getPose)
        rospy.Subscriber("/map_metadata", MapMetaData, self.getMap)

    def readConfig(self, config_file):
        with open(config_file, 'r') as stream:
            return yaml.load(stream)

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

    def addMeasurementKalman(self, rss):

            real_distance    = self.getDistance(self.position['position'], self.getPositionByID(measurement_id))*self.map_resolution
            real_metric      = rss

            m_var = 4.0
            position = self.getPositionByID(measurement_id)
            x = abs(self.position['position'][0] - position[0])*self.map_resolution
            y = abs(self.position['position'][1] - position[1])*self.map_resolution

            gamma = self.metric_kalman.getGamma()

            d = np.matrix([[10*x*gamma/(x**2 + y**2), 10*y*gamma/(x**2 + y**2)]])
            measurement_var = np.dot(np.dot(d,self.covariance),d.T)[0,0] + m_var

            self.metric_kalman.setMeasurmentVar(measurement_var)
            self.metric_kalman.addMeasurement(real_distance, real_metric)

    def getDistanceFrom(self):
        real_distance    = self.getDistance(self.position['position'], self.getPositionByID(measurement_id))*self.map_resolution
        return real_distance

    def getRSSParameters(self):
        return self.metric_kalman.getResult()

    def getPositionByID(self, id):
        #print(self.network.rcv_data)
        ##Client or tree junctions id
        if(id < self.robots_ids_start):
            position = self.tree.graph_vertex_position[id]
            return position
        else:
            return self.network.getData(id)['position']


if __name__ == "__main__":
    robot = Robot()
    rate = rospy.Rate(100.0)
    
    with open("/tmp/output.txt", "w") as text_file:

        while not rospy.is_shutdown():

            rss = robot.rss_measure.getMeasurement(measurement_id)
            robot.addMeasurementKalman(rss)
            distance = robot.getDistanceFrom()

            estimated_params = robot.getRSSParameters()
            data = str(datetime.datetime.now().time()) + ";" + str(rss) +";"+ str(distance) +";" + str(estimated_params[0]) +";"+ str(estimated_params[1]) +';'
            data +=  str(robot.position['position'])+ ";" + str(robot.position['covariance']) + "\n"
            print(data)
            text_file.write(data)
            rate.sleep()
       
