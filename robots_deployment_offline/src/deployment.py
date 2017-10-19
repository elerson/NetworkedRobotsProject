#!/usr/bin/python
import rospy
from tree import Tree, TreeSegmention
from scipy.optimize import linear_sum_assignment
from network import Network
import numpy as np
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion 
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import MapMetaData

import tf

        
class Robot:
    def __init__(self):

        rospy.init_node('robot_deployment', anonymous=True)

        self.network = Network()
        id = rospy.get_param("~id")

        self.id = int(id)

        self.position = {}
        self.position['id'] = self.id
        self.position['position'] = (0, 0)
        self.initialized = False



        self.send_position_time_diff = rospy.get_param("~pose_send_time", 0.5)
        self.tree_file = rospy.get_param("~tree_file")
        self.ray = rospy.get_param("~ray", 70)

        self.map_resolution = 0.5
        self.height = 0

        self.send_position_time = rospy.get_time() 

        self.tree = Tree(self.tree_file)
        self.tree_segmention = TreeSegmention(self.tree)

        self.num_robots = 0
        self.deployment_position = []

        self.status = -1


        rospy.Subscriber("/robot_"+str(self.id)+"/amcl_pose", PoseWithCovarianceStamped, self.getPose)
        self.goal_pub = rospy.Publisher("/robot_"+str(self.id)+"/move_base_simple/goal", PoseStamped, queue_size=10)
        rospy.Subscriber("/robot_"+str(self.id)+"/move_base/status", GoalStatusArray, self.getStatus)
        rospy.Subscriber("/map_metadata", MapMetaData, self.getMap)



    def getMap(self, MapData):

        self.map_resolution = MapData.resolution
        self.height = MapData.height



    def getStatus(self, Status):

        if(len(Status.status_list) > 0):
            self.status = Status.status_list[0].status


    def getPose(self, Pose):

        self.position['position'] = (Pose.pose.pose.position.x/self.map_resolution, Pose.pose.pose.position.y/self.map_resolution)
     
        #avoid to flood the network with messages
        if(rospy.get_time() - self.send_position_time > self.send_position_time_diff or not self.initialized):
            self.network.sendMessage(self.position)
            self.send_position_time__ = rospy.get_time()
            if(self.num_robots != len(self.network.rcv_data)):
                self.num_robots = len(self.network.rcv_data)
                self.sendDeployment()
                print(self.deployment_position)

        self.initialized = True
                
               

            
    def sendDeployment(self):

        if(self.status == 1 or self.status == 3 and not self.initialized):
            return

        do_deployment = self.doDeployment()
        if( not do_deployment):
            return 
        print(self.deployment_position)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.deployment_position[0,0]*self.map_resolution
        pose.pose.position.y = self.deployment_position[0,1]*self.map_resolution

        #for debug
        self.position['destination'] = (self.deployment_position[0,0], self.deployment_position[0,1])

        print(pose.pose.position.x, pose.pose.position.y)
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        pose.pose.orientation = Quaternion(*q)
        self.goal_pub.publish(pose)


    def getDistance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)




    #The current robot is always allocated in the first position
    def getAllRobotsPositions(self):

        robot_positions = []
        robot_positions.append(self.position['position'])

        for data_id in self.network.rcv_data:
            if(data_id != self.id):
                position = self.network.rcv_data[data_id]['position']
                robot_positions.append(position)

        return robot_positions
  



    def doDeployment(self):

        tree_allocation_positions = self.tree_segmention.doAllocation(self.ray)
        print(self.position['position'])  
        print(tree_allocation_positions)

        #put the tree positions in the map coordinate system
        for i in range(tree_allocation_positions.shape[0]):
            tree_allocation_positions[i,1] = self.height - tree_allocation_positions[i,1] 

        robots_positions = self.getAllRobotsPositions()

        allocation_id = self.getAllocation(tree_allocation_positions, robots_positions)
        self.deployment_position = tree_allocation_positions[allocation_id,]
        return not allocation_id == -1




    def getAllocation(self, tree_allocation_positions, robots_positions):
        n_robots = len(robots_positions)
        n_allocations = len(tree_allocation_positions)
        cost_matrix = np.zeros((n_robots, n_allocations))

        for r in range(n_robots):
            for t in range(n_allocations):
                cost_matrix[r, t] = self.getDistance(robots_positions[r], np.array(tree_allocation_positions[t])[0])


        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        print(row_ind)
        print(col_ind)

        #the robot is always allocated in position 0
        if( 0 in row_ind ):
            index = row_ind.tolist().index(0)
            return col_ind[index]
        else:
            return -1



if __name__ == "__main__":
    robot = Robot()
    rate = rospy.Rate(1/20.0)
    while not rospy.is_shutdown():
        print('send')
        robot.sendDeployment()
        rate.sleep()
