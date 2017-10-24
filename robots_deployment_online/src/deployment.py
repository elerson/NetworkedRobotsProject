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

from util import dist_to_segment_alpha, dist_to_segment

import tf

        
class Robot:
    def __init__(self):

        rospy.init_node('robot_deployment', anonymous=True)

        self.network = Network()
        id = rospy.get_param("~id")

        self.id = int(id)
        self.allocation_id = -1

        self.position = {}
        self.position['id'] = self.id
        self.position['position'] = (0.0, 0.0)
        self.initialized = False



        self.send_position_time_diff = rospy.get_param("~pose_send_time", 0.1)
        self.tree_file = rospy.get_param("~tree_file")
        self.ray = rospy.get_param("~ray", 70)

        self.map_resolution = 0.5
        self.height = 0

        self.send_position_time = rospy.get_time() 

        self.tree = Tree(self.tree_file)
        self.tree_segmentation = TreeSegmention(self.tree)
        self.tree_segmentation_segments = []

        self.num_robots = 0
        self.deployment_position = []

        self.status = -1


        rospy.Subscriber("/robot_"+str(self.id)+"/amcl_pose", PoseWithCovarianceStamped, self.getPose)
        self.vel_pub = rospy.Publisher("/robot_"+str(self.id)+"/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/map_metadata", MapMetaData, self.getMap)



    def getMap(self, MapData):

        self.map_resolution = MapData.resolution
        self.height = MapData.height


    def getPose(self, Pose):

        self.position['position'] = (Pose.pose.pose.position.x/self.map_resolution, self.height- Pose.pose.pose.position.y/self.map_resolution)

        #print(self.position['position'])
        #avoid to flood the network with messages
        if(rospy.get_time() - self.send_position_time > self.send_position_time_diff or not self.initialized):
            self.network.sendMessage(self.position)
            self.send_position_time__ = rospy.get_time()

        self.initialized = True
                
               

    def getDistance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)



    #The current robot is always allocated in the first position
    def getAllRobotsPositions(self, allocation):


        robot_positions = []
        robot_positions.append(self.position['position'])

        for data_id in self.network.rcv_data:
            if(data_id != self.id and data_id in allocation):
                position = self.network.rcv_data[data_id]['position']
                robot_positions.append(position)

        return robot_positions
  
    def getSegmentation(self):
        if(not self.tree_segmentation_segments == []):
            return self.tree_segmentation_segments
        self.tree_segmentation.segmentation_search([], [])
        #print(tree_segmentation.segmentaion_paths)
        segmentation = self.tree_segmentation.evaluate_segmentation()
        self.tree_segmentation_segments = segmentation
        return segmentation


    def getTreeAllocationPerSegment(self):

        ##define the number of robots per segment
        segmentation = self.getSegmentation()
        n_robots = {}
        allocation_sum = []
        sum_robots = 0
        i = 0
        for segment in segmentation:
            cost = self.tree_segmentation.get_path_cost(segment)
            n_robots[i] = math.ceil(cost/self.ray)
            sum_robots += n_robots[i]

            allocation_sum.append(sum_robots)
            i = i+1

        allocation = {}
        for j in range(len(segmentation)):
            allocation[j] = []
        for r in range(int(sum_robots)):
            for j in range(len(segmentation)):
                if(r < allocation_sum[j]):
                    allocation[j].append(r)
                    break


        return allocation, segmentation




    def getClosetPointToTree(self):

        #TODO: make it less cost
        allocation, segmentation = self.getTreeAllocationPerSegment()

        for alloc_id in allocation:
            if(self.id in allocation[alloc_id]):
                self.allocation_id = alloc_id
                break

        allocated_segment = segmentation[self.allocation_id]

        #get the closest point in each segment
        r = self.position['position']
        closest_points = []
        points_allocation = []
        closest_points_segments = []
        j = 0
        for allocation_path in segmentation:       
            for i in range(1,len(allocation_path)):
                p = self.tree.graph_vertex_position[allocation_path[i-1]]
                q = self.tree.graph_vertex_position[allocation_path[i]]

                closest_point = dist_to_segment(p, q, r)
                closest_points.append(closest_point)

                points_allocation.append(j)

                closest_points_segments.append((allocation_path[i-1], allocation_path[i]))
            j += 1
       
        #determine the closest point
        min_distance = float('inf')
        closest_point = []
        closest_point_seg_allocation = []
        closest_point_segment = []

        #from all segments, gets the closest point
        for i in range(len(closest_points)):
            distance = self.getDistance(closest_points[i], r)
            if(min_distance > distance):
                min_distance = distance
                closest_point = closest_points[i]
                closest_point_seg_allocation = (points_allocation[i], self.allocation_id)
                closest_point_segment = closest_points_segments[i]

        return closest_point, closest_point_segment, closest_point_seg_allocation, allocated_segment



    def getClosetPointToPath(self):
        
        #TODO: make it less cost
        allocation, segmentation = self.getTreeAllocationPerSegment()

        for alloc_id in allocation:
            if(self.id in allocation[alloc_id]):
                self.allocation_id = alloc_id
                break


        allocation_path = segmentation[self.allocation_id]
        print("Allocation Path", allocation_path, allocation)


        #get the closest point in each segment
        r = self.position['position']
        closest_points = []
        for i in range(1,len(allocation_path)):
            p = self.tree.graph_vertex_position[allocation_path[i-1]]
            q = self.tree.graph_vertex_position[allocation_path[i]]

            closest_point = dist_to_segment(p, q, r)
            closest_points.append(closest_point)
   
        #determine the closest point
        min_distance = float('inf')
        closest_point = []

        #from all segments, gets the closest point
        for i in range(len(closest_points)):
            distance = self.getDistance(closest_points[i], r)
            if(min_distance > distance):
                min_distance = distance
                closest_point = closest_points[i]

        return closest_point

        

    def getNeighbors(self):

        #get all robots that are from the same allocation
        #TODO: make it less cost
        allocation, segmentation = self.getTreeAllocationPerSegment()
        #print(allocation)
        #get my allocation

        for alloc_id in allocation:
            if(self.id in allocation[alloc_id]):
                self.allocation_id = alloc_id
                break



        #get all robots that are in the allocation
        positions = self.getAllRobotsPositions(allocation[self.allocation_id])

        p = self.tree.graph_vertex_position[segmentation[self.allocation_id][0]]
        q = self.tree.graph_vertex_position[segmentation[self.allocation_id][-1]]


        positions.append(p)
        positions.append(q)


        alphas = []
        for position in positions:

            alpha = dist_to_segment_alpha(p, q, position)
            alphas.append(alpha)


        #get the closet element from the position 0 alpha // self
        #negative closest and positive closest

        neighbors = []
        #position 0 is the self robot
        self_alpha = alphas[0]
        alphas = np.asfarray(alphas[1:]) - self_alpha


        positive_closest = alphas.copy()
        positive_closest[positive_closest < 0] = np.inf    
        
        negative_closest = alphas.copy()
        negative_closest[negative_closest >= 0] = -np.inf

        if(not np.isinf(positive_closest.min()) and not np.isinf(negative_closest.max())):
            neighbors.append(positive_closest.argmin() + 1)
            neighbors.append(negative_closest.argmax() + 1)

        elif(not np.isinf(positive_closest.min())):
            neighbors = positive_closest.argsort()[0:2]

        else:
            neighbors = negative_closest.argsort()[-2:]



        
        neighbor_positions = []
        neighbor_positions.append(positions[neighbors[0]])
        neighbor_positions.append(positions[neighbors[1]])

        return neighbor_positions


    def control_(self):
        r = self.position['position']
        #r = (math.ceil(r[0]), math.ceil(r[1]))

        closest_point_path = robot.getClosetPointToPath()
        neighbors_positions = robot.getNeighbors()

        neighbor_1_distance = self.getDistance(r, neighbors_positions[0])*self.map_resolution + 0.001
        neighbor_2_distance = self.getDistance(r, neighbors_positions[1])*self.map_resolution + 0.001

        print(neighbor_1_distance, neighbor_2_distance)
        derivative_neighbor_1_distance = (-((r[0]-neighbors_positions[0][0])*self.map_resolution)/(neighbor_1_distance), (-(r[1]-neighbors_positions[0][1])*self.map_resolution)/(neighbor_1_distance))
        derivative_neighbor_2_distance = (-((r[0]-neighbors_positions[1][0])*self.map_resolution)/(neighbor_2_distance), (-(r[1]-neighbors_positions[1][1])*self.map_resolution)/(neighbor_2_distance))

        #first derivative
        d1 = (derivative_neighbor_1_distance[0]-derivative_neighbor_2_distance[0], derivative_neighbor_1_distance[1]-derivative_neighbor_2_distance[1])
        d1_f = (d1[0]*(neighbor_1_distance - neighbor_2_distance)/4, -d1[1]*(neighbor_1_distance - neighbor_2_distance)/4)


        #second derivative
        d2 = (derivative_neighbor_2_distance[0]-derivative_neighbor_1_distance[0], derivative_neighbor_2_distance[1]-derivative_neighbor_1_distance[1])
        d2_f = (d2[0]*(neighbor_2_distance - neighbor_1_distance)/4, -d2[1]*(neighbor_2_distance - neighbor_1_distance)/4)


        path_distance = self.getDistance(r, closest_point_path)*self.map_resolution
        # #print("closest ", r, closest_point_path)
        path_direction = (((closest_point_path[0] - r[0])*self.map_resolution), -(closest_point_path[1] - r[1])*self.map_resolution)

        final_direction = (10*path_direction[0] + d1_f[0] + d2_f[0], 10*path_direction[1] + d1_f[1] + d2_f[1])
        


        cmd_vel = Twist()
        #if(abs(robot_direction[0]) > 0.1):
        cmd_vel.linear.x = final_direction[0]
        #if(abs(robot_direction[1]) > 0.1):
        cmd_vel.linear.y = final_direction[1]


        self.vel_pub.publish(cmd_vel)

    def segmentInsideSgment(self, segment1, segment2):
        return segment1[0] in segment2 and segment1[1] in segment2

    def getClosestNodeFromPoint(self, nodeSet, point):

        distance = float('inf')
        out_node = []
        for node in nodeSet:
            new_distance = self.getDistance(point, self.tree.graph_vertex_position[node])
            if(new_distance < distance):
                distance = new_distance
                out_node = node
        return out_node





    def control(self):
        r = self.position['position']
        #r = (math.ceil(r[0]), math.ceil(r[1]))
        #closest_point, closest_point_segment, closest_point_seg_allocation, allocated_segment
        closest_point, closest_segment, closest_point_seg_allocation, allocated_segment = robot.getClosetPointToTree()

        #
        #
        #       Gradient to the allocated Segment
        #
        #
        direction_to_segment = (0, 0)
        distance_to_segment = 0
        if( not closest_point_seg_allocation[0] == closest_point_seg_allocation[1]): ##when they are different, they where allocated to different segments
            #get the path to the allocated segment
            path_to_segment = self.tree_segmentation.findPath(closest_segment[0], allocated_segment)
            
            distance_to_segment = 0
            direction_to_segment = []
            print(path_to_segment)
            #see the second node from closest_segment [(3,2),(2,1)] - (2,3)
            if(self.segmentInsideSgment(path_to_segment[0], closest_segment)): 
                path_to_segment = path_to_segment[1:]
            

            if(path_to_segment == []):
                    #get closet point to allocated segment
                    node = self.getClosestNodeFromPoint(allocated_segment,closest_point)
                    path_to_segment = [(node, node)]
            ## Get the distance to the allocated segment
            distance_to_segment = self.tree_segmentation.get_path_cost_from_segments(path_to_segment)
            distance_to_segment += self.getDistance(closest_point, self.tree.graph_vertex_position[path_to_segment[0][0]])


            #get the direction to the allocated segment
            if(self.getDistance(closest_point, self.tree.graph_vertex_position[path_to_segment[0][0]]) < 1.0):
                next_point = self.tree.graph_vertex_position[path_to_segment[0][1]]
            else:
                next_point = self.tree.graph_vertex_position[path_to_segment[0][0]]

            direction_to_segment = (next_point[0] - closest_point[0] ,next_point[1] - closest_point[1] )
            direction_size_ = self.getDistance((0,0), direction_to_segment)
            direction_to_segment = (direction_to_segment[0]/direction_size_, direction_to_segment[1]/direction_size_)

          
            
        
        #
        #
        #       Gradient to the neighbors
        #
        #

        neighbors_positions = robot.getNeighbors()
        neighbor_1_distance = self.getDistance(r, neighbors_positions[0])*self.map_resolution + 0.001
        neighbor_2_distance = self.getDistance(r, neighbors_positions[1])*self.map_resolution + 0.001

        print(neighbor_1_distance, neighbor_2_distance)
        derivative_neighbor_1_distance = (-((r[0]-neighbors_positions[0][0])*self.map_resolution)/(neighbor_1_distance), (-(r[1]-neighbors_positions[0][1])*self.map_resolution)/(neighbor_1_distance))
        derivative_neighbor_2_distance = (-((r[0]-neighbors_positions[1][0])*self.map_resolution)/(neighbor_2_distance), (-(r[1]-neighbors_positions[1][1])*self.map_resolution)/(neighbor_2_distance))

        #first derivative
        d1 = (derivative_neighbor_1_distance[0]-derivative_neighbor_2_distance[0], derivative_neighbor_1_distance[1]-derivative_neighbor_2_distance[1])
        d1_f = (d1[0]*(neighbor_1_distance - neighbor_2_distance)/4, -d1[1]*(neighbor_1_distance - neighbor_2_distance)/4)


        #second derivative
        d2 = (derivative_neighbor_2_distance[0]-derivative_neighbor_1_distance[0], derivative_neighbor_2_distance[1]-derivative_neighbor_1_distance[1])
        d2_f = (d2[0]*(neighbor_2_distance - neighbor_1_distance)/4, -d2[1]*(neighbor_2_distance - neighbor_1_distance)/4)

        
        #
        #       Gradient to the tree
        #
        #
        closest_point_path = closest_point
        tree_distance = self.getDistance(r, closest_point_path)*self.map_resolution
        # #print("closest ", r, closest_point_path)
        tree_direction = (((closest_point_path[0] - r[0])*self.map_resolution), -(closest_point_path[1] - r[1])*self.map_resolution)

        #segment direction
        #distance_to_segment = min(distance_to_segment, 30)
        direction_to_segment = (direction_to_segment[0]*self.map_resolution, -direction_to_segment[1]*self.map_resolution)



        #final_direction = (10*tree_direction[0] + direction_to_segment[0], 10*tree_direction[1] + direction_to_segment[1])
        #final_direction = ( direction_to_segment[0],  direction_to_segment[1])
        
        #alpha = (self.getDistance((0,0), direction_to_segment)+0.01)/self.getDistance((0,0), tree_direction) + 3
        #print("direction=", direction_to_segment, tree_direction, alpha)
        alpha = 20
        final_direction = (tree_direction[0] + alpha*direction_to_segment[0], tree_direction[1] + alpha*direction_to_segment[1])


        #final_direction = (tree_direction[0] + direction_to_segment[0] + d1_f[0] + d2_f[0], 8*tree_direction[1] + direction_to_segment[1] + d1_f[1] + d2_f[1])
        


        cmd_vel = Twist()
        #if(abs(robot_direction[0]) > 0.1):
        cmd_vel.linear.x = final_direction[0]
        #if(abs(robot_direction[1]) > 0.1):
        cmd_vel.linear.y = final_direction[1]


        self.vel_pub.publish(cmd_vel)

        

if __name__ == "__main__":
    robot = Robot()
    robot.getTreeAllocationPerSegment()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        print('send')
        robot.control()
        rate.sleep()
        print(now-rospy.get_rostime())
