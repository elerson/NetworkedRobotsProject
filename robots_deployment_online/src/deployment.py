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

from rssi_kalman import RSSIKalmanFilter

import tf

        
class Robot:
    def __init__(self):

        rospy.init_node('robot_deployment', anonymous=True)
        #print("ROBOT")
        self.network = Network()
        id = rospy.get_param("~id")

        
        self.allocation_id = -1        
        self.initialized = False
        self.high_level_distance = 3
        self.sent_goal = 0


        self.send_position_time_diff = rospy.get_param("~pose_send_time", 0.1)
        self.tree_file = rospy.get_param("~tree_file")
        self.ray = rospy.get_param("~ray", 120)

        self.map_resolution = 0.5
        self.height = 0

        self.send_position_time = rospy.get_time() 

        self.tree = Tree(self.tree_file)
        self.tree_segmentation = TreeSegmention(self.tree)
        self.tree_segmentation_segments = []


        self.robots_ids_start = len(self.tree.vertices)

        self.id = int(id) + self.robots_ids_start
        self.position = {}
        self.position['id'] = self.id
        self.position['position'] = (0.0, 0.0, 0.0)
        
        self.metric_measurements = {}

        self.num_robots = 0
        self.deployment_position = []

        self.status = -1

        self.ros_id = self.id - self.robots_ids_start  
        rospy.Subscriber("/robot_"+str(self.ros_id)+"/amcl_pose", PoseWithCovarianceStamped, self.getPose)
        self.vel_pub = rospy.Publisher("/robot_"+str(self.ros_id)+"/cmd_vel", Twist, queue_size=10)

        self.goal_pub = rospy.Publisher("/robot_"+str(self.ros_id)+"/move_base_simple/goal", PoseStamped, queue_size=10)
        rospy.Subscriber("/robot_"+str(self.ros_id)+"/move_base/status", GoalStatusArray, self.getStatus)
        rospy.Subscriber("/map_metadata", MapMetaData, self.getMap)

        rospy.Timer(rospy.Duration(0.1), self.simulationMetric)

        self.metric_kalman = {}
        self.gamma = 3
        

    def logNormalMetric(self, distance, variance):
        if(distance < 1):
            return 40
        return 40 + 10*self.gamma*math.log(distance) + np.random.normal(0,variance,1)[0]


    def simulationMetric(self, param):

        variance = 10.0
        #for the robots
        for data_id in self.network.rcv_data:
            #if(data_id == self.id):
            #    continue
            real_distance = self.getDistance(self.position['position'], self.network.rcv_data[data_id]['position'])*self.map_resolution
            simulated_metric = self.logNormalMetric(real_distance, variance) #real_distance + np.random.normal(0,variance,1)[0]
            

            #0-time, 1-realposition, 2-neighposition, 3-real_distance, 4-simulated_metric
            self.metric_measurements[data_id] = (rospy.get_time(), self.position['position'], self.network.rcv_data[data_id]['position'], real_distance, simulated_metric)
            if( data_id not in self.metric_kalman):
                self.metric_kalman[data_id] =  RSSIKalmanFilter([40.0, 2.4], 10.0, variance)

            m, P = self.metric_kalman[data_id].getResult()
            #print(m)

            self.metric_kalman[data_id].addMeasurement(real_distance, simulated_metric)


        #for the tree
        for vertex in self.tree.graph_adj_list:
            real_distance = self.getDistance(self.position['position'], self.tree.graph_vertex_position[vertex])*self.map_resolution
            simulated_metric = self.logNormalMetric(real_distance, variance)#real_distance + np.random.normal(0,variance,1)[0]
            self.metric_measurements[vertex] = (rospy.get_time(), self.position['position'], self.tree.graph_vertex_position[vertex], real_distance, simulated_metric)

            if( vertex not in self.metric_kalman):
                self.metric_kalman[vertex] =  RSSIKalmanFilter([40.0, 2.4], 10.0, variance)

            self.metric_kalman[vertex].addMeasurement(real_distance, simulated_metric)

    def getStatus(self, Status):

        if(len(Status.status_list) > 0):
            self.status = Status.status_list[0].status

    def getMap(self, MapData):

        self.map_resolution = MapData.resolution
        self.height = MapData.height


    def sendDeployment(self, deployment_position):


        #print(deployment_position)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = deployment_position[0]*self.map_resolution
        pose.pose.position.y = deployment_position[1]*self.map_resolution

        #for debug
        self.position['destination'] = (deployment_position[0], deployment_position[1])

        print(pose.pose.position.x, pose.pose.position.y)
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        pose.pose.orientation = Quaternion(*q)
        self.goal_pub.publish(pose)



    def getPose(self, Pose):
        orientation = (
            Pose.pose.pose.orientation.x,
            Pose.pose.pose.orientation.y,
            Pose.pose.pose.orientation.z,
            Pose.pose.pose.orientation.w)
        orientation_euler = tf.transformations.euler_from_quaternion(orientation)

        self.position['position'] = (Pose.pose.pose.position.x/self.map_resolution, self.height- Pose.pose.pose.position.y/self.map_resolution, orientation_euler[2])

        #print(self.position['position'])
        #avoid to flood the network with messages
        if(rospy.get_time() - self.send_position_time > self.send_position_time_diff or not self.initialized):
            self.network.sendMessage(self.position)
            self.send_position_time__ = rospy.get_time()

        self.initialized = True
                
               
    def getMetricDistance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


    def getDistance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)



    #The current robot is always allocated in the first position
    def getAllRobotsPositions(self, allocation, inverse = False):


        robot_positions = []
        robot_ids = []
        robot_positions.append(self.position['position'])
        robot_ids.append(self.id)

        for data_id in self.network.rcv_data:
            if(data_id != self.id and allocation == []):
                position = self.network.rcv_data[data_id]['position']
                robot_positions.append(position)
                robot_ids.append(data_id)
            elif(data_id != self.id and data_id in allocation and not inverse):
                position = self.network.rcv_data[data_id]['position']
                robot_positions.append(position)
                robot_ids.append(data_id)
            elif(data_id != self.id and inverse): #data_id not in allocation
                position = self.network.rcv_data[data_id]['position']
                robot_positions.append(position)
                robot_ids.append(data_id)

        return robot_positions, robot_ids
  
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
                    allocation[j].append(r+self.robots_ids_start)
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

        #gets the closest point for each segment
        closest_points = []     
        closest_points_segments = []
        for i in range(1,len(allocated_segment)):
            p = self.tree.graph_vertex_position[allocated_segment[i-1]]
            q = self.tree.graph_vertex_position[allocated_segment[i]]

            closest_point = dist_to_segment(p, q, r)
            closest_points.append(closest_point)

            closest_points_segments.append((allocated_segment[i-1], allocated_segment[i]))

        #determine the closest point
        min_distance = float('inf')
        closest_point = []
        closest_point_seg_allocation = []
        closest_point_segment = []


        #gets the closest point from all segments
        for i in range(len(closest_points)):
            distance = self.getDistance(closest_points[i], r)
            if(min_distance > distance):
                min_distance = distance
                closest_point = closest_points[i]
                closest_point_segment = closest_points_segments[i]

        return closest_point, closest_point_segment, allocated_segment



    def getClosetPointToPath(self):
        
        #TODO: make it less cost
        allocation, segmentation = self.getTreeAllocationPerSegment()

        for alloc_id in allocation:
            if(self.id in allocation[alloc_id]):
                self.allocation_id = alloc_id
                break


        allocation_path = segmentation[self.allocation_id]
        #print("Allocation Path", allocation_path, allocation)


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

    def robotAsObstacles(self, robot_size = 0.70, force_multiplier = 1):

        robots,_ = self.getAllRobotsPositions([])
        resulting_direction = (0, 0)

        #get the distances
        for robot in robots[1:]:
            distance = self.getDistance(robot, robots[0])*self.map_resolution
            direction = ((robots[0][0]-robot[0])*self.map_resolution,(robots[0][1]-robot[1])*self.map_resolution)
            size = self.getDistance((0,0), direction) + 0.001
            direction = (direction[0]/size, direction[1]/size)

            if(distance < robot_size):
                force = (robot_size - distance)*force_multiplier
                resulting_direction = (resulting_direction[0] +force*direction[0], resulting_direction[1] +force*direction[1])

            #print("DIRECTION", distance)
        return resulting_direction


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
        positions,_ = self.getAllRobotsPositions(allocation[self.allocation_id])

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

    def getNeighborsIDs(self):

        #get all robots that are from the same allocation
        #TODO: make it less costy
        allocation, segmentation = self.getTreeAllocationPerSegment()

        #get my allocation
        for alloc_id in allocation:
            if(self.id in allocation[alloc_id]):
                self.allocation_id = alloc_id
                break

        #print(allocation, self.allocation_id)
        #get all robots that are in the allocation
        positions, ids = self.getAllRobotsPositions(allocation[self.allocation_id])


        p = self.tree.graph_vertex_position[segmentation[self.allocation_id][0]]
        q = self.tree.graph_vertex_position[segmentation[self.allocation_id][-1]]

        #append the clients ids to the allocation list
        ids.append(segmentation[self.allocation_id][0])
        ids.append(segmentation[self.allocation_id][-1])
        #append the clients positions to the positions list
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
            neighbors.append(positive_closest.argmin() + 1) # +1 -> the element 1 was desconsidered -> alphas = alphas[1:]
            neighbors.append(negative_closest.argmax() + 1)

        elif(not np.isinf(positive_closest.min())):
            neighbors = positive_closest.argsort()[0:2]

        else:
            neighbors = negative_closest.argsort()[-2:]

        neigh_0 = ids[neighbors[0]]
        neigh_1 = ids[neighbors[1]]
        # ##
        # ##  IF the neigbor selected is not a client or a robot from the segment allocation
        # ##
        # if(neighbors[0] < self.robots_ids_start or neighbors[1] < self.robots_ids_start):
        #     #defining the alphas for all robots that are not from my segment
        #     positions, ids = self.getAllRobotsPositions(allocation[self.allocation_id], True)

        #     alphas = []
        #     for position in positions:

        #         alpha = dist_to_segment_alpha(p, q, position)
        #         alphas.append(alpha)

        #     self_alpha = alphas[0]
        #     alphas = np.asfarray(alphas[1:]) - self_alpha
            
        #     if(neighbors[0] < self.robots_ids_start):
        #         positive_closest = alphas.copy()
        #         positive_closest[positive_closest < 0] = np.inf
        #         if(not np.isinf(positive_closest.min())):
        #             neighbors[0] = neighbors.append(positive_closest.argmin() + 1)
        #             neigh_0 = ids[neighbors[0]]

        #     if(neighbors[1] < self.robots_ids_start):
        #         negative_closest = alphas.copy()
        #         negative_closest[negative_closest >= 0] = -np.inf
        #         if(not np.isinf(negative_closest.max())):
        #             neighbors[1] = neighbors.append(negative_closest.argmax() + 1)
        #             neigh_1 = ids[neighbors[1]]


        return [neigh_0, neigh_1]

    def limitVector(self, vec, maxsize):

        size = math.sqrt(vec[0]**2 + vec[1]**2)

        if(size > maxsize):
            return (maxsize*vec[0]/size, maxsize*vec[1]/size)
        else:
            return vec

    def highLevelControl(self):
        # print("control")
        closest_point = robot.getClosetPointToPath()
        # r = self.position['position']

        # dist_to_segmentation = self.getDistance(r, closest_point)*self.map_resolution
        
        # #  
        # #  Chose the high level navigation or the gradient allocation
        # #
        # if(dist_to_segmentation > self.high_level_distance):
        #     if(self.status == 1 or self.status == 0 or self.status == 2):
        #         return

        #     goal = (closest_point[0], (self.height-closest_point[1]))
        #     #goal = (19.695165209372934, 10.23384885215893)
        #     print("Goal", goal)
        #     self.sendDeployment(goal)
        
        #     #print(self.position['destination'])
        # else:
        #     if(self.status == 3 or self.status == -1): #succed or pending
        #         self.control_(closest_point)
        self.control_nonholonomic(closest_point)


    def distanceDerivative(self, x, y, id):


        gamma = self.metric_kalman[id].getGamma()
        print(id, gamma)
        #return(x/(math.sqrt(x**2 + y**2)+0.01), y/(math.sqrt(x**2 + y**2)+0.01))
        return(10*gamma*x/((x**2 + y**2)+0.001), 10*gamma*y/((x**2 + y**2)+0.001))
        #return(5*self.gamma*x/(((x**2 + y**2))*self.logNormalMetric((x**2 + y**2)**(1.0/2),0)), 5*self.gamma*y/(((x**2 + y**2))*self.logNormalMetric((x**2 + y**2)**(1.0/2),0)))


    def getDistanceByID(self, id):
        #0-time, 1-realposition, 2-neighposition, 3-real_distance, 4-simulated_metric
        #print(self.metric_measurements)
        r = self.position['position']
        p = self.getPositionByID(id)

        distance = self.getDistance(r, p)*self.map_resolution

        # return self.logNormalMetric(dist, 0)
        #return self.metric_measurements[id][4]
        #print(self.metric_kalman)
        return self.metric_kalman[id].getMetricValue(distance)


    def getPositionByID(self, id):
        #print(self.network.rcv_data)
        ##Client or tree junctions id
        if(id < self.robots_ids_start):
            return self.tree.graph_vertex_position[id]
        else:
            return self.network.rcv_data[id]['position']


    def verifyMetricOnNeighbors(self, neighbors_ids):
        return (neighbors_ids[0] in self.metric_kalman) and (neighbors_ids[1] in self.metric_kalman)      


    def control_holonomic(self, closest_point):
        r = self.position['position']
        #print(r)
        neighbors_ids = robot.getNeighborsIDs()

        #print(self.metric_kalman, neighbors_ids, self.id)
        #verify if we already can use the metric
        if( not self.verifyMetricOnNeighbors(neighbors_ids) ):
            return
        #print(self.metric_kalman)


        neighbors_positions = [ self.getPositionByID(neighbors_ids[0]), self.getPositionByID(neighbors_ids[1])]

        #neighbors_positions = self.getNeighbors()
        neighbor_1_distance = self.getDistanceByID(neighbors_ids[0])
        neighbor_2_distance = self.getDistanceByID(neighbors_ids[1])
       
        #neighbor_1_distance = self.getDistance(r, neighbors_positions[0])*self.map_resolution + 0.001
        #neighbor_2_distance = self.getDistance(r, neighbors_positions[1])*self.map_resolution + 0.001


        #print("neighbors", neighbors_positions, (neighbor_1_distance-neighbor_2_distance)**2)

        closest_point_path = closest_point
        path_distance = self.getDistance(r, closest_point_path)*self.map_resolution
        path_direction = (((closest_point_path[0] - r[0])*self.map_resolution), -(closest_point_path[1] - r[1])*self.map_resolution)

        derivative_neighbor_1_distance = self.distanceDerivative((r[0] -neighbors_positions[0][0])*self.map_resolution, (r[1]-neighbors_positions[0][1])*self.map_resolution, neighbors_ids[0])
        derivative_neighbor_2_distance = self.distanceDerivative((r[0] -neighbors_positions[1][0])*self.map_resolution, (r[1]-neighbors_positions[1][1])*self.map_resolution, neighbors_ids[1])
        
        beta = 0.01
        d1 = (beta*(derivative_neighbor_1_distance[0]-derivative_neighbor_2_distance[0]), beta*(derivative_neighbor_1_distance[1]-derivative_neighbor_2_distance[1]))
        df = (2*(neighbor_1_distance - neighbor_2_distance)*d1[0], 2*(neighbor_1_distance - neighbor_2_distance)*d1[1])
        
        #print("Derivative", self.ros_id, df )



        df = self.limitVector(df, 2)

        alpha = 6
        final_direction = (alpha*path_direction[0] -df[0], alpha*path_direction[1] +df[1])
        

      
        ##
        ##  Coverting to non-holonomic
        ##
        robot_angle = -r[2]

        cmd_vel = Twist()
        cmd_vel.linear.x = final_direction[0]*math.cos(robot_angle) - final_direction[1]*math.sin(robot_angle)
        cmd_vel.linear.y = final_direction[0]*math.sin(robot_angle) + final_direction[1]*math.cos(robot_angle)        
        cmd_vel.angular.z = 0

        self.vel_pub.publish(cmd_vel)


    def control_nonholonomic(self, closest_point):
        r = self.position['position']

        neighbors_ids = robot.getNeighborsIDs()

        if( not self.verifyMetricOnNeighbors(neighbors_ids) ):
            return

        #neighbors_positions = robot.getNeighbors()
        neighbors_positions = [ self.getPositionByID(neighbors_ids[0]), self.getPositionByID(neighbors_ids[1])]

        #neighbors_positions = self.getNeighbors()
        neighbor_1_distance = self.getDistanceByID(neighbors_ids[0])
        neighbor_2_distance = self.getDistanceByID(neighbors_ids[1])

        closest_point_path = closest_point
        path_distance = self.getDistance(r, closest_point_path)*self.map_resolution
        path_direction = (((closest_point_path[0] - r[0])*self.map_resolution), -(closest_point_path[1] - r[1])*self.map_resolution)

        derivative_neighbor_1_distance = self.distanceDerivative(r[0] -neighbors_positions[0][0], r[1]-neighbors_positions[0][1], neighbors_ids[0])
        derivative_neighbor_2_distance = self.distanceDerivative(r[0] -neighbors_positions[1][0], r[1]-neighbors_positions[1][1], neighbors_ids[1])
    
        beta = 0.1
        d1 = (beta*(derivative_neighbor_1_distance[0]-derivative_neighbor_2_distance[0]), beta*(derivative_neighbor_1_distance[1]-derivative_neighbor_2_distance[1]))
        df = (2*(neighbor_1_distance - neighbor_2_distance)*d1[0], 2*(neighbor_1_distance - neighbor_2_distance)*d1[1])
        df = self.limitVector(df, 2)

        alpha = 6
        final_direction = (alpha*path_direction[0] -df[0], alpha*path_direction[1] +df[1])
        
        robot_angle = r[2]
    
        theta = (final_direction[1]*math.cos(robot_angle) - final_direction[0]*math.sin(robot_angle))
        linear = final_direction[0]*math.cos(robot_angle) + final_direction[1]*math.sin(robot_angle)
    
        cmd_vel = Twist()
        cmd_vel.linear.x = linear#final_direction[0]*math.cos(robot_angle) - final_direction[1]*math.sin(robot_angle)
        cmd_vel.linear.y = 0#final_direction[0]*math.sin(robot_angle) + final_direction[1]*math.cos(robot_angle)
        
        cmd_vel.angular.z = theta

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
        #       Gradient to the neighbors
        #
        #

        neighbors_positions = robot.getNeighbors()
        neighbor_1_distance = self.getDistance(r, neighbors_positions[0])*self.map_resolution + 0.001
        neighbor_2_distance = self.getDistance(r, neighbors_positions[1])*self.map_resolution + 0.001

        #print(neighbor_1_distance, neighbor_2_distance)
        derivative_neighbor_1_distance = (-((r[0]-neighbors_positions[0][0])*self.map_resolution)/(neighbor_1_distance), (-(r[1]-neighbors_positions[0][1])*self.map_resolution)/(neighbor_1_distance))
        derivative_neighbor_2_distance = (-((r[0]-neighbors_positions[1][0])*self.map_resolution)/(neighbor_2_distance), (-(r[1]-neighbors_positions[1][1])*self.map_resolution)/(neighbor_2_distance))


        d1 = (derivative_neighbor_1_distance[0]-derivative_neighbor_2_distance[0], derivative_neighbor_1_distance[1]-derivative_neighbor_2_distance[1])
        df = (2*(neighbor_1_distance - neighbor_2_distance)*d1[0], 2*(neighbor_1_distance - neighbor_2_distance)*d1[0])

        #first derivative
        #d1 = (derivative_neighbor_1_distance[0]-derivative_neighbor_2_distance[0], derivative_neighbor_1_distance[1]-derivative_neighbor_2_distance[1])
        #d1_f = (d1[0]*(neighbor_1_distance - neighbor_2_distance)/4, -d1[1]*(neighbor_1_distance - neighbor_2_distance)/4)


        #second derivative
        #d2 = (derivative_neighbor_2_distance[0]-derivative_neighbor_1_distance[0], derivative_neighbor_2_distance[1]-derivative_neighbor_1_distance[1])
        #d2_f = (d2[0]*(neighbor_2_distance - neighbor_1_distance)/4, -d2[1]*(neighbor_2_distance - neighbor_1_distance)/4)

        
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
        #direction_to_segment = (direction_to_segment[0]*self.map_resolution, -direction_to_segment[1]*self.map_resolution)



        #final_direction = (10*tree_direction[0] + direction_to_segment[0], 10*tree_direction[1] + direction_to_segment[1])
        #final_direction = ( direction_to_segment[0],  direction_to_segment[1])
        
        #alpha = (self.getDistance((0,0), direction_to_segment)+0.01)/self.getDistance((0,0), tree_direction) + 3
        #print("direction=", direction_to_segment, tree_direction, alpha)
        alpha = 15

        final_direction = (alpha*tree_direction[0] + df[0] , alpha*tree_direction[1] + df[1])
        
        #print("ROBOT AS OBSTACLES", robot_as_obstacles)
        #final_direction =  robot_as_obstacles


        cmd_vel = Twist()
        #if(abs(robot_direction[0]) > 0.1):
        cmd_vel.linear.x = final_direction[0]
        #if(abs(robot_direction[1]) > 0.1):
        cmd_vel.linear.y = final_direction[1]


        self.vel_pub.publish(cmd_vel)

        
if __name__ == "__main__":
    robot = Robot()
    robot.getTreeAllocationPerSegment()
    rate = rospy.Rate(25.0)
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        #print('send')
        robot.highLevelControl()
        rate.sleep()
        #print(now-rospy.get_rostime())
