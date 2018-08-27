#!/usr/bin/python

import rospy
from network_utils.tree import Tree
from scipy.optimize import linear_sum_assignment
from network_utils.network import Network
from network_utils.sceneGraph import sceneGraph
from enum import IntEnum
import yaml
import time
import tf
import numpy as np
import math


#ROS Imports
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion 
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import MapMetaData
from move_base_msgs.msg import MoveBaseActionGoal


from network_utils.steiner import Steiner


class MSG(IntEnum):    
    INIT                    = 0
    DISCOVERY_COMPLETE      = 1
    SETTLE_COMPLETE         = 2
    MIGRATE                 = 3
    DEPLOY                  = 4
    DISCOVERY_START         = 6
    INIT_ACK                = 8
    PROBE                   = 9
    MOVETO                  = 10
    INFO                    = 11

class STEP(IntEnum):
    DISCOVER                = 0
    MIGRATE                 = 1
    SETTLE                  = 2
    DEPLOY                  = 3


class PHASE(IntEnum):
    INIT                    = 0
    SEARCH                  = 1
    DEPLOY                  = 2

class DEPLOYSTEP(IntEnum):
    INIT                    = 0
    DEPLOY                  = 1


class Robot:
    def __init__(self):
        rospy.init_node('anchor_robot', anonymous=True)
        self.config_file     = self.readConfig(rospy.get_param("~config_file"))

        self.tree_file       = self.config_file['configs']['treefile']
        self.tree            = Tree(self.tree_file)
        self.clients         = set(self.tree.clients)

        self.ros_id          = rospy.get_param("~id")
        self.real_robot      = not self.config_file['configs']['simulation'] 
        if(self.real_robot):            
            self.routing         = Routing('teste4', self.config_file, 'ra0')
            self.rss_measure     = RSSMeasure('teste4', self.config_file)
            id                   = self.routing.getID()
        else:
            id                   = self.ros_id + len(self.clients)


        self.id              = id
        self.send_node       = -1

        self.node_id         = 0 

        self.status          = -1

        self.is_coord        = False
        self.radius          = rospy.get_param("~radius", 70)
        self.xoffset         = rospy.get_param("~xoffset", 0)
        self.yoffset         = rospy.get_param("~yoffset", 0)
        self.clients_pos     = [ (self.tree.graph_vertex_position[i][0], self.tree.graph_vertex_position[i][1]) for i in self.tree.clients]

        
        graph_radius         = self.radius*(2.0/3.0)
        self.graph           = sceneGraph(self.config_file['configs'], graph_radius, self.clients_pos, (self.xoffset, self.yoffset))
        self.height          =  self.graph.heigh
        print(graph_radius, self.radius)

        #get the terminal nodes in the graph
        self.terminals       = []
        self.terminal_src_ids= {}
        i = 0
        for client_id in self.tree.clients:
            position         = self.tree.graph_vertex_position[client_id]
            position         = (position[0], position[1])
            self.terminals.append(self.graph.getClosestNode(position))
            self.terminal_src_ids[i] = self.graph.getClosestNode(position)
            i += 1

        self.num_measurements = 10
        self.measurment_time  = 0.1

        self.visited_nodes   = set([])
        self.wait_init_ack   = False
        self.step            = STEP.DISCOVER
        self.is_idle         = True
        self.discovery_start = False
        self.dicover_walk    = []
        self.gateway_id      = 1
        self.gateway_node_id = -1
        self.is_anchored     = True
        self.map_resolution  = 0.05

        self.migrate_last_time  = -1
        self.migrate_time       = 5

        self.last_go_next_time = 0
        self.last_node       = -1


        self.last_probe      = 0
        self.probe_time      = 0.2

        self.last_send_deplyment   = 0
        self.send_deployment_time  = 4.5
        self.deploy_ended    = False
        self.deploy_numbers  = 0


        self.candAnchor      = set([])
        self.prevCandAnchor  = set([])
        self.links           = set([])
        self.links_graph     = {}
        self.walkPath        = set([])
        self.message_fifo    = []
        self.measurement_link= {}
        self.sim             = -1
        self.phase_alg       = PHASE.INIT

        self.network         = Network(self.id, broadcast_addr = self.config_file['configs']['broadcast_address'], port = self.config_file['configs']['algorithm_port'])
        self.network.addMessageCallback(self.receiveMessage)
        self.network.addCommandCallback(self.receiveNetworkCommand)

        self.init_tsp        = True
        self.allow_go_next_node = False
        
        self.level           = 0
        self.search_queue_level    = {}


        self.start_noncoor   = False

        self.robot_position_ids  = {}
        self.position            = {}
        self.position['position']= (0, 0, (0,0,0))
        self.steiner             = []
        self.comm_route          = []
        self.steiner_graph       = {}

        self.send_position_time_diff = rospy.get_param("~pose_send_time", 0.5)
        self.send_position_time      = 0.0


        if(self.real_robot):
            prefix               = rospy.get_param("~prefix")
        else:
            prefix               = "/robot_"+str(self.ros_id)


        rospy.Subscriber(prefix+"/amcl_pose", PoseWithCovarianceStamped, self.getPose)
  
        self.cancel_pub          = rospy.Publisher(prefix + "/move_base/cancel", GoalID, queue_size=10)
        self.current_goal_id     = 0
        self.goal_pub            = rospy.Publisher(prefix+"/move_base/goal", MoveBaseActionGoal, queue_size=10)

        self.initial_pub         = rospy.Publisher(prefix+"/initialpose", PoseWithCovarianceStamped, queue_size=10)

        rospy.Subscriber(prefix+"/move_base/status", GoalStatusArray, self.getStatus)
        rospy.Subscriber("/map_metadata", MapMetaData, self.getMap)
        self.initialized         = False

        self.deploy_steps        = DEPLOYSTEP.INIT
        self.steiner             = []

        self.measurements          = {}
        self.n_measurements      = {}
        self.gamma               = 3

        self.start_real  = True
        if(self.real_robot):
            self.start_real = False

        while(not self.initialized):
            print('waiting')
            time.sleep(0.3)

    def receiveNetworkCommand(self, commnad):
        #print(command)
        if (command['command'] == COMMANDS.SETINITALPOSE):
            self.start_real = True

    def logNormalMetric(self, distance, variance):
        if(distance < 1):
            return -40
        return -40 -10*self.gamma*math.log(distance) + np.random.normal(0,math.sqrt(variance),1)[0]

    def getRSSmeasurement(self, src):
 
        distance = self.graph.getDistanceFromId((self.position['position'][0], self.position['position'][1]), src)*self.map_resolution
        if(self.real_robot):
            measurement = self.rss_measure.getMeasurement(src)
        else:
            measurement = abs(self.logNormalMetric(distance, 1.0))

        print('node dst and current', self.graph.getClosestNode(self.position['position']), src, self.getCurrentNode(), distance, measurement)

        return measurement
        

    def getMinAssigment(self, robots, deployment):

        #create a matrix for the deployment
        dist_matrix = []
        print(robots, deployment)
        for robot in robots:
            dist_vector = []
            for node in deployment:
                print('node', robot,' ', node)
                distance = self.graph.getDistance(self.robot_position_ids[robot], node)
                dist_vector.append(distance)
            dist_matrix.append(dist_vector)

        row_ind, col_ind = linear_sum_assignment(dist_matrix)
        print(row_ind, col_ind, robots, deployment)
        assignment = {}
        inverse_assignment = {}
        for i in range(len(col_ind)):
            assignment[robots[row_ind[i]]] = deployment[col_ind[i]]
            inverse_assignment[deployment[col_ind[i]]] = robots[row_ind[i]]

        for client_id in self.terminal_src_ids:
            inverse_assignment[self.terminal_src_ids[client_id]] = client_id


        return assignment, inverse_assignment



    def deployPhase(self, message):

        if self.deploy_steps == DEPLOYSTEP.INIT:
            if self.steiner == []:
                print(self.links_graph, self.terminals)
                self.steiner = Steiner(self.links_graph, self.terminals)

            deploy_visit = list(set([v for k, v in self.robot_position_ids.items()]))
            deploy_visit.insert(0, self.node_id)
            deploy_visit = sorted(set(deploy_visit), key=lambda x: deploy_visit.index(x))
            

            self.deployVisit  = self.graph.calculateTSP(deploy_visit)
            self.deploy_steps = DEPLOYSTEP.DEPLOY
            self.allow_go_next_node = True
            self.visiting     = self.robot_position_ids.keys()


            deployment_positions  = self.steiner.steiner_vertices
            print('steiner ', deployment_positions, self.terminals, self.links_graph, self.steiner.steiner)
            self.steiner_graph = self.links_graph


            self.deployment_assignment, self.node_robot_assignment = self.getMinAssigment(self.robot_position_ids.keys(), deployment_positions)
            print(self.deployment_assignment, self.node_robot_assignment)
            self.deploy_numbers = len(deployment_positions)

            self.robot_neighbors = {}
            for robot in self.deployment_assignment:
                node = self.deployment_assignment[robot]
                self.robot_neighbors[robot] = []
                print(self.steiner.neighbors, node)
                for neigh in self.steiner.neighbors[node]:
                    try:
                        self.robot_neighbors[robot].append(self.node_robot_assignment[neigh])
                    except:
                        pass
            
            print('neighbors', self.robot_neighbors)

            #exit()

        elif self.deploy_steps == DEPLOYSTEP.DEPLOY:


            #self.deployVisit = self.goNextNode(self.deployVisit, 5)
            if((self.allow_go_next_node) or (message['type'] == MSG.PROBE and message['src'] in self.visiting) or (not self.getCurrentNode() in self.visiting)):
                last_node_id = self.getCurrentNode()               
                self.deployVisit = self.goNextNode(self.deployVisit)
                if(message['type'] == MSG.PROBE and not message['terminal']):                
                    ## send the deployment message
                    new_message          = {}
                    new_message['type']  = int(MSG.MOVETO)
                    new_message['id']    = self.id
                    new_message['phase'] = int(STEP.DEPLOY)
                    new_message['dest']  = self.deployment_assignment.get(message['id'], self.gateway_src)
                    new_message['route'] = self.robot_neighbors.get(message['id'], [])
                    #print(self.deployment_assignment, self.deployment_assignment.get(message['id'], self.gateway_src), message['id'],message)
                    self.network.sendMessageTo(message['id'], new_message)

                self.allow_go_next_node = last_node_id == self.getCurrentNode()
            if self.deployVisit == []:
                self.deploy_ended = True
            #print()
            #exit()



    def readConfig(self, config_file):
        with open(config_file, 'r') as stream:
            return yaml.load(stream)
    
    def alreadyMeasured(self, src, dst):
        if(src in self.links_graph and dst in self.links_graph[src]):
            return True
        return False

    def addLink(self, src, dst, weigth):
        try:
            self.links_graph[src][dst] = weigth
        except:
            self.links_graph[src]      = {}
            self.links_graph[src][dst] = weigth

        try:
            self.links_graph[dst][src] = weigth
        except:
            self.links_graph[dst]      = {}
            self.links_graph[dst][src] = weigth

        #print ('conn graph', self.links_graph)

    def receiveMessage(self, message):

        if(self.phase_alg != PHASE.INIT and (not self.is_coord) and message['type'] == MSG.PROBE):
            return

        self.message_fifo.insert(0, message)

        if(message['type'] == MSG.PROBE):
            try:
                self.measurement_link[self.getCurrentNode()].append(message['src'])
            except:
                self.measurement_link[self.getCurrentNode()] = [message['src']]                

    def hasMeasumentsWaiting(self):
        try:
            return self.links_graph[self.getCurrentNode()].keys() == self.measurement_link[self.getCurrentNode()].keys()
        except:
            return False


    def searchPhaseCoor(self, message):

        if(message['type'] == MSG.PROBE and not message['terminal']):
            self.robot_position_ids[message['id']] = message['src']


        #print('step ', self.step, self.status)

        if self.step == STEP.DISCOVER:
            if(self.is_idle and self.discovery_start):
                local_visited_nodes = set([k for k, v in self.search_queue_level.items() if v < self.level])

                to_visit = list((set(self.graph.graph.keys()) - local_visited_nodes).union(set([self.gateway_src])))
                to_visit.insert(0, self.node_id)
                to_visit = sorted(set(to_visit), key=lambda x: to_visit.index(x))


                self.dicover_walk       = self.graph.calculateTSP(to_visit)
                self.last_discover_walk = self.dicover_walk
                self.is_idle            = False
                self.discovery_start    = False

            if self.isInDestination() and message['type'] == MSG.PROBE and not self.alreadyMeasured(self.getCurrentNode(), message['src']):
                measurement = 0.0
                for m_ in range(self.num_measurements):
                    measurement += float(self.getRSSmeasurement(message['src']))
                    time.sleep(self.measurment_time)

                measurement = int(measurement/self.num_measurements)

                if self.isInDestination(): #measurment < self.radius:
                    self.links      = self.links.union((self.getCurrentNode(), message['src']))
                    self.addLink(self.getCurrentNode(), message['src'], measurement)
                    print(self.links_graph, measurement)
                    #time.sleep(2)
                    if(message['terminal']):
                        self.search_queue_level[self.getCurrentNode()] = 0
                        #print(message, 'terminal')
                    else:
                        if(message['src'] in self.search_queue_level and self.node_id not in self.search_queue_level):
                            self.search_queue_level[self.node_id] = self.search_queue_level[message['src']] + 1
                        pass


            if not self.hasMeasumentsWaiting():
                self.dicover_walk = self.goNextNode(self.dicover_walk, 3)
            print(self.dicover_walk, self.search_queue_level)
            #print(self.search_queue_level, self.getCurrentNode(), self.graph.getDistance(8, self.getCurrentNode()), 'discover walk')
            #if( 'src' in message):
            #    print(self.getCurrentNode(), self.graph.getDistance(message['src'], self.getCurrentNode()), 'new_walk')

            if(self.dicover_walk == []):
                self.is_idle = False

                self.to_visit = list(set([k for k, v in self.search_queue_level.items() if self.level == v]) - self.visited_nodes)
                print('to visit', self.to_visit)
                if(self.to_visit == []):
                    #self.is_idle = True
                    self.level  += 1
                    #self.step    = STEP.SETTLE


                self.step = STEP.MIGRATE
                self.to_visit = list(set([k for k, v in self.search_queue_level.items() if self.level == v]) - self.visited_nodes)
                self.visiting = []


                print('to visit', self.to_visit)
                if(self.to_visit == []):
                    #exit()
                    self.phase_alg = PHASE.DEPLOY
                    return

                

                migrate_visit = list(set([v for k, v in self.robot_position_ids.items()]))
                print('migrate visit', migrate_visit)
                migrate_visit.insert(0, self.node_id)
                migrate_visit = sorted(set(migrate_visit), key=lambda x: migrate_visit.index(x))
                
                #for key in self.robot_position_ids:
                #    self.robot_position_ids[key] = -1

                self.message_fifo    = []
                self.start_noncoor = True
                self.migrateWalk = self.graph.calculateTSP(migrate_visit)
                self.migrate_last_time = rospy.get_time()
                self.migrating_robots  = [] 
                print('migrate walk', self.migrateWalk)

        elif self.step == STEP.MIGRATE:
            if( (message['type'] == MSG.PROBE) and (not message['src'] in self.visiting)):                  
                print('probe', message['id'])

                if( not message['id'] in self.migrating_robots and abs(message['id']) >= len(self.clients) and len(self.to_visit) > 0 ):
                    dest                 = self.to_visit.pop()

                    new_message          = {}
                    new_message['type']  = int(MSG.MOVETO)
                    new_message['id']    = self.id
                    new_message['phase'] = int(self.step)
                    new_message['dest']  = dest
                    new_message['route'] = []

                    self.robot_position_ids[message['id']] = dest
                    self.migrating_robots.append(message['id'])

                    self.visiting.append(dest)

                    self.visited_nodes = self.visited_nodes.union(set([dest]))

                    print('move to', new_message)
                    self.network.sendMessageTo(message['id'], new_message)


                if abs(message['id']) >= len(self.clients) and self.to_visit == []:
                    new_message          = {}
                    new_message['type']  = int(MSG.MOVETO)
                    new_message['id']    = self.id
                    new_message['phase'] = int(self.step)
                    new_message['dest']  = self.gateway_src
                    new_message['route'] = []

                    self.robot_position_ids[message['id']] = self.gateway_src
                    self.visiting = list(set(self.visiting).union(set([self.gateway_src])))

                    self.network.sendMessageTo(message['id'], new_message)


            self.migrateWalk = self.goNextNode(self.migrateWalk, 5)


            print(self.migrateWalk, 'migrate', self.visiting, ' ', rospy.get_time() - self.migrate_last_time, self.migrate_time)
            if(self.migrateWalk == [] and (rospy.get_time() - self.migrate_last_time > self.migrate_time)):
                self.step = int(STEP.SETTLE)
                self.message_fifo    = []

                self.allow_go_next_node = True

                mylist = list(self.visiting)
                mylist.insert(0, self.node_id)
                to_visit = sorted(set(mylist), key=lambda x: mylist.index(x))
                self.settleWalk = self.graph.calculateTSP(to_visit)
                print('b visiting settle', self.visiting)


        elif self.step == STEP.SETTLE:

            #print('b visiting settle', self.visiting)
            #print(message, self.visiting, self.getCurrentNode())
            if((self.allow_go_next_node) or (message['type'] == MSG.PROBE and message['src'] in self.visiting) or (not self.getCurrentNode() in self.visiting)):
                #print('settle walk', self.settleWalk)
                last_node_id = self.getCurrentNode()               
                self.settleWalk = self.goNextNode(self.settleWalk)
                self.allow_go_next_node = last_node_id == self.getCurrentNode()
            
            #print('visiting settle', self.visiting)

            if message['type'] == MSG.PROBE:
                if(self.settleWalk == []):
                    self.discovery_start = True
                    self.step            = int(STEP.DISCOVER)
                    self.message_fifo    = []
                    self.is_idle         = True





    def searchAndDeployNonCoor(self, message):

        #if(not message['type'] == None and abs(message['id']) >= len(self.clients)):
        #    self.phase = message['phase']

        #if not self.solution_start:
        #    return
            

        if self.is_anchored:
            self.sendProbe()
            if(message['type'] == MSG.MOVETO):

                self.comm_route = message['route']
                #print(message, 'move to', self.getCurrentNode(),  message['dest'])
                self.is_anchored = False
                self.phase       = message['phase']
                if(message['dest'] != self.getCurrentNode()):
                    self.walkPath   = [self.getCurrentNode(), message['dest']]
                else:
                    self.walkPath   = []

        else:
            if self.walkPath == []:
                self.is_anchored = True
                self.is_idle     = True
            self.walkPath = self.goNextNode(self.walkPath)
            #print(self.walkPath, 'move to walk')


    def sendProbe(self):

        if(rospy.get_time() - self.last_probe < self.probe_time):
            return

        self.last_probe = rospy.get_time()

        #print('send probe', self.id)

        message         = {}
        message['type'] = int(MSG.PROBE)
        message['id']   = self.id
        message['src']  = self.getCurrentNode()
        message['idle'] = self.is_idle
        message['terminal']  = 0
        self.network.sendMessage(message)


    def run(self):

        #get the current message
        message = {'type': None}
        if len(self.message_fifo) > 0:
            message = self.message_fifo.pop()
        #print('phase alg', self.phase_alg)
        #print(message)

        if('src' in message):
            distance = self.graph.getDistance(message['src'], self.getCurrentNode())
            # rss = int(self.getRSSmeasurement(message['src']))
            if(distance > self.radius):
                message = {'type': None}

        if(message['type'] == MSG.INFO):
           return
                #return   

        #print('phase alg 2', self.phase_alg)

        if(self.phase_alg == PHASE.INIT):
            self.InitPhase(message)

        if(self.phase_alg == PHASE.SEARCH):
            if(self.is_coord):
                #print('phase alg 3', self.phase_alg)
                self.searchPhaseCoor(message)

                #self.deploy()
            else:
                self.searchAndDeployNonCoor(message)

        if(self.phase_alg == PHASE.DEPLOY):
            self.deployPhase(message)
        #print('run')
        #time.sleep(0.01)


    def InitPhase(self, message):
        #print('entrou')
        if(self.init_tsp):
            print('1')
            mylist = self.graph.graph.keys()
            mylist.insert(0, self.getCurrentNode())
            to_visit = sorted(set(mylist), key=lambda x: mylist.index(x))
            print(list(to_visit), self.getCurrentNode())
            self.graphWalk  = self.graph.calculateTSP(list(to_visit))
            self.init_tsp   = False
            #print('tsp out')


        #self.visited_nodes = self.visited_nodes.union(set([self.getCurrentNode()])) 
            
        if(message['type'] == MSG.PROBE and message['id'] == self.gateway_id):
            print('2')
            self.gateway_src= message['src']
            new_message         = {}
            new_message['id']   = self.id
            new_message['type'] = int(MSG.INIT)             
            self.network.sendMessageTo(message['id'], new_message)
            self.wait_init_ack  = True
            #print('send init', new_message, message['id'])

        if message['type'] == MSG.INIT_ACK:
            print('receive init ack')
            self.is_coord  = message['value'] == 1
            self.is_idle   = True
            self.phase_alg = PHASE.SEARCH
            self.Stall()
            time.sleep(1.0)
            if(self.is_coord):
                mylist = self.graph.graph.keys()
                mylist.insert(0, self.getCurrentNode())
                to_visit = sorted(set(mylist), key=lambda x: mylist.index(x))
                self.dicover_walk       = self.graph.calculateTSP(list(to_visit))
                self.last_discover_walk = self.dicover_walk

            self.message_fifo    = []

            print('search phase', self.is_coord)
            #print('saiu')
            return

        if(not self.wait_init_ack):
            self.graphWalk = self.goNextNode(self.graphWalk)
            print(self.graphWalk)
            print(self.node_id)

        #print('saiu')
    

    # def goNextNode(self, walk):
        

    #     if(walk == []):
    #         return []

    #     if(self.sim < 0):
    #         self.sim = 80


    #     if(self.sim == 0):
    #         self.sim = -1
    #         if (len(walk) == 1 ):
    #             self.node_id = walk[0]
    #             return []         

    #         self.node_id = walk[walk.index(self.node_id)+1]
    #         new_walk = walk[walk.index(self.node_id):]
            
    #         return new_walk

    #     self.sim -=1
    #     return walk


    def goNextNode(self, walk, waittime=-1):
        

        if(walk == []):
            return []

        new_walk__ = False
        if(self.getCurrentNode() in walk and walk.index(self.getCurrentNode()) == 1):
            walk = walk[walk.index(self.getCurrentNode()):]
            new_walk__ = True


        #print(self.status, self.getCurrentNode(), walk, 'go next')

        if (self.status == 3 or self.status == 4 or self.status == 2 or  self.status == 5 or self.status == -1) and self.getCurrentNode() == walk[0]:

            if (len(walk) == 1):
                return []

            if(self.last_node != self.getCurrentNode()):
                self.last_go_next_time = rospy.get_time()
                self.last_node = self.getCurrentNode()

            if( rospy.get_time() - self.last_go_next_time < waittime):
                return walk

            #print('go next node',  self.status , ' ', self.getCurrentNode(), walk[1])


            self.sendDeployment(self.graph.vertice_position[walk[1]], walk[1])
            if(not new_walk__):
                new_walk = walk[walk.index(self.node_id):]
                return new_walk

        return walk

    def isInDestination(self):
        return (self.status == 3)

    def getCurrentNode(self):
        self.node_id = self.graph.getClosestNode(self.position['position'])
        return self.node_id


    def Stall(self):
        goal = GoalID()
        goal.id = str(self.current_goal_id-1)
        print('stall', self.current_goal_id-1)
        self.cancel_pub.publish(goal)


    def getStatus(self, Status):

        if(len(Status.status_list) > 0):
            self.status = Status.status_list[0].status
            #self.current_goal_id = Status.status_list[0].goal_id

    def getMap(self, MapData):

        self.map_resolution = MapData.resolution
        self.height         = MapData.height


    def sendDeployment(self, deployment_position, node):


         
        if(rospy.get_time() - self.last_send_deplyment < self.send_deployment_time and self.send_node == node):
            return
        self.send_node = node
        self.last_send_deplyment = rospy.get_time()

        self.status = -1
        #print(deployment_position)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = deployment_position[0]*self.map_resolution
        pose.pose.position.y = (self.height - deployment_position[1])*self.map_resolution

        print("send deployment_position")
        print(pose.pose.position)

        #for debug
        self.position['destination'] = (deployment_position[0], deployment_position[1])

        #print(pose.pose.position.x, pose.pose.position.y)
        q = tf.transformations.quaternion_from_euler(self.position['position'][2][0], self.position['position'][2][1], self.position['position'][2][2])
        pose.pose.orientation = Quaternion(*q)


        goal_id = GoalID()
        goal_id.id = str(self.current_goal_id)

        self.current_goal_id +=1


        goal = MoveBaseActionGoal()
        goal.goal_id = goal_id
        goal.goal.target_pose = pose


        self.goal_pub.publish(goal)

    def getPose(self, Pose):
        orientation = (
            Pose.pose.pose.orientation.x,
            Pose.pose.pose.orientation.y,
            Pose.pose.pose.orientation.z,
            Pose.pose.pose.orientation.w)
        orientation_euler = tf.transformations.euler_from_quaternion(orientation)

        #self.orientation = Pose.pose.pose.orientation

        #the variance for the kalman filter
        xx = Pose.pose.covariance[0]
        xy = Pose.pose.covariance[1]
        yx = Pose.pose.covariance[6]
        yy = Pose.pose.covariance[7]

        self.covariance = np.matrix([[xx, xy], [yx, yy]])

        self.position['position'] = (Pose.pose.pose.position.x/self.map_resolution, self.height- Pose.pose.pose.position.y/self.map_resolution, orientation_euler)
        
        self.initialized = True
        
        if(rospy.get_time() - self.send_position_time > self.send_position_time_diff):
            message              = {}
            message['type']      = int(MSG.INFO)
            message['id']        = self.id
            message['position']  = self.position['position']
            message['routing']   = self.comm_route 
            message['ended']     = self.deploy_ended
            message['deploy_n']  = self.deploy_numbers
            message['steiner']   = str(self.steiner_graph) 
            self.network.sendMessage(message)
            self.send_position_time = rospy.get_time()        



        
if __name__ == "__main__":
    robot = Robot()
    rate = rospy.Rate(20.0)
    
    while(not robot.start_real and not rospy.is_shutdown()):
        #print('sleep')
        rate.sleep()


    while not rospy.is_shutdown():
        #print('run')
        robot.run()
        rate.sleep()
