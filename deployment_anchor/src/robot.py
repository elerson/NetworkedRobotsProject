#!/usr/bin/python

#!/usr/bin/python
import rospy
from network_utils.tree import Tree
from scipy.optimize import linear_sum_assignment
from network_utils.network import Network
from network_utils.sceneGraph import sceneGraph
from enum import IntEnum
import yaml
import time


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

class STEP(IntEnum):
    DISCOVER                = 0
    MIGRATE                 = 1
    SETTLE                  = 2


class PHASE:
    INIT                    = 0
    SEARCH                  = 1

class Robot:
    def __init__(self):
        rospy.init_node('anchor_robot', anonymous=True)
        self.config_file     = self.readConfig(rospy.get_param("~config_file"))

        self.tree_file       = self.config_file['configs']['treefile']
        self.tree            = Tree(self.tree_file)
        self.clients         = set(self.tree.clients)

        self.ros_id          = rospy.get_param("~id")

        self.id              = self.ros_id + len(self.clients)

        self.node_id         = 0 



        self.is_coord        = False
        self.radius          = rospy.get_param("~radius", 100)
        self.graph           = sceneGraph(self.config_file['configs']['map'], self.radius)
        self.visited_nodes   = set([])
        self.wait_init_ack   = False
        self.step            = STEP.DISCOVER
        self.is_idle         = True
        self.discovery_start = False
        self.dicover_walk    = []
        self.gateway_id      = 0
        self.gateway_node_id = -1
        self.is_anchored     = True


        self.last_probe   = 0
        self.probe_time   = 0.2


        self.candAnchor      = set([])
        self.prevCandAnchor  = set([])
        self.links           = set([])
        self.walkPath        = set([])
        self.message_fifo    = []
        self.sim             = -1
        self.phase_alg       = PHASE.INIT

        self.network         = Network(self.id)
        self.network.addMessageCallback(self.receiveMessage)
        self.init_tsp        = True
        self.allow_go_next_node = False
        
        self.level           = 0
        self.search_queue_level    = {}


        self.start_noncoor   = False

        self.robot_position_ids  = {}


    def readConfig(self, config_file):
        with open(config_file, 'r') as stream:
            return yaml.load(stream)


    def receiveMessage(self, message):
        self.message_fifo.insert(0, message)

    def searchPhaseCoor(self, message):

        if(message['type'] == MSG.PROBE and not message['terminal']):
            self.robot_position_ids[message['id']] = message['src']
            if(not self.start_noncoor):
                return


        if self.step == STEP.DISCOVER:
            if(self.is_idle and self.discovery_start):
                to_visit = list((set(self.graph.graph.keys()) - self.visited_nodes).union(set([self.gateway_src])))

                to_visit.insert(0, self.node_id)
                to_visit = sorted(set(to_visit), key=lambda x: to_visit.index(x))


                self.dicover_walk       = self.graph.calculateTSP(to_visit)
                self.last_discover_walk = self.dicover_walk
                self.is_idle            = False
                self.discovery_start    = False


            if message['type'] == MSG.PROBE:
                self.links      = self.links.union((self.node_id, message['src']))
                print(message)
                if(message['terminal']):
                    self.search_queue_level[self.getCurrentNode()] = 0
                    #print(message, 'terminal')
                else:
                    if(message['src'] in self.search_queue_level and self.node_id not in self.search_queue_level):
                        self.search_queue_level[self.node_id] = self.search_queue_level[message['src']] + 1
                    pass


            self.dicover_walk = self.goNextNode(self.dicover_walk)
            print(self.dicover_walk, self.search_queue_level)
            #print(self.search_queue_level, self.getCurrentNode(), self.graph.getDistance(8, self.getCurrentNode()), 'discover walk')
            #if( 'src' in message):
            #    print(self.getCurrentNode(), self.graph.getDistance(message['src'], self.getCurrentNode()), 'new_walk')

            if(self.dicover_walk == []):
                self.is_idle = False

                if(set(self.graph.graph.keys()).issubset(set(self.search_queue_level.keys()))):
                    exit()
                

                self.to_visit = list(set([k for k, v in self.search_queue_level.items() if self.level == v]) - self.visited_nodes)
                print('to visit', self.to_visit)
                if(self.to_visit == []):
                    #self.is_idle = True
                    self.level  += 1
                    #self.step    = STEP.SETTLE


                self.step = STEP.MIGRATE
                self.to_visit = list(set([k for k, v in self.search_queue_level.items() if self.level == v]) - self.visited_nodes)
                self.visiting = []



                migrate_visit = list(set([v for k, v in self.robot_position_ids.items()]))
                print('migrate visit', migrate_visit)
                migrate_visit.insert(0, self.node_id)
                migrate_visit = sorted(set(migrate_visit), key=lambda x: migrate_visit.index(x))
                
                #for key in self.robot_position_ids:
                #    self.robot_position_ids[key] = -1

                self.message_fifo    = []
                self.start_noncoor = True
                self.migrateWalk = self.graph.calculateTSP(migrate_visit)
                print('migrate walk', self.migrateWalk)

        elif self.step == STEP.MIGRATE:
            if( message['type'] == MSG.PROBE and not message['src'] in self.visiting):                  
                print('probe', message['id'])

                if(abs(message['id']) >= len(self.clients) and len(self.to_visit) > 0 ):
                    dest                 = self.to_visit.pop()

                    new_message          = {}
                    new_message['type']  = int(MSG.MOVETO)
                    new_message['id']    = self.id
                    new_message['phase'] = int(self.step)
                    new_message['dest']  = dest

                    self.robot_position_ids[message['id']] = dest

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

                    self.robot_position_ids[message['id']] = self.gateway_src
                    self.visiting.append(self.gateway_src)

                    self.network.sendMessageTo(message['id'], new_message)


            self.migrateWalk = self.goNextNode(self.migrateWalk)
            print(self.migrateWalk, 'migrate', self.visiting)
            if(self.migrateWalk == []):
                self.step = int(STEP.SETTLE)
                self.message_fifo    = []

                self.allow_go_next_node = True

                mylist = list(self.visiting)
                mylist.insert(0, self.node_id)
                to_visit = sorted(set(mylist), key=lambda x: mylist.index(x))
                self.settleWalk = self.graph.calculateTSP(to_visit)
                print('b visiting settle', self.visiting)


        elif self.step == STEP.SETTLE:

            print('b visiting settle', self.visiting)
            #print(message, self.visiting, self.getCurrentNode())
            if((self.allow_go_next_node) or (message['type'] == MSG.PROBE and message['src'] in self.visiting) or (not self.getCurrentNode() in self.visiting)):
                print('settle walk', self.settleWalk)
                last_node_id = self.getCurrentNode()               
                self.settleWalk = self.goNextNode(self.settleWalk)
                self.allow_go_next_node = last_node_id == self.getCurrentNode()
            
            print('visiting settle', self.visiting)

            if message['type'] == MSG.PROBE:
                if(self.settleWalk == []):
                    self.discovery_start = True
                    self.step            = int(STEP.DISCOVER)
                    self.message_fifo    = []
                    self.is_idle    = True





    def searchAndDeployNonCoor(self, message):

        #if(not message['type'] == None and abs(message['id']) >= len(self.clients)):
        #    self.phase = message['phase']

        #if not self.solution_start:
        #    return
            

        if self.is_anchored:
            self.sendProbe()
            if(message['type'] == MSG.MOVETO):
                print(message, 'move to', self.getCurrentNode(),  message['dest'])
                self.is_anchored = False
                self.phase       = message['phase']
                if(message['dest'] != self.getCurrentNode()):
                    self.walkPath   = self.graph.getShortestPath(message['dest'],  self.getCurrentNode())
                else:
                    self.walkPath   = []

        else:
            if self.walkPath == []:
                self.is_anchored = True
                self.is_idle     = True
            self.walkPath = self.goNextNode(self.walkPath)
            print(self.walkPath, 'move to walk')


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

        #print(message)

        if('src' in message):
            distance = self.graph.getDistance(message['src'], self.getCurrentNode())
            if(distance > self.radius):
                return   

        if(self.phase_alg == PHASE.INIT):
            self.InitPhase(message)

        if(self.phase_alg == PHASE.SEARCH):
            if(self.is_coord):
                self.searchPhaseCoor(message)
                #self.deploy()
            else:
                self.searchAndDeployNonCoor(message)
        #print('run')
        #time.sleep(0.01)


    def InitPhase(self, message):
        if(self.init_tsp):
            self.graphWalk  =  self.graph.calculateTSP(self.graph.graph.keys())
            self.init_tsp   = False


        #self.visited_nodes = self.visited_nodes.union(set([self.getCurrentNode()])) 
            
        if(message['type'] == MSG.PROBE and message['id'] == self.gateway_id):
            self.gateway_src= message['src']
            new_message         = {}
            new_message['id']   = self.id
            new_message['type'] = int(MSG.INIT)             
            self.network.sendMessageTo(message['id'], new_message)
            self.wait_init_ack  = True
            print('send init', new_message, message['id'])

        if message['type'] == MSG.INIT_ACK:
            print('receive init ack')
            self.is_coord  = message['value'] == 1
            self.is_idle   = True
            self.phase_alg = PHASE.SEARCH

            if(self.is_coord):
                mylist = self.graph.graph.keys()
                mylist.insert(0, self.node_id)
                to_visit = sorted(set(mylist), key=lambda x: mylist.index(x))
                self.dicover_walk       = self.graph.calculateTSP(list(to_visit))
                self.last_discover_walk = self.dicover_walk 

            print('search phase') 
            return

        if(not self.wait_init_ack):
            self.graphWalk = self.goNextNode(self.graphWalk)
            print(self.graphWalk)
            print(self.node_id)

    

    def goNextNode(self, walk):
        

        if(walk == []):
            return []

        if(self.sim < 0):
            self.sim = 80


        if(self.sim == 0):
            self.sim = -1
            if (len(walk) == 1 ):
                self.node_id = walk[0]
                return []         

            self.node_id = walk[walk.index(self.node_id)+1]
            new_walk = walk[walk.index(self.node_id):]
            
            return new_walk

        self.sim -=1
        return walk

    def getCurrentNode(self):
        #self.node_id = self.sceneGraph.getClosestNode(self.position)
        return self.node_id



        
if __name__ == "__main__":
    robot = Robot()
    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        
        robot.run()
        rate.sleep()
