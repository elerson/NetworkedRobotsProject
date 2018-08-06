#!/usr/bin/python
import rospy
from network_utils.tree import Tree, TreeSegmention
from scipy.optimize import linear_sum_assignment
from network_utils.network import Network
from network_utils.tree import Tree
from network_utils.sceneGraph import sceneGraph
import sys
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
    INFO                    = 11


class Terminal:
    def __init__(self):
        rospy.init_node('anchor_terminal', anonymous=True)
        self.config_file  = self.readConfig(rospy.get_param("~config_file"))

        self.radius       = rospy.get_param("~radius", 100)
        self.real_robot   = not self.config_file['configs']['simulation']

        if(self.real_robot):            
            self.routing         = Routing('teste4', self.config_file, 'ra0')
            self.rss_measure     = RSSMeasure('teste4', self.config_file)
            id                   = self.routing.getID()
        else:
            id                   = rospy.get_param("~id")

        self.client_id    = id

        self.tree_file    = self.config_file['configs']['treefile']


        self.tree         = Tree(self.tree_file)
        self.tree_clients = set(self.tree.clients)

        self.xoffset      = rospy.get_param("~xoffset", 0)
        self.yoffset      = rospy.get_param("~yoffset", 0)

        self.clients_pos  = [ (self.tree.graph_vertex_position[i][0], self.tree.graph_vertex_position[i][1]) for i in self.tree.clients]
        self.radius       = (2.0/3.0)*self.radius
        self.sceneGraph   = sceneGraph(self.config_file['configs'], self.radius, self.clients_pos, (self.xoffset, self.yoffset))

        
        self.msg_id       = {}
        self.coord        = -1
        self.msg_num      = 0 

        self.links        = set([])
        self.is_anchor    = True
        self.solution_start = False
        self.gateway_id   = 1
        self.in_messages_fifo = []
        self.last_probe   = 0
        self.last_message = 0
        self.probe_time   = 0.2

        if self.client_id > len(self.tree.clients):
            sys.exit(0)

        self.tree_clients_id = list(self.tree_clients)[self.client_id]


        position          = self.tree.graph_vertex_position[self.tree_clients_id]
        position          = (position[0], position[1])
        self.node_id      = self.sceneGraph.getClosestNode(position)
        self.position     = (position[0],  position[1])

        print('node id', self.node_id)

        self.network      = Network(self.client_id,  broadcast_addr = self.config_file['configs']['broadcast_address'], port = self.config_file['configs']['algorithm_port'])
        self.network.addMessageCallback(self.receiveNetworkMessage)


    def readConfig(self, config_file):
        with open(config_file, 'r') as stream:
            return yaml.load(stream)

    def receiveNetworkMessage(self, message):
        
        if(message['id'] != self.client_id):
            self.in_messages_fifo.insert(0, message)


    def run(self):
        #
        self.sendProbe()



        if(len(self.in_messages_fifo) <= 0):
            return

        #print(len(self.in_messages_fifo), 'fifo size')
        message = self.in_messages_fifo.pop()
        #print(len(self.in_messages_fifo), 'fifo size')

        #print('rcvd msg', message)
        type_    = message['type']
        id_     = message['id']
        

        if(self.client_id != self.gateway_id):
            return
        
        #print(message, type_ )

        if(type_ == MSG.INIT):

            new_message = {}
            new_message['type'] = int(MSG.INIT_ACK)
            if(self.coord == -1 or self.coord == id_):
                new_message['value'] = 1
                self.coord       = id_
                
            else:
                new_message['value'] = 0
            self.replyMsg(id_, new_message)
            print("init ack")

        elif(type_ == MSG.DISCOVERY_COMPLETE):
            self.links = self.links.union(set(message['links']))

    

        elif(type_ == MSG.SETTLE_COMPLETE):
            new_message = {}
            new_message['type'] = int(MSG.DISCOVERY_START)
            #print(self.solution_start, 'solution_start')
            self.replyMsg(id_, message)


        # elif(type == MSG.MIGRATE):
        #     if(message['dest'] == self.node_id): 
        #         self.is_anchor = True
        #     else:
        #         self.is_anchor = False

        elif(type_ == MSG.DEPLOY):
            #Steiner tree
            return

    def replyMsg(self, to_id, message):

        message['src']  = self.node_id
        message['id']   = self.client_id
        self.network.sendMessageTo(to_id, message)

    def sendProbe(self):
        if(rospy.get_time() - self.last_probe < self.probe_time):
            #print(rospy.get_time() - self.last_probe)
            return

        self.last_probe = rospy.get_time()
        message          = {}
        message['type']  = int(MSG.PROBE)
        message['src']   = self.node_id
        message['terminal']  = 1
        message['id']    = self.client_id
        message['links'] = list(self.links)
        message['idle']  = False
        message['start'] = self.solution_start
        message['position'] = self.position

        self.network.sendMessage(message)


        
if __name__ == "__main__":
    terminal = Terminal()
    rate = rospy.Rate(200.0)
    while not rospy.is_shutdown():
        
        terminal.run()
        rate.sleep()
