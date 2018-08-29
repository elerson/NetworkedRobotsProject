#!/usr/bin/python

from   PyQt4 import QtCore, QtGui, Qt# Import the PyQt4 module we'll need
from   PyQt4.QtCore import QPointF, QElapsedTimer
import sys # We need sys so that we can pass argv to QApplication
from   network_utils.network import Network
from   network_utils.tree    import Tree, TreeSegmention
import client_ui # This file holds our MainWindow and all design related things
              # it also keeps events etc that we defined in Qt Designer
from network_utils.bspline import BSpline

import os
import subprocess as sub
import yaml
from   enum import IntEnum
import math
import time, threading
import glob

import sys
import signal
import math
import argparse

import subprocess as sub


class COMMANDS(IntEnum):    
    SETINITALPOSE         = 0
    STARTDEPLOYMENT       = 1
    EXECCOMMAND           = 2

class clientApp(QtGui.QMainWindow, client_ui.Ui_MainWindow):
    def __init__(self):
        # Explaining super is out of the scope of this article
        # So please google it if you're not familar with it
        # Simple reason why we use it here is that it allows us to
        # access variables, methods etc in the design.py file
        super(self.__class__, self).__init__()
        self.setupUi(self)  # This is defined in design.py file automatically
                            # It sets up layout and widgets that are defined


        self.pushButton_setInitialPos.clicked.connect(self.setInitialPos)
        self.pushButton_execCommand.clicked.connect(self.executeCommand)
        self.pushButton_startDeployment.clicked.connect(self.startDeployment)

        #home = os.path.expanduser("~")

        #
	global config
        self.config_data = self.readConfig(config)
        
        # if(self.config_data['simulation']):
        #     dir_ = os.environ['EXP_DIR']
        #     self.config_data['treefile'] = dir_ + '/steinerData1.dat'
        #     self.config_data['map'] = dir_ + '/map/ambiente.png'

        #     with open(dir_ +'/map/simple.yaml', 'r') as stream:
        #         self.config_data['resolution'] = yaml.load(stream)['resolution']


        self.resolution  = self.config_data['resolution']
        self.treefile    = self.config_data['treefile']
        self.exit        = self.config_data['exit']


        print(self.config_data)


        self.image       = self.loadImage()
        self.widget_image.setImage(self.image)
        self.network     = Network(id=-1, broadcast_addr = self.config_data['broadcast_address'], port = self.config_data['algorithm_port'])
        self.tree        = Tree(self.treefile)
        print(self.tree.getSize(), 'size')
        self.tree_segmentation   = TreeSegmention(self.tree)
        self.tree_segmentation.segmentation_search([], [])
        self.tree_segmentation_segments = []
        self.radius = 110
        segmentation, splines = self.getSegmentation()

        #print(tree_segmentation.segmentaion_paths)
        #segmentation = self.tree_segmentation.evaluate_segmentation(100)

        self.widget_image.setSegementation(segmentation, splines)

        self.widget_image.addTree(self.tree.graph_adj_list, self.tree.graph_vertex_position, self.tree.clients)


        self.solution_started = False
        self.log_id = 0
        self.robots_ids_start    = len(self.tree.vertices)


        self.widget_image.setIdStart(self.robots_ids_start)


        self.log_folder, self.log_data, self.log_network = self.createLog()

        self.log_data_file      = open(self.log_data, 'a', 0)
        self.log_network_file   = open(self.log_network, 'a', 0)


        self.log_timer  = QElapsedTimer()
        self.finish_time  = QElapsedTimer()
        self.finish_time.start()
        #create a new log folder
        self.widget_image.setLogFolder(self.log_folder)

        self.commnand_id = 0
        self.updateALL()


    def getSegmentation(self):
        if(not self.tree_segmentation_segments == []):
            return self.tree_segmentation_segments, self.splines
        self.tree_segmentation.segmentation_search([], [])
        #print(tree_segmentation.segmentaion_paths)
        segmentation = self.tree_segmentation.evaluate_segmentation(self.radius)
        self.tree_segmentation_segments = segmentation

        #create the spline
        self.splines = {}
        i = 0
        for segment in segmentation:
            x = []
            y = []
            for index in segment:
                p = self.tree.graph_vertex_position[index]
                x.append(p[0])
                y.append(p[1])
                #print('teste', self.height)
            #print(x, y)
            self.splines[i] = BSpline(x,y, 10.0)
            i += 1




        return segmentation, self.splines

    def createLog(self):
        log_dir_ = os.environ['LOG_DIR']
        folders = glob.glob('Logs/'+log_dir_+'/*')
        folder = 'Logs/' +log_dir_+ '/log' + str(len(folders) + 1)

        os.makedirs(folder)
        log_folder = folder
        log_data = folder + '/log.txt'
        log_network = folder + '/network.txt'

        print(log_folder, log_data, log_network)

        return log_folder, log_data, log_network
            

    def closeRos(self):
        sub.Popen(('killall', 'roslaunch'))

    def updateALL(self):
       
        self.solutionStarted()
        #update robots draw
        self.widget_image.addRobots(self.network.rcv_data)

        #update connections 

        #update log
        graph = self.createRoutingGraph()
        min_distance, max_distance = self.getMinAndMaxDistances(graph)
        simulation_time = self.getSimulationTime()
        connected, num_connected = self.verifyNetworkConcluded()

        self.widget_image.addConnections(graph)

        self.saveLog(min_distance, max_distance, simulation_time, num_connected)
        self.saveNetworkLog()
        self.widget_image.saveLog()

        #update routing

        #verify experiment exit
        
        if((connected and num_connected > 1 and self.exit) or (self.exit and (self.finish_time.elapsed() > 600000 ))):
            self.closeRos()
            self.close()

        threading.Timer(0.2, self.updateALL).start()

    def saveLog(self, min_distance, max_distance, simulation_time, num_connected):
        str_data = str(min_distance) + ',' + str(max_distance) + ',' + str(simulation_time) + ',' + str(num_connected) + '\n'
        #print(str_data)
        self.log_data_file.write(str_data)

    def getSimulationTime(self):
        return self.log_timer.elapsed()

    def getDistance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)*self.resolution

    def saveNetworkLog(self):
        str_data = str(self.network.rcv_data) + '\n'
        self.log_network_file.write(str_data)

        #print log in file

    def solutionStarted(self):
        if(self.solution_started):
            return True

        started = 0
        for id in self.network.rcv_data:
            if(id < 0):
                continue

            started = max(started, self.network.rcv_data[id]['started'])
        self.solution_started = (started == 1)

        if(self.solution_started):
            self.log_timer.start()
            print('Started', )

        return (started == 1)

    def getPositionByID(self, id):
        #print(self.network.rcv_data)
        ##Client or tree junctions id
        if(id < self.robots_ids_start):
            position = self.tree.graph_vertex_position[id]
            return position
        else:
            return self.network.rcv_data[id]['position']


    def getMinAndMaxDistances(self, graph):

        min_distance  = float('inf')
        max_distance  = 0 
        for node1 in graph:
            p1 = self.getPositionByID(node1)
            if( node1 >= self.robots_ids_start and self.network.rcv_data[node1]['routing'] !=  []):
                for node2 in graph[node1]:
                    p2 = self.getPositionByID(node2)
                    distance = self.getDistance(p1, p2)
                    min_distance = min(distance, min_distance)
                    max_distance = max(distance, max_distance)
        return min_distance, max_distance

    def createRoutingGraph(self):
        graph = {}
        for id in self.network.rcv_data:
            if id < 0:
                continue
                
            graph[id] = set([])

        for id in self.network.rcv_data:
            if id < 0:
                continue
            graph[id] = graph[id].union(set(self.network.rcv_data[id]['routing']))
            for neigbor in self.network.rcv_data[id]['routing']:
                try:
                    graph[neigbor] = graph[neigbor].union(set([id]))
                except:
                    graph[neigbor] = set([id])


        for id in self.network.rcv_data:
            if id < 0:
                continue

            graph[id] = list(graph[id])
        return graph

        #self.re

    def readConfig(self, config_file):
        with open(config_file, 'r') as stream:
            return yaml.load(stream)['configs']

    def setInitialPos(self):
        initial_position        = self.widget_image.initial_pose
        end_position            = self.widget_image.end_pose

        angle                   = math.atan2(-(end_position.y() - initial_position.y()),  end_position.x() - initial_position.x())
        
        command                 =  {}
        command['id']           =  -1
        command['command']      =  COMMANDS.SETINITALPOSE
        command['command_id']   =  self.commnand_id
        self.commnand_id        += 1


        command['robot_id']     =  str(self.lineEdit_id.text())
        command['initial_pose'] =  (initial_position.x(), initial_position.y())
        command['direction']    =  angle

        print(command)
        self.network.sendMessage(command)

    def executeCommand(self):
        
        command                 =  {}
        command['id']           =  -1
        command['command']      =  int(COMMANDS.EXECCOMMAND)
        command['command_id']   =  self.commnand_id
        self.commnand_id        += 1

        command['robot_id']     =  str(self.lineEdit_id.text())
        command['execute']      =  str(self.lineEdit_command.text())

        print(command)
        self.network.sendMessage(command)


    def startDeployment(self):
        command                 =  {}
        command['id']           =  -1
        command['command']      =  int(COMMANDS.STARTDEPLOYMENT)
        command['command_id']   =  self.commnand_id
        self.commnand_id        += 1

        print(command)
        self.network.sendMessage(command)

    def loadImage(self):
        return QtGui.QImage(self.config_data['map'])

    def verifyNetworkConcluded(self):
        #cefsm
        connected = True
        ended     = False
        num_connected = 0
        ids = self.network.getDataIds()
        for id in ids:

            if 'diff' in self.network.rcv_data[id]:
                #print(abs(self.network.rcv_data[id]['diff']))
                print(abs(self.network.rcv_data[id]['diff']))
                distance = self.network.rcv_data[id]['radius']
                gamma = 4.0
                print('distance ', self.network.rcv_data[id]['radius'])
                radius = 10*gamma*math.log(distance+0.1)-10*gamma*math.log(distance-0.1)

                if abs(self.network.rcv_data[id]['diff']) > radius:
                    connected = False
                else:
                    if(self.network.rcv_data[id]['routing'] != []):
                        num_connected += 1
                ended = True

            if 'state' in self.network.rcv_data[id]:
                if self.network.rcv_data[id]['state'] == 3:
                    num_connected  += 1
                if not (self.network.rcv_data[id]['state'] == 0 or self.network.rcv_data[id]['state'] == 3):
                    connected = False 

                ended = ended or self.network.rcv_data[id]['ended']

        if len(ids) > 0 and 'diff' in self.network.rcv_data[ids[0]]:
            connected = connected and (num_connected >= self.network.rcv_data[ids[0]]['s_size'])

        connected = connected and ended
        return connected, num_connected

    def close(self):
        sub.Popen(('kill', '-9', str(os.getpid())))

    def closeEvent(self, event):
        sub.Popen(('kill', '-9', str(os.getpid())))

def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    app = QtGui.QApplication(sys.argv)  # A new instance of QApplication
    form = clientApp()                 # We set the form to be our ExampleApp (design)
    form.show()                         # Show the form
    sys.exit(app.exec_())                         # and execute the app
   

if __name__ == '__main__':              # if we're running file directly and not importing it
    global config
    parser = argparse.ArgumentParser(description='Visualization script:\n "./client -c ~/data.yaml"')

    parser.add_argument('-c', action='store', dest='config', help='Path where de data is at', required=True)

    pargs = parser.parse_args()
    
    config = pargs.config   
    main()                              # run the main function



