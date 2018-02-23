#!/usr/bin/python

from   PyQt4 import QtCore, QtGui, Qt# Import the PyQt4 module we'll need
from   PyQt4.QtCore import QPointF, QElapsedTimer
import sys # We need sys so that we can pass argv to QApplication
from   network import Network
from   tree    import Tree
import client_ui # This file holds our MainWindow and all design related things
              # it also keeps events etc that we defined in Qt Designer

import os
import subprocess as sub
import yaml
from   enum import IntEnum
import math
import time, threading
import glob

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

        self.config_data = self.readConfig('../configs/data.yaml')
        self.resolution  = self.config_data['resolution']
        self.treefile    = self.config_data['treefile']
        self.exit        = self.config_data['exit']
        print(self.config_data)


        self.image       = self.loadImage()
        self.widget_image.setImage(self.image)
        self.network     = Network()
        self.tree        = Tree(self.treefile)
        self.widget_image.addTree(self.tree.graph_adj_list, self.tree.graph_vertex_position, self.tree.clients)

        self.solution_started = False
        self.log_id = 0


        self.log_folder, self.log_data, self.log_network = self.createLog()

        self.log_data_file      = open(self.log_data, 'a')
        self.log_network_file   = open(self.log_network, 'a')


        self.log_timer  = QElapsedTimer()
        #create a new log folder
        self.widget_image.setLogFolder(self.log_folder)

        self.commnand_id = 0
        self.updateALL()

    def createLog(self):
        folders = glob.glob('Logs/*')
        folder = 'Logs/' + 'log' + str(len(folders) + 1)

        os.mkdir(folder)
        log_folder = folder
        log_data = folder + '/log.txt'
        log_network = folder + '/network.txt'

        return log_folder, log_data, log_network
            

    def closeRos(self):
        sub.Popen(('killall', 'roslaunch'))

    def updateALL(self):
       
        #update robots draw
        self.widget_image.addRobots(self.network.rcv_data)

        #update connections 

        #update log
        graph = self.createRoutingGraph()
        min_distance, max_distance = self.getMinAndMaxDistances(graph)
        simulation_time = self.getSimulationTime()

        self.saveLog(min_distance, max_distance, simulation_time)
        self.saveNetworkLog()
        self.widget_image.saveLog()

        #update routing

        #verify experiment exit
        connected, num_connected = self.verifyNetworkConcluded()
        if(connected and num_connected > 0 and self.exit):
            self.closeRos()

        threading.Timer(0.2, self.updateALL).start()

    def saveLog(self, min_distance, max_distance, simulation_time):
        str_data = str(min_distance) + ',' + str(max_distance) + ',' + str(simulation_time)
        self.log_data_file.write(str_data)

    def getSimulationTime(self):
        return self.log_timer.elapsed()

    def getDistance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    def saveNetworkLog(self):
        str_data = str(self.network.rcv_data)
        self.log_network_file.write(str_data)

        #print log in file

    def solutionStarted(self):
        if(self.solution_started):
            return True

        started = 0
        for id in self.network.rcv_data:
            started = max(started, self.network.rcv_data[id]['started'])
        self.solution_started = (started == 1)

        if(self.solution_started):
            self.log_timer.start()

        return (started == 1)

    def getMinAndMaxDistances(self, graph):

        min_distance  = float('inf')
        max_distance  = 0 
        for node1 in graph:
            p1 = self.self.network.rcv_data[node1]['position']
            if( self.self.network.rcv_data[node1]['routing'] !=  []):
                for node2 in graph[node1]:
                    p2 = self.self.network.rcv_data[node2]['position']
                    distance = self.getDistance(p1, p2)
                    min_distance = min(distance, min_distance)
                    max_distance = max(distance, max_distance)
        return min_distance, max_distance

    def createRoutingGraph(self):
        graph = {}
        for id in self.network.rcv_data:
            graph[id] = set([])

        for id in self.network.rcv_data:
            graph[id] = graph[id].union(set(self.network.rcv_data[id]['routing']))
            for neigbor in self.network.rcv_data[id]['routing']:
                graph[neigbor] = graph[neigbor].union(set([id]))


        for id in self.network.rcv_data:
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
        num_connected = 0
        for id in self.network.rcv_data:

            if 'diff' in self.network.rcv_data[id]:
                if abs(self.network.rcv_data[id]) > 1:
                    connected = False
                    num_connected += 1

            if 'state' in self.network.rcv_data[id]:
                if self.network.rcv_data[id]['state'] == 3:
                    self.num_connected  += 1
                if self.network.rcv_data[id]['state'] != 0 or self.network.rcv_data[id]['state'] != 3:
                    connected = False 

        return connected, num_connected


def main():
    app = QtGui.QApplication(sys.argv)  # A new instance of QApplication
    form = clientApp()                 # We set the form to be our ExampleApp (design)
    form.show()                         # Show the form
    app.exec_()                         # and execute the app


if __name__ == '__main__':              # if we're running file directly and not importing it
    main()                              # run the main function
