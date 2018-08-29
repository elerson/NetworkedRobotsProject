#!/usr/bin/python

from   PyQt4 import QtCore, QtGui, Qt# Import the PyQt4 module we'll need
from   PyQt4.QtCore import QPointF, QElapsedTimer
import sys # We need sys so that we can pass argv to QApplication
from   network_utils.network import Network
import client_ui # This file holds our MainWindow and all design related things

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

import subprocess as sub

class COMMANDS(IntEnum):    
    SETINITALPOSE         = 0
    STARTDEPLOYMENT       = 1
    EXECCOMMAND           = 2
    SETGOAL               = 3

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
        self.pushButton_setGoal.clicked.connect(self.setGoal)

        home = os.path.expanduser("~")
        self.config_data = self.readConfig(home+'/NetworkedRobotsProject/configs/data_real.yaml')


        self.image       = self.loadImage()
        self.widget_image.setImage(self.image)
        self.network     = Network(id=-1,  broadcast_addr = self.config_data['broadcast_address'], port = self.config_data['configuration_port'])
        
        self.log_timer  = QElapsedTimer()
        self.finish_time  = QElapsedTimer()
        self.finish_time.start()
        
        self.commnand_id = 0
        self.updateALL()


    def updateALL(self):
        #print(self.network.rcv_data)

        self.widget_image.addRobots(self.network.rcv_data)

        threading.Timer(0.2, self.updateALL).start()


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


        command['robot_id']     =  int(str(self.lineEdit_id.text()))
        print((initial_position.x(), initial_position.y()))
        command['initial_pose'] =  (initial_position.x()*self.config_data['resolution'], (self.image.height() - initial_position.y())*self.config_data['resolution'])
        command['direction']    =  angle

        print(command)
        self.network.sendMessage(command)


    def setGoal(self):
        initial_position        = self.widget_image.initial_pose
        end_position            = self.widget_image.end_pose

        angle                   = math.atan2(-(end_position.y() - initial_position.y()),  end_position.x() - initial_position.x())
        
        command                 =  {}
        command['id']           =  -1
        command['command']      =  COMMANDS.SETGOAL
        command['command_id']   =  self.commnand_id
        self.commnand_id        += 1


        command['robot_id']     =  int(str(self.lineEdit_id.text()))
        print((initial_position.x(), initial_position.y()))
        command['goal'] =  (initial_position.x()*self.config_data['resolution'], (self.image.height() - initial_position.y())*self.config_data['resolution'])
        command['direction']    =  angle

        print(command)
        self.network.sendMessage(command)


    def executeCommand(self):
        
        command                 =  {}
        command['id']           =  -1
        command['command']      =  int(COMMANDS.EXECCOMMAND)
        command['command_id']   =  self.commnand_id
        self.commnand_id        += 1

        command['robot_id']     =  int(str(self.lineEdit_id.text()))
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



def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    app = QtGui.QApplication(sys.argv)  # A new instance of QApplication
    form = clientApp()                 # We set the form to be our ExampleApp (design)
    form.show()                         # Show the form
    sys.exit(app.exec_())                         # and execute the app
   

if __name__ == '__main__':              # if we're running file directly and not importing it
    main()                              # run the main function
