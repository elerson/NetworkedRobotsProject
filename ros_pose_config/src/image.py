from PyQt4 import QtCore, QtGui, Qt# Import the PyQt4 module we'll need
from PyQt4.QtCore import QPointF
from math import ceil, log
from network_utils.bspline import BSpline
import numpy as np


class Image(QtGui.QWidget):
    def __init__(self, widget):      
        super(Image, self).__init__()
        self.pressed             = False
        self.press_position      = QPointF(0, 0)
        self.offset              = QPointF(0, 0)
        self.offset_             = QPointF(0, 0)
        self.pressed_button      = -1
        

        self.robots              = {}

        self.initial_pose        = QPointF(0, 0)
        self.end_pose            = QPointF(0, 0)
        self.image_number        = 2
        self.pixmap  = QtGui.QImage(960, 529, QtGui.QImage.Format_RGB32)
        self.communication_graph = {}

  
    def setIdStart(self, start):
        self.robots_ids_start = start


    def setImage(self, image):
        self.image = image


    def addRobots(self, robots):
        for robot_id in robots:
            self.robots[robot_id] = robots[robot_id]['position']


    def getPositionByID(self, id):
        return self.robots[id]


    def saveImage(self, filename):
        
        painter = QtGui.QPainter(self.pixmap)
        self.plotImage(painter)

        self.pixmap.save(filename, 'jpg')
    

    def paintEvent(self, e):
        painter = QtGui.QPainter(self)
        self.plotImage(painter)    

        #for robot_id in self.robot_initials:
        #    
    def plotImage(self, painter):
        painter.drawImage(self.offset +  self.offset_, self.image)


        pen = QtGui.QPen()
       
        color = QtGui.QColor(255, 0, 0)
        pen.setColor(color)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.drawLine(self.offset +  self.offset_ + self.initial_pose, self.offset +  self.offset_ + self.end_pose)
        pen.setWidth(6)
        painter.setPen(pen)
        painter.drawPoint(self.offset +  self.offset_ + self.end_pose)

        painter.drawPoint(self.offset +  self.offset_ + self.press_position)



        for robot_id in self.robots:
            position = QPointF(self.robots[robot_id][0], self.robots[robot_id][1])
            painter.drawPoint(self.offset +  self.offset_ + position)


    def mousePressEvent(self, e):
        #print(e.button())
        if(e.button() == QtCore.Qt.LeftButton):
            self.press_position = e.pos()

        if(e.button() == QtCore.Qt.RightButton):
            self.initial_pose   = e.pos() - self.offset -  self.offset_

        self.pressed_button = e.button()


    def mouseMoveEvent(self, e):
        
        if(self.pressed_button == QtCore.Qt.LeftButton):
            self.offset = e.pos() - self.press_position
            self.repaint()

        if(self.pressed_button == QtCore.Qt.RightButton):
            self.end_pose   = e.pos() - self.offset -  self.offset_
        self.repaint()


    def mouseReleaseEvent(self, e):
        if(e.button() == QtCore.Qt.LeftButton):
            self.offset   = QPointF(0, 0)
            self.offset_ += e.pos() - self.press_position

        if(e.button() == QtCore.Qt.RightButton):
            self.end_pose   = e.pos() - self.offset -  self.offset_

        
        self.pressed_button = -1
        self.repaint()


        
