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

    def setSegementation(self, segments, splines):
        self.segments = segments
        self.splines  = splines

    def setIdStart(self, start):
        self.robots_ids_start = start


    def setImage(self, image):
        self.image = image


    def setLogFolder(self, folder):
        self.log_folder = folder


    def addTree(self, graph, positions, clients):

        self.graph     = graph
        self.positions = positions
        self.clients   = clients

    def addRobots(self, robots):
        for robot_id in robots:
            if (robot_id >= 0):
                self.robots[robot_id] = robots[robot_id]['position']
                print(self.robots[robot_id])

    def addConnections(self, graph):
        self.communication_graph = graph

    def getPositionByID(self, id):
        #print(self.network.rcv_data)
        ##Client or tree junctions id
        if(id < self.robots_ids_start):
            position = self.positions[id]
            return position
        else:
            return self.robots[id]

    def saveLog(self):
        number = self.image_number
        filename = '/image' + ''.zfill(8-int(ceil(log(number, 10)))) + str(number) + '.jpg'
        self.saveImage(self.log_folder + filename)

        self.image_number += 1

    def paintSegments(self, painter):
        pen_back = painter.pen()
        pen = QtGui.QPen()
        #print connections
        color = QtGui.QColor(0,100,255)
        pen.setColor(color)
        pen.setWidth(2)
        painter.setPen(pen)
        k = 1
        for segment in self.segments:
            color = QtGui.QColor(k*30,100,255)
            pen.setColor(color)
            pen.setWidth(2 + k*2)
            painter.setPen(pen)
            k += 1
            for s in range(1,len(segment)):
                i = segment[s-1]
                j = segment[s]
                initial_pose = QPointF(self.positions[i][0], self.positions[i][1])
                end_pose     = QPointF(self.positions[j][0], self.positions[j][1])
                painter.drawLine(self.offset +  self.offset_ + initial_pose, self.offset +  self.offset_ + end_pose)
        painter.setPen(pen_back)

    def paintSplines(self, painter):
        pen_back = painter.pen()
        pen = QtGui.QPen()
        #print connections
        color = QtGui.QColor(0,100,255)
        pen.setColor(color)
        pen.setWidth(2)
        painter.setPen(pen)
        k = 1
        for spline in [0]:
            color = QtGui.QColor(k*30,100,255)
            pen.setColor(color)
            pen.setWidth(2 + k*2)
            painter.setPen(pen)
            k += 1

            for s in range(1,len(self.splines[spline].x)):

                initial_pose = QPointF(self.splines[spline].x[s-1], self.splines[spline].y[s-1])
                end_pose     = QPointF(self.splines[spline].x[s], self.splines[spline].y[s])
                #painter.drawLine(self.offset +  self.offset_ + initial_pose, self.offset +  self.offset_ + end_pose)


            tt = np.linspace(0.001, 0.999, num=1000)

            for s in range(1,len(tt)):
                x0, y0 = self.splines[spline].get(tt[s])
                x1, y1 = self.splines[spline].get(tt[s])

                initial_pose = QPointF(x0, y0)
                end_pose     = QPointF(x1, y1)

                painter.drawLine(self.offset +  self.offset_ + initial_pose, self.offset +  self.offset_ + end_pose)


        color = QtGui.QColor(255, 0, 0)
        pen.setColor(color)
        pen.setWidth(8)
        painter.setPen(pen)
        #print(self.press_position.x)
        t, closest_point = self.splines[spline].getClosestPoint(int(self.press_position.x()), int(self.press_position.y()))
        painter.drawPoint(self.offset +  self.offset_ + QPointF(closest_point[0], closest_point[1]))
        painter.setPen(pen_back)


    def paintTree(self, painter):

        pen_back = painter.pen()
        pen = QtGui.QPen()
        pen.setWidth(2)
        painter.setPen(pen)
        #print(self.positions)
        for i in self.graph:
            for j in self.graph[i]:
                initial_pose = QPointF(self.positions[i][0], self.positions[i][1])
                end_pose     = QPointF(self.positions[j][0], self.positions[j][1])
                painter.drawLine(self.offset +  self.offset_ + initial_pose, self.offset +  self.offset_ + end_pose)
        
        pen.setWidth(8)
        painter.setPen(pen)
        #print robots
        for client in self.clients:
            position = QPointF(self.positions[client][0], self.positions[client][1])
            painter.drawPoint(self.offset +  self.offset_ + position)


        painter.setPen(pen_back)

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

        self.paintTree(painter)
        #self.paintSegments(painter)
        #self.paintSplines(painter)
        pen = QtGui.QPen()
        #print connections
        color = QtGui.QColor(255,215,0)
        pen.setColor(color)
        pen.setWidth(2)
        painter.setPen(pen)
        #print connections
        for node1 in self.communication_graph:
            for node2 in self.communication_graph[node1]:
                p = self.getPositionByID(node1)
                p1 = QPointF(p[0], p[1])

                p = self.getPositionByID(node2)
                p2 = QPointF(p[0], p[1])

                painter.drawLine(self.offset +  self.offset_ + p1, self.offset +  self.offset_ + p2)
        #print(self.offset +  self.offset_)

            #print(position)

        #paint tree

       
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


        
