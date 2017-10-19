#!/usr/bin/python

from PyQt4 import QtGui # Import the PyQt4 module we'll need
from PyQt4.QtGui import QImage, QPainter, QPen, QColor
from PyQt4.QtCore import QPoint
import sys # We need sys so that we can pass argv to QApplication

import visualizer_ui # This file holds our MainWindow and all design related things
              # it also keeps events etc that we defined in Qt Designer
from network import Network
import PyQt4.QtCore as QtCore
import signal
from tree import Tree, TreeSegmention


class Visualizer(QtGui.QMainWindow, visualizer_ui.Ui_VisualizerWindow):
    def __init__(self, image_file, tree_file, ray):
        # Explaining super is out of the scope of this article
        # So please google it if you're not familar with it
        # Simple reason why we use it here is that it allows us to
        # access variables, methods etc in the design.py file
        super(self.__class__, self).__init__()
        self.setupUi(self)  # This is defined in design.py file automatically
                            # It sets up layout and widgets that are define

        self.map = QImage(image_file)
        self.height = self.map.height()

        self.ray = ray
        self.tree = Tree(tree_file)
        self.tree_segmention = TreeSegmention(self.tree)
        self.tree_segmentation_segments = []
        self.segmentation = self.getSegmentation()
        self.tree_allocation_positions = self.tree_segmention.doAllocation(self.ray)


        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.draw)
        self.timer.start(500)



    def getAllRobotsPositions(self):

        robot_positions = []
        for data_id in self.network.rcv_data:           
            position = self.network.rcv_data[data_id]['position']
            destination = self.network.rcv_data[data_id]['destination']


            robot_positions.append((position[0], self.height - position[1], destination[0], self.height - destination[1]))

        print(robot_positions)

        return robot_positions

    def getSegmentation(self):
        if(not self.tree_segmentation_segments == []):
            return self.tree_segmentation_segments
        self.tree_segmention.segmentation_search([], [])
        #print(tree_segmentation.segmentaion_paths)
        segmentation = self.tree_segmention.evaluate_segmentation()
        self.tree_segmentation_segments = segmentation
        return segmentation

    def paintEvent(self,event):
        painter = QPainter()
        painter.begin(self)
        point = QPoint(0,0)
        painter.drawImage (point, self.map)

        #drawing the robots positions

        pen = painter.pen()
        pen.setWidth(5)
        # #pen.setColor(QColor(255, 0, 0, 255))
        # painter.setPen(pen)
        # print(self.tree_allocation_positions)
        # for i in range(self.tree_allocation_positions.shape[0]):
        #     painter.drawPoint(self.tree_allocation_positions[i,0], self.tree_allocation_positions[i,1]) 

        # pen.setColor(QColor(255, 0, 0, 255))
        # painter.setPen(pen)
        # for client in self.tree.clients:
        #     position = self.tree.graph_vertex_position[client]
        #     painter.drawPoint(position[0], position[1])
        for segment in self.segmentation[2:]:
            for i in range(1, len(segment)):
                p = self.tree.graph_vertex_position[segment[i-1]]
                q = self.tree.graph_vertex_position[segment[i]]
                painter.drawLine(p[0], p[1], q[0], q[1])
            break

        painter.end()



    def draw(self):
        print("teste")
        self.repaint()
        pass



def main():

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtGui.QApplication(sys.argv)  # A new instance of QApplication
    form = Visualizer(sys.argv[1], sys.argv[2], int(sys.argv[3]))                 # We set the form to be our ExampleApp (design)
    form.show()                         # Show the form
    app.exec_()                         # and execute the app



if __name__ == '__main__':              # if we're running file directly and not importing it
    main()                              # run the main function