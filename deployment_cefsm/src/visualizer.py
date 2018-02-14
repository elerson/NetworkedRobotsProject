#!/usr/bin/python

from PyQt4 import QtGui # Import the PyQt4 module we'll need
from PyQt4.QtGui import QImage, QPainter, QPen
from PyQt4.QtCore import QPoint
import sys # We need sys so that we can pass argv to QApplication

import visualizer_ui # This file holds our MainWindow and all design related things
              # it also keeps events etc that we defined in Qt Designer
from network import Network
import PyQt4.QtCore as QtCore
import signal


class Visualizer(QtGui.QMainWindow, visualizer_ui.Ui_VisualizerWindow):
    def __init__(self, image_file):
        # Explaining super is out of the scope of this article
        # So please google it if you're not familar with it
        # Simple reason why we use it here is that it allows us to
        # access variables, methods etc in the design.py file
        super(self.__class__, self).__init__()
        self.setupUi(self)  # This is defined in design.py file automatically
                            # It sets up layout and widgets that are defined
        self.map = QImage(image_file)
        self.height = self.map.height()

        self.network = Network()


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


    def paintEvent(self,event):
        painter = QPainter()
        painter.begin(self)
        point = QPoint(0,0)
        painter.drawImage (point, self.map)

        #drawing the robots positions

        pen = QPen()
        pen.setWidth(5)
        painter.setPen(pen)
        positions = self.getAllRobotsPositions()
        for position in positions:
            pen.setColor(QtCore.Qt.black)
            painter.drawPoint(position[0], position[1])
            painter.drawLine(position[0], position[1], position[2], position[3])
            pen.setColor(QtCore.Qt.red)
            painter.drawPoint(position[2], position[3])



        painter.end()



    def draw(self):
        print("teste")
        self.repaint()
        pass



def main():

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtGui.QApplication(sys.argv)  # A new instance of QApplication
    form = Visualizer(sys.argv[1])                 # We set the form to be our ExampleApp (design)
    form.show()                         # Show the form
    app.exec_()                         # and execute the app



if __name__ == '__main__':              # if we're running file directly and not importing it
    main()                              # run the main function