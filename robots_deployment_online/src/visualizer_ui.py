# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'visualizer.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_VisualizerWindow(object):
    def setupUi(self, VisualizerWindow):
        VisualizerWindow.setObjectName(_fromUtf8("VisualizerWindow"))
        VisualizerWindow.resize(1024, 768)
        self.centralwidget = QtGui.QWidget(VisualizerWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.mainwidget = QtGui.QWidget(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.mainwidget.sizePolicy().hasHeightForWidth())
        self.mainwidget.setSizePolicy(sizePolicy)
        self.mainwidget.setMinimumSize(QtCore.QSize(100, 100))
        self.mainwidget.setObjectName(_fromUtf8("mainwidget"))
        self.horizontalLayout.addWidget(self.mainwidget)
        VisualizerWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(VisualizerWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        VisualizerWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(VisualizerWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        VisualizerWindow.setStatusBar(self.statusbar)

        self.retranslateUi(VisualizerWindow)
        QtCore.QMetaObject.connectSlotsByName(VisualizerWindow)

    def retranslateUi(self, VisualizerWindow):
        VisualizerWindow.setWindowTitle(_translate("VisualizerWindow", "Visualizer", None))

