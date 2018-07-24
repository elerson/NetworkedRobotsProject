# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'client.ui'
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

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1200, 800)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.gridLayout = QtGui.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.widget_image = Image(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.widget_image.sizePolicy().hasHeightForWidth())
        self.widget_image.setSizePolicy(sizePolicy)
        self.widget_image.setMinimumSize(QtCore.QSize(521, 331))
        self.widget_image.setObjectName(_fromUtf8("widget_image"))
        self.gridLayout.addWidget(self.widget_image, 0, 0, 1, 1)
        self.line = QtGui.QFrame(self.centralwidget)
        self.line.setFrameShape(QtGui.QFrame.VLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.gridLayout.addWidget(self.line, 0, 1, 1, 1)
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.lineEdit_id = QtGui.QLineEdit(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_id.sizePolicy().hasHeightForWidth())
        self.lineEdit_id.setSizePolicy(sizePolicy)
        self.lineEdit_id.setObjectName(_fromUtf8("lineEdit_id"))
        self.verticalLayout.addWidget(self.lineEdit_id)
        self.pushButton_setInitialPos = QtGui.QPushButton(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_setInitialPos.sizePolicy().hasHeightForWidth())
        self.pushButton_setInitialPos.setSizePolicy(sizePolicy)
        self.pushButton_setInitialPos.setObjectName(_fromUtf8("pushButton_setInitialPos"))
        self.verticalLayout.addWidget(self.pushButton_setInitialPos)
        self.pushButton_setGoal = QtGui.QPushButton(self.centralwidget)
        self.pushButton_setGoal.setObjectName(_fromUtf8("pushButton_setGoal"))
        self.verticalLayout.addWidget(self.pushButton_setGoal)
        self.lineEdit_command = QtGui.QLineEdit(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_command.sizePolicy().hasHeightForWidth())
        self.lineEdit_command.setSizePolicy(sizePolicy)
        self.lineEdit_command.setObjectName(_fromUtf8("lineEdit_command"))
        self.verticalLayout.addWidget(self.lineEdit_command)
        self.pushButton_execCommand = QtGui.QPushButton(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_execCommand.sizePolicy().hasHeightForWidth())
        self.pushButton_execCommand.setSizePolicy(sizePolicy)
        self.pushButton_execCommand.setObjectName(_fromUtf8("pushButton_execCommand"))
        self.verticalLayout.addWidget(self.pushButton_execCommand)
        self.pushButton_startDeployment = QtGui.QPushButton(self.centralwidget)
        self.pushButton_startDeployment.setObjectName(_fromUtf8("pushButton_startDeployment"))
        self.verticalLayout.addWidget(self.pushButton_startDeployment)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.gridLayout.addLayout(self.verticalLayout, 0, 2, 1, 1)
        self.line_2 = QtGui.QFrame(self.centralwidget)
        self.line_2.setFrameShape(QtGui.QFrame.HLine)
        self.line_2.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_2.setObjectName(_fromUtf8("line_2"))
        self.gridLayout.addWidget(self.line_2, 1, 0, 1, 3)
        self.textEdit_status = QtGui.QTextEdit(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.textEdit_status.sizePolicy().hasHeightForWidth())
        self.textEdit_status.setSizePolicy(sizePolicy)
        self.textEdit_status.setMinimumSize(QtCore.QSize(0, 120))
        self.textEdit_status.setMaximumSize(QtCore.QSize(16777215, 121))
        self.textEdit_status.setObjectName(_fromUtf8("textEdit_status"))
        self.gridLayout.addWidget(self.textEdit_status, 2, 0, 1, 3)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1200, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "ClientUI", None))
        self.lineEdit_id.setText(_translate("MainWindow", "0", None))
        self.pushButton_setInitialPos.setText(_translate("MainWindow", "Set Initial Pos", None))
        self.pushButton_setGoal.setText(_translate("MainWindow", "Set Goal", None))
        self.pushButton_execCommand.setText(_translate("MainWindow", "Exec Command ", None))
        self.pushButton_startDeployment.setText(_translate("MainWindow", "Start Deployment", None))

from image import Image

if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

