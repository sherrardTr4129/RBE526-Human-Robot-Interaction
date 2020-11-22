# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(270, 14, 160, 471))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)
        self.xpose = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.xpose.setObjectName("xpose")
        self.verticalLayout.addWidget(self.xpose)
        self.label_2 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.verticalLayout.addWidget(self.label_2)
        self.ypose = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.ypose.setObjectName("ypose")
        self.verticalLayout.addWidget(self.ypose)
        self.label_3 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.verticalLayout.addWidget(self.label_3)
        self.zpose = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.zpose.setObjectName("zpose")
        self.verticalLayout.addWidget(self.zpose)
        self.label_4 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_4.setObjectName("label_4")
        self.verticalLayout.addWidget(self.label_4)
        self.rawpose = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.rawpose.setObjectName("rawpose")
        self.verticalLayout.addWidget(self.rawpose)
        self.label_5 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_5.setObjectName("label_5")
        self.verticalLayout.addWidget(self.label_5)
        self.pitchpose = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.pitchpose.setObjectName("pitchpose")
        self.verticalLayout.addWidget(self.pitchpose)
        self.label_6 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_6.setObjectName("label_6")
        self.verticalLayout.addWidget(self.label_6)
        self.yawpose = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.yawpose.setObjectName("yawpose")
        self.verticalLayout.addWidget(self.yawpose)
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(550, 210, 161, 101))
        self.pushButton.setObjectName("pushButton")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "x"))
        self.label_2.setText(_translate("MainWindow", "y"))
        self.label_3.setText(_translate("MainWindow", "z"))
        self.label_4.setText(_translate("MainWindow", "raw"))
        self.label_5.setText(_translate("MainWindow", "pitch"))
        self.label_6.setText(_translate("MainWindow", "yaw"))
        self.pushButton.setText(_translate("MainWindow", "Publish"))

