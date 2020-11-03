#!/usr/bin/env python

import sys
import random
import numpy as np
from time import sleep
import datetime
from PyQt5 import QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import rospy

class WorkThread(QThread):
  # 初始化线程
  def __int__(self):
    super(WorkThread, self).__init__()
  #线程运行函数
  def run(self):
    while True:
      global T_value
      global P_value
      T_value = random.randint(200,225)
      P_value = random.randint(150,200)
      print(T_value, P_value)
      sleep(3)

class plotwindows(QtWidgets.QWidget):
    def __init__(self):
        super(plotwindows,self).__init__()
        layout = QFormLayout()
        self.edita3 = QLineEdit()
        self.edita4 = QLineEdit()
        self.edita5 = QLineEdit()
        layout.addRow("A", self.edita3)
        layout.addRow("B", self.edita4)
        layout.addRow("C", self.edita5)
        self.setLayout(layout)

    #     self.Mytimer()
    #
    # def Mytimer(self):
    #     timer = QTimer(self)
    #     timer.timeout.connect(self.update)
    #     timer.start(100)

    def update(self,data):
        self.edita3.setText(str(data.axes(0)))
        self.edita4.setText(str(data.axes(1)))
        self.edita5.setText(str(data.axes(2)))


class ROS:
    def __init__(self):
        rospy.init_node('gui', anonymous=True)
        rospy.Subscriber('/joy',Joy,self.update,queue_size=1,buff_size=52428800)


def mainwindows():
    app = QtWidgets.QApplication(sys.argv)
    new = plotwindows()
    new.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    workThread = WorkThread()
    ros = ROS()
    workThread.start()
    mainwindows()
    rospy.spin()