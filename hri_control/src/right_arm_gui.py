#!/usr/bin/env python

import sys
import rospy
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QMainWindow
from geometry_msgs.msg import Pose
import geometry_msgs
import os
from mainwindow import Ui_MainWindow
from constants import *
from PyQt5.QtWidgets import QDialogButtonBox, QWidget
from PyQt5.QtWidgets import QFormLayout
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtWidgets import QVBoxLayout
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class ROS(QThread):
    def __init__(self):
        rospy.init_node('gui', anonymous=True)
        self.pubpose = rospy.Publisher('/camArmPoseGoal',
                                           Pose, queue_size=1)

    def pub(self,x,y,z,r,p,yaw):
        newPose = Pose()

        arr = quaternion_from_euler(r, p, yaw)
        ox = arr[0]
        oy = arr[1]
        oz = arr[2]
        ow = arr[3]

        newPose.orientation.w = ow
        newPose.orientation.x = ox
        newPose.orientation.y = oy
        newPose.orientation.z = oz

        ## If you want the end-effector to be prependicular to the groud
        # pose_goal.orientation.w = 0.0
        # pose_goal.orientation.x = 1.0
        # pose_goal.orientation.y = 0.0
        # pose_goal.orientation.z = 0.0

        newPose.position.x = x
        newPose.position.y = y
        newPose.position.z = z
        self.pubpose.publish(newPose)




class Login(QMainWindow):
    """Dialog."""

    def __init__(self, parent=None):
        QMainWindow.__init__(self)
        self.main_ui = Ui_MainWindow()
        self.main_ui.setupUi(self)


def publish():
    ros.pub(float(ui.xpose.text()),
            float(ui.ypose.text()),
            float(ui.zpose.text()),
            float(ui.rawpose.text()),
            float(ui.pitchpose.text()),
            float(ui.yawpose.text()))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ros = ROS()
    window = Login()
    window.setWindowTitle('Camera Arm Pose Control')
    window.setFixedSize(window.width(), window.height())
    window.main_ui.pushButton.clicked.connect(publish)
    ui = window.main_ui
    window.show()
    sys.exit(app.exec_())
