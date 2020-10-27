#!/usr/bin/env python

# Author: Trevor Sherrard
# Since: 10/27/2020
# Project: Autonomous Camera Assitance for Teleoperation

# import required libraries
import rospy
import time
import threading
from flask import Flask, request, jsonify
from flask_cors import CORS
from sensor_msgs.msg import Joy

app = Flask(__name__)
CORS(app)

global axisList
axisList = [0, 0, 0, 0]

@app.route('/xyJoyPost', methods=['POST']) 
def xyJoyDataCB():
    global axisList
    xyDict = request.get_json()
    axisList[0] = int(xyDict['x'])
    axisList[1] = int(xyDict['y'])
    return "OK"

@app.route('/gzJoyPost', methods=['POST'])
def gzJoyDataCB():
    global axisList
    gzDict = request.get_json()
    axisList[2] = int(gzDict['z'])
    axisList[3] = int(gzDict['g'])
    return "OK"

def publishLoopThread(joyPub):
    global axisList

    while not rospy.is_shutdown():
        JoyMessage = Joy()
        JoyMessage.axes = axisList
        joyPub.publish(JoyMessage)

def main():
    global axisList

    # init node
    rospy.init_node('flask_to_joy')
    joyPub = rospy.Publisher('virtualJoystick', Joy, queue_size=10)
    rospy.loginfo("node initalized...")

    threading.Thread(target=lambda: app.run(host="0.0.0.0", port=5000)).start()

    thread = threading.Thread(target=publishLoopThread, args=(joyPub,))
    thread.start()

if __name__ == "__main__":
    main()
