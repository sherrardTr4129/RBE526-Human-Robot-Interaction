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
from std_msgs.msg import Bool

app = Flask(__name__)
CORS(app)

global axisList
global gripperValList
global camObjList
axisList = [0, 0, 0]
gripperValList = [0]
camObjList = [False, False, True]

@app.route('/xyJoyPost', methods=['POST']) 
def xyJoyDataCB():
    global axisList
    xyDict = request.get_json()
    axisList[0] = int(xyDict['x'])
    axisList[1] = int(xyDict['y'])
    return "OK"

@app.route('/zJoyPost', methods=['POST'])
def gzJoyDataCB():
    global axisList
    gzDict = request.get_json()
    axisList[2] = int(gzDict['z'])
    return "OK"

@app.route('/closeGripper', methods=['POST'])
def closeGripperCB():
    global gripperValList 
    valDict = request.get_json()
    gripperValList[0] -= int(valDict['val'])

    # constrain gripperVal to min out at 0
    if(gripperValList[0] < 0):
        gripperValList[0] = 0
    return "OK"

@app.route('/openGripper', methods=['POST'])
def openGripperCB():
    global gripperValList
    valDict = request.get_json()
    gripperValList[0] += int(valDict['val'])

    # constrain gripperVal to max out at 10
    if(gripperValList[0] > 10):
        gripperValList[0] = 10
    return "OK"

@app.route('/setCamObjectives', methods=['POST'])
def setCamObjectivesCB():
    global camObjList
    camObjList[0] = request.form.get('camObj1') != None
    camObjList[1] = request.form.get('camObj2') != None
    camObjList[2] = request.form.get('camObj3') != None
    return "Camera optimization objectives set. Please use the back button to resume control."

def publishLoopThread(joyPub, camObjPubList):
    # setup globals
    global axisList
    global gripperValList
    global camObjList

    # initialize control variables
    zeroList = [0,0,0]
    zeroCount = 0
    lastGripperVal = 0

    while not rospy.is_shutdown():
        # always publish axis or button values change
        if(axisList != zeroList or gripperValList[0] != lastGripperVal):
            JoyMessage = Joy()
            JoyMessage.axes = axisList
            JoyMessage.buttons = gripperValList
            joyPub.publish(JoyMessage)

            # update zeroCount
            zeroCount = 0

        # only publish three zero-valued axis values to rate limit
        # requests
        elif(axisList == zeroList and zeroCount < 3):
            JoyMessage = Joy()
            JoyMessage.axes = axisList
            JoyMessage.buttons = gripperValList
            joyPub.publish(JoyMessage)

            #update zeroCount
            zeroCount += 1

        # update lastGripperVal
        lastGripperVal = gripperValList[0]

        # publish current camera control objectives
        camObjPubList[0].publish(camObjList[0])
        camObjPubList[1].publish(camObjList[1])
        camObjPubList[2].publish(camObjList[2])

        # update delay of 200ms to match refresh rate of 200ms
        # in joystick data recieved from webpage
        time.sleep(0.2)

def main():
    # init node
    rospy.init_node('flask_to_joy')
    joyPub = rospy.Publisher('virtualJoystick', Joy, queue_size=1)

    camObjPubList = []
    camObjPubList.append(rospy.Publisher('isOcclusionAvoidance', Bool, queue_size=1))
    camObjPubList.append(rospy.Publisher('isSaliencyMaximization', Bool, queue_size=1))
    camObjPubList.append(rospy.Publisher('isInfoEntropyMaximization', Bool, queue_size=1))

    rospy.loginfo("node initalized...")

    # start flask app as a thread
    threading.Thread(target=lambda: app.run(host="0.0.0.0", port=5000)).start()

    # start publishing thread
    thread = threading.Thread(target=publishLoopThread, args=(joyPub, camObjPubList,))
    thread.start()

if __name__ == "__main__":
    main()
