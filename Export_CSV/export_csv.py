#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose

# -------

def receiveForce(data):
    global force
    force = data

def receiveAcceleration(data):
    global accel
    accel = data

def receiveVelocity(data):
    global veloc
    veloc = data

def receivePosition(data):
    global posit
    posit = data

# -------

force = None
accel = None
veloc = None
posit = None


results = open("../Results/results.csv", "w")

string = ("force.x, force.y, force.z, accel.x, accel.y, accel.z, " +
    "posit.pos.x, posit.pos.y, posit.pos.z, " +
    "posti.ori.x, posit.ori.y, posit.ori.z, posti.ori.w\n")
results.write(string)


rospy.init_node("exportCSV")

while not rospy.is_shutdown():
    rospy.Subscriber("/gazebo_talker/forca", Vector3, receiveForce)
    rospy.Subscriber("/gazebo_talker/aceleracao", Vector3, receiveAcceleration)
    rospy.Subscriber("/gazebo_talker/velocidade", Vector3, receiveVelocity)
    rospy.Subscriber("/gazebo_talker/posicao", Pose, receivePosition)

    rospy.sleep(1)

    if (force != None):
        string = (str(force.x) + ", " + str(force.y) + ", " + str(force.z) + ", "
            + str(accel.x) + ", " + str(accel.y) + ", " + str(accel.z) + ", "
            + str(veloc.x) + ", " + str(veloc.y) + ", " + str(veloc.z) + ", "
            + str(posit.position.x) + ", " + str(posit.position.y) + ", " + str(posit.position.z) + ", "
            + str(posit.orientation.x) + ", " + str(posit.orientation.y) + ", "
            + str(posit.orientation.z) + ", "+ str(posit.orientation.w) + "\n")
        results.write(string)

results.close()
