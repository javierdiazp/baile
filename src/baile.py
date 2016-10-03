#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState

state = 0
iPos = [0, 0]

pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)

def isMoving(vel):
    eps = 0.001
    return abs(vel[0]) > eps or abs(vel[1]) > eps

def callback(data):
    global iPos
    global state
    distLin = 27
    distAng = 7
    vLin = 0
    vAng = 0
    # print "state="+str(state)+" iPos="+str(iPos)
    # print data.position
    # print data.velocity
    if state == 0:
        iPos = data.position
        state = 1
    if state == 1:
        if data.position[0] - iPos[0] < distLin or data.position[1] - iPos[1] < distLin:
            vLin = 0.3
        elif not isMoving(data.velocity):
            iPos = data.position 
            state = 2
    if state == 2:
        if data.position[0] - iPos[0] < distAng or data.position[1] - iPos[1] > 0-distAng:
            vAng = -0.3
        elif not isMoving(data.velocity):
            state = 0

    pub.publish(Vector3(vLin, 0, 0), Vector3(0, 0, vAng))

def listener():
    rospy.init_node('baile1', anonymous=True)
    rospy.Subscriber("/joint_states", JointState , callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
