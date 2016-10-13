#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState

# Constantes de estado
INIT_LOOP = 0
MOV_FORWARD = 1
TURNING = 2

# Estado del robot
state = INIT_LOOP

# Posición inicial de cada rueda [izq, der]
iPos = [0, 0]

# Tópico donde se publican los comando para que el robot se mueva.
pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)

# Retorna True si el robot se está moviendo (con margen de error eps). False si no.
def isMoving(vel):
    eps = 0.001
    return abs(vel[0]) > eps or abs(vel[1]) > eps

# Función que se ejecutra cada vez que llega información del robot.
def callback(data):
    global iPos
    global state
    
    # Distancia que debe avanzar el robot antes de frenar.
    # Forma un lado del triángulo.
    distLin = 27
    
    # Distancia que avanzan las ruedas (en sentido contrario) para girar al robot.
    # Forma un vértice del triángulo.
    distAng = 7
    
    vLin = 0
    vAng = 0

    if state == INIT_LOOP:
        iPos = data.position
        state = MOV_FORWARD
    
    if state == MOV_FORWARD:
        if data.position[0] - iPos[0] < distLin or data.position[1] - iPos[1] < distLin:
            vLin = 0.3
        elif not isMoving(data.velocity):
            iPos = data.position 
            state = TURNING
    
    if state == TURNING:
        if data.position[0] - iPos[0] < distAng or data.position[1] - iPos[1] > 0-distAng:
            vAng = -0.3
        elif not isMoving(data.velocity):
            state = INIT_LOOP

    pub.publish(Vector3(vLin, 0, 0), Vector3(0, 0, vAng))

# Función que se subscribe al tópico joint_states, que recibe información del estado del robot
def listener():
    rospy.init_node('baile1', anonymous=True)
    rospy.Subscriber("/joint_states", JointState , callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
