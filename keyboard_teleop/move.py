#!/usr/bin/env python
import rospy
import time
import math
from geometry_msgs.msg import Twist

def move():
    # Starts a new node
    rospy.init_node('move_circle', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    msg = Twist()
    
    #Radius and Ang vel
    R = 1
    omega = 6.28/60
    
    start = time.time()
    
    while not rospy.is_shutdown():

        time_now = time.time()
        duration = time_now - start

        msg.linear.x = -R*omega*math.sin(omega*duration)
        msg.linear.y = R*omega*math.cos(omega*duration)
        #Force the robot to stop
        velocity_publisher.publish(msg)

    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0

    velocity_publisher.publish(msg)


if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
