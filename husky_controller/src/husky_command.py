#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move_in_circle():
    rospy.init_node('circle_mover', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    
    vel_msg.linear.x = 1.0  # Linear velocity
    vel_msg.angular.z = 1.0  # Angular velocity
    
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        velocity_publisher.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_in_circle()
    except rospy.ROSInterruptException:
        pass
