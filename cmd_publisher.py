#!/usr/bin/env python
import rospy
import math
import time
from threading import Thread
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
from mavros_msgs.msg import PositionTarget, State
from geometry_msgs.msg import PoseStamped
import tf


class CMdVelPublisher:
    def __init__(self):
        self.pose_pub = rospy.Publisher('/vehicle/desPose', PoseStamped, queue_size=10)
        self.rate = rospy.Rate(10)

    def run(self):
        start_time = time.time()
        circle_radius = 2
        counter = 0
        counter_step = 0.01
        while not rospy.is_shutdown():
            #elapsed_time = time.time() - start_time
            set_point = PoseStamped()
            #twist.linear.x = 1.0
            set_point.pose.position.x = math.cos(counter)  * circle_radius
            set_point.pose.position.y = math.sin(counter)  * circle_radius
            set_point.pose.position.z = 3
            self.pose_pub.publish(set_point)
            self.rate.sleep()
            counter += counter_step

if __name__ == '__main__':
    rospy.init_node('sunflower_controller_node', anonymous=True)
    cmd_vel_publisher = CMdVelPublisher()
    cmd_vel_publisher.run()
