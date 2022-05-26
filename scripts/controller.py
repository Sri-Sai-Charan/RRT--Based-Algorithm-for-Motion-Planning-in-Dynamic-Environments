#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import sleep
import numpy as np
import sys
from math import degrees, atan2
from RRTStarFN import RRT
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class TurtleBot_RRT:
    def __init__(self):
        self.position = Odometry()
        self.odom_sub = rospy.Subscriber('/odom',Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.positionx = 0
        self.positiony = 0
        self.yaw = 0
        self.file = open("robot_pose.txt","w")


    def odom_callback(self, data):
        self.positionx = data.pose.pose.position.x
        self.positiony = data.pose.pose.position.y
        self.orientation_q = data.pose.pose.orientation
        self.orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (self.orientation_list)
        print(self.positionx, self.positiony, self.yaw)

        self.file.write(str(self.positionx)+", "+str(self.positiony)+"\n")


    def trace_path(self, path,start, end):
        for next_pos in path:
            start = [start[0]/100,start[1]/100]
            next_pos = [next_pos[0]/100,next_pos[1]/100]
            pose_x = self.position.pose.pose.position.x/100
            pose_y = self.position.pose.pose.position.y/100

            del_theta = atan2(next_pos[1] - pose_y - start[1], next_pos[0] - pose_x - start[0])-self.yaw
            vel_msg = Twist()
            vel_msg.angular.z = 0.1
            turn = 0
            rate = rospy.Rate(10)
            while(turn<del_theta):
                self.vel_pub.publish(vel_msg)
                turn+=0.05
                rate.sleep()
                
            dist = np.sqrt((pose_x-next_pos[0] + start[0])**2+(pose_y-next_pos[1] + start[1])**2)
            traversal = 0
            vel_msg = Twist()
            vel_msg.linear.x = 0.05
            print("dist:",dist)
            # exit()
            while(traversal<dist):
                self.vel_pub.publish(vel_msg)
                traversal+=0.05
                rate.sleep()

            print("------------------------REACHED ONE STEP TOWARDS GOAL-------------------------")
        # Stop the turtlebot
        vel_msg = Twist()
        self.vel_pub.publish(vel_msg)
        print("Reached GOAL!!!")


def main():
    rospy.init_node('RRT_starNode')
    obstacle_list = [
        (50, 50, 10),
        (400, 200, 15),
        (400, 250, 25),
        (300, 180, 20),
    ]
    viz_anim = True

    x_range = 800
    y_range = 600
    start=[265, 180]
    goal=[540, 150]
    rrt = RRT(start, goal,
              randArea=[x_range, y_range], obstacle_list=obstacle_list)
    path = rrt.Planning(animation=viz_anim)
    turtle = TurtleBot_RRT()
    turtle.trace_path(path,start, goal)

if __name__ == '__main__':
    main()
