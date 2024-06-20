'''
@Author: Chucheng Wang
@Date  : 2021/10/02
'''

import math
import subprocess

import numpy as np

import rospy
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

FREQUENCY = 60 #Hz.
LINEAR_VELOCITY = 0.3 # m/s
ANGULAR_VELOCITY = math.pi/4 # rad/s

class SearchSolution:
    def __init__(self, problem, search_method):
        self.problem_name = str(problem)
        self.search_method = search_method
        self.path = []
        self.nodes_visited = 0
        self.cost = 0

        rospy.init_node('SearchSolution')

        self.poses = {}
        self.odom_subs = []
        odoms = [x.strip() for x in subprocess.Popen("rostopic list | grep odom", shell=True, stdout=subprocess.PIPE).stdout.read().split('\n') if x != '']
        for odom in odoms:
            self.odom_subs.append(rospy.Subscriber(odom, Odometry, self.odom_callback, queue_size=1))
            print(odom)

        self.cmd_pubs = {}
        cmd_pubs = [x.strip() for x in subprocess.Popen("rostopic list | grep cmd_vel", shell=True, stdout=subprocess.PIPE).stdout.read().split('\n') if x != '']
        for cmd_pub in cmd_pubs:
            robot_name = cmd_pub.split('/')[1]
            self.cmd_pubs[robot_name] = rospy.Publisher(cmd_pub, Twist, queue_size=1)

        # Other variables.
        self.linear_velocity = LINEAR_VELOCITY # Constant linear velocity set.
        self.angular_velocity = ANGULAR_VELOCITY # Constant angular velocity set.
        self._close_obstacle = False

    def move(self, robot_name):
        while len(self.poses) < 1:
            rospy.sleep(1)

        i=0
        while i < (len(self.path)-2):
            if self.path[i][1] == self.path[i+1][1] == self.path[i+2][1] or self.path[i][2] == self.path[i+1][2] == self.path[i+2][2]:
                self.path.remove(self.path[i+1])
            else:
                i+=1

        try:
            for i in range(len(self.path)-1):
                curr_odom = self.poses.get(robot_name)
                
                curr_x, curr_y = self.index_to_loc(self.path[i][1], self.path[i][2])
                next_x, next_y = self.index_to_loc(self.path[i + 1][1], self.path[i + 1][2])
                self.move_to_point(robot_name, next_x, next_y)
            

        except rospy.ROSInterruptException:
            pass

    def move_to_point(self, robot_name, loc_x, loc_y):
        # Rate at which to operate the while loop.
        rate = rospy.Rate(FREQUENCY)
        twist_msg = Twist()
        while not rospy.is_shutdown():
            inc_x = loc_x - self.poses.get(robot_name).x
            inc_y = loc_y - self.poses.get(robot_name).y

            if math.sqrt(inc_x ** 2 + inc_y ** 2) < 0.015:
                break

            angle_to_goal = math.atan2(inc_y, inc_x)
            theta = self.poses.get(robot_name).theta
            a = theta if theta > 0 else theta + 2 * math.pi
            b = angle_to_goal if angle_to_goal > 0 else angle_to_goal + 2 * math.pi
            angle = (b - a) % (2 * math.pi)
            kp = 1.6
            angular_velocity = min(kp * abs(angle), self.angular_velocity)

            if abs(angle) > 0.1:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = angular_velocity if angle < math.pi else -angular_velocity
                # self.rotate_in_place(robot_name, angle_to_goal)
            else:
                twist_msg.linear.x = 0.2
                twist_msg.angular.z = 0.0

            self.cmd_pubs.get(robot_name).publish(twist_msg)
            rate.sleep()

        # Traveled the required distance, stop.
        self.stop(robot_name)

    def stop(self, robot_name):
        """Stop the robot."""
        twist_msg = Twist()
        self.cmd_pubs.get(robot_name).publish(twist_msg)

    def odom_callback(self, msg):
        robot_name = msg._connection_header["topic"].split('/')[1]
        pose = Pose2D()
        pose.x = msg.pose.pose.position.x
        pose.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        pose.theta = yaw
        self.poses[robot_name] = pose

    def loc_to_index(self, x, y):
        return int(x * 10 + 100), int(100 - y * 10)

    def index_to_loc(self, x, y):
        return float(x - 100) / 10, float(100 - y) / 10

    def __str__(self):
        string = "----\n"
        string += "{:s}\n"
        string += "attempted with search method {:s}\n"

        if len(self.path) > 0:

            string += "number of nodes visited: {:d}\n"
            string += "solution length: {:d}\n"
            string += "cost: {:d}\n"
            string += "path: {:s}\n"

            string = string.format(self.problem_name, self.search_method,
                self.nodes_visited, len(self.path), self.cost, str(self.path))
        else:
            string += "no solution found after visiting {:d} nodes\n"
            string = string.format(self.problem_name, self.search_method, self.nodes_visited)

        return string
