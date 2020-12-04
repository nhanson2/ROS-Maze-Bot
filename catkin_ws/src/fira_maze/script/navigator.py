#!/usr/bin/env python3


import typing
import math
import tkinter as tk
from tkinter import simpledialog
import rospy, time, math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
'''
Purpose:

1) Receive goal topic and set 2-D nav goal to that destination
2) Record positions along the way to the destination
3) Stop at destination
4) Play back destinations along the way as a series of 2-D nav goals

'''

class Navigator:
    def __init__(self) -> None:
        self.goal = None
        self.waypoints = []
        self.state = 'waiting'
        '''
        waiting
        nav_goal
        at_goal
        reversing
        '''
        # Create the node
        rospy.init_node('navigator')
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1) # move the robot to goal
        self.pos_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.update_pose)            # read the imu
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.get_status)            # read the imu
        self._last_heard = rospy.Time.now()
        self.rate = rospy.Rate(10)
        self.status = 'waiting'

    def update_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if self.status == 'nav_goal' and (rospy.Time.now() - self._last_heard).to_sec() > 1.0:
            self._last_heard = rospy.Time.now()
            rospy.loginfo('got him')
            # Accept and append goal positions
            print(msg.pose.pose)
            location = msg.pose.pose
            self.waypoints.append(location)

    def get_goal(self) -> None:
        valid = False
        while not valid:
            ROOT = tk.Tk()
            ROOT.withdraw()
            # the input dialog
            USER_INP = simpledialog.askstring(title="Enter goal",
                                            prompt="Enter two floats in range (0-2) seperated by a space")
            x, y = [float(z) for z in USER_INP.split()]
            if 0 < x < 2 and 0 < y < 2:
                rospy.loginfo(f'Accepted -> x: {x}, y {y}')
                # Set new goal
                self.goal = (x,y)
                # Clear intermediary waypoints
                self.waypoints = []
                valid = True
    
    def set_nav_goal(self) -> None:
        toSend = PoseStamped()
        toSend.header.frame_id = "map"
        toSend.header.stamp = rospy.Time.now()
        toSend.pose.position.x = self.goal[0]
        toSend.pose.position.y = self.goal[1]
        toSend.pose.orientation.w = 1.0
        self.goal_pub.publish(toSend)

    def set_nav_goal_reverse(self) -> None:
        if len(self.waypoints) == 0 and self.status == 'reversing':
            rospy.loginfo('Completed traversing through previous waypoints')
            self.status = 'waiting'
            return
        
        newGoal = self.waypoints.pop()
        rospy.loginfo('Setting new intermediary goal!!!')
        quat = (newGoal.orientation.x, newGoal.orientation.y, newGoal.orientation.z, newGoal.orientation.w)
        euler = euler_from_quaternion(quat)
        # Rotate quaternion by 180 degress
        x, y, z = euler
        z += math.pi/2
        euler = (x,y,z)
        quat2= quaternion_from_euler(*euler)
        toSend = PoseStamped()
        #toSend.pose.orientation.x = quat2[0]
        #toSend.pose.orientation.y = quat2[1]
        #toSend.pose.orientation.z = quat2[2]
        toSend.pose.orientation.w = quat2[3]
        toSend.pose.position.x = newGoal.position.x
        toSend.pose.position.y = newGoal.position.y
        toSend.pose.position.z = newGoal.position.z
        toSend.header.frame_id = "map"
        toSend.header.stamp = rospy.Time.now()
        rospy.loginfo(toSend)
        # Send new goal
        self.goal_pub.publish(toSend)
        rospy.sleep(1)


    def get_status(self, msg: GoalStatusArray) -> None:
        if self.status == 'nav_goal' and msg.status_list[-1].status == 3:
            print(msg)
            rospy.loginfo('At goal')
            self.status = 'at_goal'
            print(self.waypoints)
        elif self.status == 'reversing' and (rospy.Time.now() - self._last_heard).to_sec() > 1.0 and msg.status_list[-1].status == 3:
            self._last_heard = rospy.Time.now()
            rospy.loginfo('At intermediary goal')
            self.set_nav_goal_reverse()


    def run(self):
        while not rospy.is_shutdown():
            if self.status == 'waiting':
                print('should be here')
                # Ask user for new goal if we don't have one
                self.get_goal()
                # Set new navigation goal
                self.set_nav_goal()
                rospy.sleep(1)
                self.status = 'nav_goal'

            elif self.status == 'nav_goal':
                # print distance to goal
                pass

            elif self.status == 'at_goal':
                # Sleep for 5 seconds
                rospy.loginfo('sleeping....')
                rospy.sleep(5)
                rospy.loginfo('resuming....')
                self.status = 'reversing'

            elif self.status == 'reversing':
                # Reverse positions through goals
                pass

            self.rate.sleep()

if __name__ == '__main__':
    engine = Navigator()
    engine.run()