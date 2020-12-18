#!/usr/bin/env python

import math
#import tkinter as tk
#from tkinter import simpledialog
import rospy, time, math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import rospy
from fira_maze.msg import goal
'''
Purpose:

1) Receive goal topic and set 2-D nav goal to that destination
2) Record positions along the way to the destination
3) Stop at destination
4) Play back destinations along the way as a series of 2-D nav goals

'''

class Navigator:
    def __init__(self):
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
        self.goal_sub = rospy.Subscriber('/goal', goal, self.set_global_goal)            # read the imu
        self._last_heard = rospy.Time.now()
        self.rate = rospy.Rate(10)
        self.status = 'waiting'

    def set_global_goal(self, msg):
        rospy.loginfo('Received new goal on /goal topic')
        # Set new goal
        print(msg)
        x,y = msg.x, msg.y
        self.goal = (int(x),int(y))
        # Clear intermediary waypoints
        self.waypoints = []
        self.status = 'nav_goal'
        valid = True
        self.set_nav_goal()
        rospy.sleep(1)

    def distance(self, currMsg, prevMsg):
        if not prevMsg:
            # Return a large number since this is our first waypoint to track
            return 10000
        # Calculate distance
        return math.sqrt((currMsg.position.x - prevMsg.position.x)**2 + (currMsg.position.y - prevMsg.position.y)**2)


    def update_pose(self, msg):
        if self.status == 'nav_goal' and (rospy.Time.now() - self._last_heard).to_sec() > 2.0:
            # Check distance from previous position to keep us from appending multiple redundant waypoints
            if not self.waypoints or self.distance(msg.pose.pose, self.waypoints[-1]) > 0.1:
                self._last_heard = rospy.Time.now()
                rospy.loginfo('Accepted intermediary goal')
                # Accept and append goal positions
                location = msg.pose.pose
                self.waypoints.append(location)
            else:
                rospy.logwarn('Distance is too small to append')

    def get_goal(self):
        valid = False
        while not valid:
            ROOT = tk.Tk()
            ROOT.withdraw()
            # the input dialog
            USER_INP = simpledialog.askstring(title="Enter goal",
                                            prompt="Enter two floats in range (-2, 2) seperated by a space")
            x, y = [float(z) for z in USER_INP.split()]
            if -2 <= x <= 2 and -2 <= y <= 2:
                rospy.loginfo('Accepted -> x: {}, y {}'.format(x,y))
                # Set new goal
                self.goal = (x,y)
                # Clear intermediary waypoints
                self.waypoints = []
                valid = True
    
    def set_nav_goal(self):
        toSend = PoseStamped()
        toSend.header.frame_id = "map"
        toSend.header.stamp = rospy.Time.now()
        toSend.pose.position.x = self.goal[0]
        toSend.pose.position.y = self.goal[1]
        toSend.pose.orientation.w = 1.0
        self.goal_pub.publish(toSend)

    def set_nav_goal_reverse(self):
        if len(self.waypoints) == 0 and self.status == 'reversing':
            rospy.loginfo('Completed traversing through previous waypoints')
            self.status = 'waiting'
            return
        
        newGoal = self.waypoints.pop()
        rospy.loginfo('Setting new intermediary goal!')
        quat = (newGoal.orientation.x, newGoal.orientation.y, newGoal.orientation.z, newGoal.orientation.w)
        euler = euler_from_quaternion(quat)
        # Rotate quaternion by 180 degress
        x, y, z = euler
        z += math.pi/2
        euler = (x,y,z)
        quat2= quaternion_from_euler(*euler)
        toSend = PoseStamped()
        toSend.pose.orientation.x = quat2[0]
        toSend.pose.orientation.y = quat2[1]
        toSend.pose.orientation.z = quat2[2]
        toSend.pose.orientation.w = quat2[3]
        toSend.pose.position.x = newGoal.position.x
        toSend.pose.position.y = newGoal.position.y
        toSend.pose.position.z = newGoal.position.z
        toSend.header.frame_id = "map"
        toSend.header.stamp = rospy.Time.now()
        # rospy.loginfo(toSend)
        # Send new goal
        self.goal_pub.publish(toSend)
        rospy.sleep(1)


    def get_status(self, msg):
        if self.status == 'nav_goal' and msg.status_list[-1].status == 3:
            # Check the distance to this goal
            rospy.loginfo('At goal')
            self.status = 'at_goal'
        elif self.status == 'reversing' and (rospy.Time.now() - self._last_heard).to_sec() > 1.0 and msg.status_list[-1].status == 3:
            self._last_heard = rospy.Time.now()
            rospy.loginfo('At intermediary goal')
            self.set_nav_goal_reverse()


    def run(self):
        while not rospy.is_shutdown():
            if self.status == 'waiting':
                pass
                # Ask user for new goal if we don't have one
                #self.get_goal()
                # Set new navigation goal
                #self.set_nav_goal()
                #rospy.sleep(1)
                #self.status = 'nav_goal'

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