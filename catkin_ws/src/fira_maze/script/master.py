#!/usr/bin/env python3

import rospy, time, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from fira_maze.msg import hello

def scan_callback(msg):
    global range_front
    global range_right
    global range_left
    global ranges
    global min_front, i_front, min_right, i_right, min_left, i_left
    
    ranges = msg.ranges
    # get the range of a few points
    # in front of the robot (between 5 to -5 degrees)
    range_front[:5] = msg.ranges[5:0:-1]  
    range_front[5:] = msg.ranges[-1:-5:-1]
    # to the right (between 300 to 345 degrees)
    range_right = msg.ranges[300:345]
    # to the left (between 15 to 60 degrees)
    range_left = msg.ranges[60:15:-1]
    # get the minimum values of each range 
    # minimum value means the shortest obstacle from the robot
    min_range,i_range = min( (ranges[i_range],i_range) for i_range in range(len(ranges)) )
    min_front,i_front = min( (range_front[i_front],i_front) for i_front in range(len(range_front)) )
    min_right,i_right = min( (range_right[i_right],i_right) for i_right in range(len(range_right)) )
    min_left ,i_left  = min( (range_left [i_left ],i_left ) for i_left  in range(len(range_left )) )

def imu_callback(msg):
    global yaw

    quat = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    euler = euler_from_quaternion(quat)

    yaw = euler[2]

def get_ang_err():
    global des_ang

    ang_err = yaw - des_ang

    if(ang_err > math.pi):
        ang_err = ang_err - (2*math.pi)

    elif(ang_err < -math.pi):
        ang_err = ang_err + (2*math.pi)

    return ang_err

def go_to_ang(lin_spd):
    global kp
    global command
    global des_ang

    ang_err = get_ang_err()

    while (abs(ang_err) > 0.03 and not rospy.is_shutdown()):
        ang_err = get_ang_err()

        command.angular.z = -ang_err*kp
        command.linear.x = lin_spd
        cmd_vel_pub.publish(command)

        rate.sleep()

    else:
        command.angular.z = 0.0
        command.linear.x = 0.0
        cmd_vel_pub.publish(command)

def turn_right():
    global des_ang

    des_ang = des_ang - (math.pi/2)

    if(des_ang < math.pi):
        des_ang = des_ang + (2*math.pi)
    
    go_to_ang((0.0))

def turn_left():
    global des_ang

    des_ang = des_ang + (math.pi/2)

    if(des_ang > math.pi):
        des_ang = des_ang - (2*math.pi)
    
    go_to_ang(0.0)

def send_stop():
    command = Twist()
    command.linear.x = 0.0
    command.angular.z = 0.0
    cmd_vel_pub.publish(command)

def distance_update(msg):
    global distance_from_s
    if bool(msg.seen):
        # We have been seen by the other robot, update the distance between us
        distance_from_s = float(msg.distance)
    if bool(msg.stop):
        # Robot s thinks we should stop as we are now too close
        send_stop()
        rospy.loginfo('Stopped near Robot S! Path planning route out of the maze')


    

# Initialize all variables
range_front = []
range_right = []
range_left  = []
min_front = 0
i_front = 0
min_right = 0
i_right = 0
min_left = 0
i_left = 0

yaw = 0.0
des_ang = 0.0
kp = 1
distance_from_s = 10000000

# Create the node
comm_sub = rospy.Subscriber("/comm", hello, distance_update)
comm_pub = rospy.Subscriber("/comm", hello, queue_size=1)
cmd_vel_pub = rospy.Publisher('robot_M/cmd_vel', Twist, queue_size = 1) # move the robot
scan_sub = rospy.Subscriber('robot_M/scan', LaserScan, scan_callback)   # read the laser scanner
imu_sub = rospy.Subscriber('robot_M/imu', Imu, imu_callback)            # read the imu

rospy.init_node('maze_explorer',)

command = Twist()
command.linear.x = 0.0
command.angular.z = 0.0

rate = rospy.Rate(10)
time.sleep(5) # wait for node to initialize

state = 0

# State
# 0: no wall found
# 1: wall on left
# 2: opening turn left
# 3: corner turn right

state = 0

while not rospy.is_shutdown():

    while(state == 0 and not rospy.is_shutdown()):
        if(min_front > 0.2 and min_right > 0.2 and min_left > 0.25):    
            command.angular.z = 0.0    # if nothing near, go forward
            command.linear.x = 0.15
            print("Moving Toward Wall")
        elif(min_left < 0.25):           # if wall on left, start tracking
            state = 1       
            print("Left Wall Detected, Tracking")   
            command.angular.z = 0.0
            command.linear.x = 0.0       
        else:
            print("Wall Detected, Turning")
            turn_right()
            command.angular.z = 0.0
            command.linear.x = 0.0

        cmd_vel_pub.publish(command)
        
    else:   # left wall detected
        if(min_front > 0.18):
            if(min_left > 0.3):  
                print("Range: {:.2f}m - Opening on left, turn".format(min_left))
                command.angular.z = 0.0
                command.linear.x = 0.1
                cmd_vel_pub.publish(command)
                time.sleep(1.5)

                turn_left()

                command.angular.z = 0.0
                command.linear.x = 0.1
                cmd_vel_pub.publish(command)
                time.sleep(0.1)

            else:
                print("Range: {:.2f}m - Following Wall".format(min_left))
                ang_err = get_ang_err()
                command.angular.z = -ang_err*kp
                command.linear.x = 0.1
                
        else:   
            print("Front obstacle detected. Turning")
            turn_right()
            command.angular.z = 0.0
            command.linear.x = 0.0
        # publish command 
        cmd_vel_pub.publish(command)
    # wait for the loop
    rate.sleep()