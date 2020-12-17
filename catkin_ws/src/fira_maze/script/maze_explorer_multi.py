#!/usr/bin/env python

from __future__ import nested_scopes
from __future__ import generators

# 5 / 2 == 2.5 instead of 2
from __future__ import division
from __future__ import absolute_import
from __future__ import with_statement

# print('hello') instead of print 'hello'
from __future__ import print_function

# because '{}' is now unicode
from __future__ import unicode_literals
import os
import rospy, time, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

map_position = ()

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
    # to the right (between 255 to 295 degrees)
    range_right = msg.ranges[255:295]
    # to the left (between 75 to 95 degrees)
    range_left = msg.ranges[95:75:-1]
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

def odom_callback(msg):
    global map_position 

    map_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)


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
        command.linear.x = lin_spd*command.angular.z
        cmd_vel_pub.publish(command)

        rate.sleep()

    else:
        command.angular.z = 0.0
        command.linear.x = 0.0
        cmd_vel_pub.publish(command)

def turn_right():
    global des_ang

    des_ang = des_ang - (math.pi/2)

    if(des_ang < -math.pi):
        des_ang = des_ang + (2*math.pi)
    
    go_to_ang((0.0))

def turn_left():
    global des_ang

    des_ang = des_ang + (math.pi/2)

    if(des_ang > math.pi):
        des_ang = des_ang - (2*math.pi)
    
    go_to_ang(0.0)

    

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
des_ang = math.pi/2
kp = 1

map_position_init = ()
odom_init = True
init_map_time = 0.0
err_tolerance = 0.1

save_position = (0.0, 0.0)
travel_dist = 0.0
turn_dist = 0.06

front_dist_min = 0.15

drive_vel = 0.05

# Create the node
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1) # move the robot
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)   # read the laser scanner
imu_sub = rospy.Subscriber('imu', Imu, imu_callback)            # read the imu
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)    # read odometry information

rospy.init_node('maze_explorer',)

command = Twist()
command.linear.x = 0.0
command.angular.z = 0.0

rate = rospy.Rate(10)
time.sleep(10) # wait for node to initialize

state = 0

# State
# 0: no wall found
# 1: wall on left
# 2: opening turn left
# 3: finish turn
# 4: corner turn right

while not rospy.is_shutdown():

    if(odom_init == True):
        map_position_init = map_position
        odom_init = False
        # Grab time
        init_map_time = rospy.get_time()
    
    # THIS IS THE CODE THAT STOPS THE MAPPING PROCESS
    if ((rospy.get_time() > (init_map_time + 60.0)) and (odom_init == False)):
        #check current position against saved initial map position
        position_err = (abs(map_position_init[0]-map_position[0]), abs(map_position_init[1]-map_position[1]))
        #if error is low, mapping is done. Break the loop and save the map.
        if ((position_err[0] < err_tolerance) and (position_err[1] < err_tolerance)):
            print("all done, save the map now")
            break

    rospy.loginfo_throttle(2,'Range left: {}'.format(range_left))
    
    # State 0: Looking for wall

    if(state == 0):
        if(min_front > front_dist_min and min_right > 0.2 and min_left > 0.2):    
            print("No Wall Found")
            command.angular.z = 0.0    # if nothing near, go forward
            command.linear.x = 0.05
        elif(min_left < 0.2):           # if wall on left, start tracking
            print("Left Wall Found")
            state = 1        
            command.angular.z = 0.0
            command.linear.x = 0.0       
        else:
            print("Wall Found")
            turn_right()
            command.angular.z = 0.0
            command.linear.x = 0.0
    

    # State 1: Following Wall

    elif(state == 1):
        print("Range: {:.2f}m - Following Wall".format(min_left))

        if (min_front < front_dist_min):
            state = 4
            command.angular.z = 0.0
            command.linear.x = 0.0
        
        elif (min_left > 0.2):
            state = 2
            save_position = map_position
            command.angular.z = 0.0
            command.linear.x = 0.0

        else:
            ang_err = get_ang_err()
            command.angular.z = -ang_err*kp + (min_left-0.1)*2
            command.linear.x = drive_vel


    # State 2: Opening on Left

    elif(state == 2):
        print("Opening on left")

        travel_dist = math.sqrt(sum([(a - b) ** 2 for a, b in zip(map_position, save_position)]))

        if(min_left < 0.2):
            state = 1
            ang_err = get_ang_err()
            command.angular.z = -ang_err*kp
            command.linear.x = drive_vel*0.5

        elif(travel_dist < turn_dist and min_front > front_dist_min):
            ang_err = get_ang_err()
            command.angular.z = -ang_err*kp
            command.linear.x = drive_vel

        elif(travel_dist < turn_dist):
            state = 4
            command.angular.z = 0.0
            command.linear.x = 0.0

        else:
            turn_left()
            state = 3
            save_position = map_position
            command.angular.z = 0.0
            command.linear.x = 0.0

        
    # State 3: Finish Turn

    elif(state == 3):
        print("Range: {:.2f}m - Following Wall".format(min_left))

        travel_dist = math.sqrt(sum([(a - b) ** 2 for a, b in zip(map_position, save_position)]))

        if(travel_dist < 0.1 and min_front > front_dist_min):
            ang_err = get_ang_err()
            command.angular.z = -ang_err*kp
            command.linear.x = drive_vel

        elif(travel_dist < 0.1):
            state = 4
            command.angular.z = 0.0
            command.linear.x = 0.0

        else:
            state = 1
            ang_err = get_ang_err()
            command.angular.z = -ang_err*kp
            command.linear.x = drive_vel
    

    # State 4: Front Obstacle, Turn Right

    elif(state == 4):
        print("Front obstacle detected")
        if(min_front < 0.13):
            command.angular.z = 0.0
            command.linear.x = -0.03

        else:
            turn_right()
            state = 1
            command.angular.z = 0.0
            command.linear.x = 0.0


    # publish command 
    cmd_vel_pub.publish(command)
    # wait for the loop
    rate.sleep()

### INSERT CODE HERE FOR SAVING THE MAP ###
rospy.loginfo('Saving map!')
os.system('rosrun map_server map_saver -f $(rospack find fira_maze)/maps/part_2_map')