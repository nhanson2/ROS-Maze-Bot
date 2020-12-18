import rospy, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

def scan_callback(msg):
    global range_front
    global range_right
    global range_left
    global ranges
    global min_front, i_front, min_right, i_right, min_left, i_left
    global scan_received
    
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

    scan_received = True

def imu_callback(msg):
    global yaw

    quat = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    euler = euler_from_quaternion(quat)

    yaw = euler[2]

def odom_callback(msg):
    global map_position 

    map_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

def comm_callback(msg):
    global target_pos
    global target_set

    if(msg[0] == 'S'):
        target_pos = (float(msg[1:6]), float(msg[6:]))
        target_set = True

def get_ang_err():
    global des_ang

    ang_err = yaw - des_ang

    if(ang_err > math.pi):
        ang_err = ang_err - (2*math.pi)

    elif(ang_err < -math.pi):
        ang_err = ang_err + (2*math.pi)

    return ang_err


range_front = []
range_right = []
range_left  = []
ranges = []
min_front = 0
i_front = 0
min_right = 0
i_right = 0
min_left = 0
i_left = 0
scan_received = False

scan_init = []
scan_init_sum = 0
scan_set = False

yaw = 0.0
des_ang = math.pi/2
kp = 1

map_position = []
err_tolerance = 0.1

target_pos = []
target_set = False

save_position = (0.0, 0.0)
travel_dist = 0.0
turn_dist = 0.0

front_dist_min = 0.16

drive_vel = 0.05

state = 0

# Create the node
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1) # move the robot
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)   # read the laser scanner
imu_sub = rospy.Subscriber('imu', Imu, imu_callback)            # read the imu
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)    # read odometry information
comm_pub = rospy.Publisher('comm', String, queue_size=1)        # communicate to robot M
comm_sub = rospy.Subscriber('comm', String, comm_callback)       # communicate to robot M

rospy.init_node('maze_explorer',)

command = Twist()
command.linear.x = 0.0
command.angular.z = 0.0

rate = rospy.Rate(10)
time.sleep(15) # wait for node to initialize


while not rospy.is_shutdown():

    if(state == 0):     # Waiting for robot M

        command.angular.z = 0.0
        command.linear.x = 0.0

        if(scan_received and not scan_set):
            scan_init = ranges
            scan_init_sum = math.fsum(scan_init)
            scan_set = True

        scan_sum = math.fsum(ranges)

        if(abs(scan_init_sum - scan_sum) > 1):
            comm_pub.publish("Mhello")
            state = 1
    
    elif(state == 1 and target_set):

        dist = math.dist(target_pos, map_position)

        if(dist > err_tolerance):
            x_dist = map_position[0] - target_pos[0]
            y_dist = map_position[1] - target_pos[1]

            des_ang = math.atan2(y_dist,x_dist)

            ang_err = get_ang_err()

            command.angular.z = -ang_err*kp

            if(ang_err < 0.5)
                command.linear.x = drive_vel
            else:
                command.linear.x = 0.0

        else:
            command.angular.z = 0.0
            command.linear.x = 0.0
            comm_pub.publish("Mhello")
            target_set = False

    else:
        command.angular.z = 0.0
        command.linear.x = 0.0


    cmd_vel_pub.publish(command)
    rate.sleep()



        
































