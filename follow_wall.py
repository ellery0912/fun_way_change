#! /usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
#from tf import transformations
#from datetime import datetime
import pyrealsense2 as rs
import numpy as np
import cv2
# Util imports
import random
import math
import time
import os

hz = 20                     # Cycle Frequency
loop_index = 0              # Number of sampling cycles
loop_index_outer_corner = 0 # Loop index when the outer corner is detected
loop_index_inner_corner = 0 # Loop index when the inner corner is detected
inf = 6                     # Limit to Laser sensor range in meters, all distances above this value are 
                            #      considered out of sensor range
wall_dist = 0.3             # Distance desired from the wall
max_speed = 0.4             # Maximum speed of the robot on meters/seconds
p = 15                      # Proportional constant for controller  
d = 0                       # Derivative constant for controller 
angle = 1                   # Proportional constant for angle controller (just simple P controller)
direction = -1              # 1 for wall on the left side of the robot (-1 for the right side)
e = 0                       # Diference between current wall measurements and previous one
angle_min = 0               # Angle, at which was measured the shortest distance between the robot and a wall
dist_front = 0              # Measured front distance
diff_e = 0                  # Difference between current error and previous one
dist_min = 0                # Minimum measured distance

inf_=0.5

# Time when the last outer corner; direction and inner corner were detected or changed.
last_outer_corner_detection_time = time.time()
last_change_direction_time = time.time()
last_inner_corner_detection_time = time.time()
rotating = 0 
pub_ = None
# Sensor regions
regions_ = {
        'bright': 0,
        'right': 0,
        'fright': 0,
        'front': 0,
        'left': 0,
}
last_kinds_of_wall=[0, 0, 0, 0, 0]
index = 0

state_outer_inner=[0, 0, 0, 0]
index_state_outer_inner = 0

bool_outer_corner = 0
bool_inner_corner =0

last_vel = [random.uniform(0.1,0.3),  random.uniform(-0.3,0.3)]
wall_found =0

#Robot state machines
state_ = 0
state_dict_ = {
    0: 'random wandering',
    1: 'following wall',
    2: 'rotating'
}
main_state=0

def clbk_laser(msg):
    """
    Read sensor messagens, and determine distance to each region. 
    Manipulates the values measured by the sensor.
    Callback function for the subscription to the published Laser Scan values.
    """
    global regions_, e, angle_min, dist_front, diff_e, direction, bool_outer_corner, bool_inner_corner, index, last_kinds_of_wall
    size = len(msg.ranges)
    min_index = size*(direction+1)/4
    max_index = size*(direction+3)/4
    
    # Determine values for PD control of distance and P control of angle
    for i in range(min_index, max_index):
        if msg.ranges[i] < msg.ranges[min_index] and msg.ranges[i] > 0.01:
            min_index = i
    angle_min = (min_index-size/2)*msg.angle_increment
    dist_min = msg.ranges[min_index]
    dist_front = msg.ranges[size/2]
    diff_e = min((dist_min - wall_dist) - e, 100)
    e = min(dist_min - wall_dist, 100)

    # Determination of minimum distances in each region
    regions_ = {
        'bright':  min(min(msg.ranges[0:51]), inf),
        'right': min(min(msg.ranges[52:102]), inf),
        'fright':  min(min(msg.ranges[103:153]), inf),
        'front':  min(min(msg.ranges[154:205]), inf),
        'fleft':   min(min(msg.ranges[206:256]), inf),
        'left':   min(min(msg.ranges[257:307]), inf),
        'bleft':   min(min(msg.ranges[308:359]), inf),
    }
    #rospy.loginfo(regions_)

    # Detection of Outer and Inner corner
    bool_outer_corner = is_outer_corner()
    bool_inner_corner = is_inner_corner()
    if bool_outer_corner == 0 and bool_inner_corner == 0:
        last_kinds_of_wall[index]=0
    
    # Indexing for last five pattern detection
    # This is latter used for low pass filtering of the patterns
    index = index + 1 #5 samples recorded to asses if we are at the corner or not
    if index == len(last_kinds_of_wall):
        index = 0
        
    take_action()

def change_state(state):
    """
    Update machine state
    """
    global state_, state_dict_
    if state is not state_:
        #print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def take_action():
    """
    Change state for the machine states in accordance with the active and inactive regions of the sensor.
            State 0 No wall found - all regions infinite - Random Wandering
            State 1 Wall found - Following Wall
            State 2 Pattern sequence reached - Rotating
    """
    global regions_, index, last_kinds_of_wall, index_state_outer_inner, state_outer_inner, loop_index, loop_index_outer_corner
    
    global wall_dist, max_speed, direction, p, d, angle, dist_min, wall_found, rotating, bool_outer_corner, bool_inner_corner, main_state

    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    # Patterns for rotating
    rotate_sequence_V1 = ['I', 'C', 'C', 'C']
    rotate_sequence_V2 = [0, 'C', 'C', 'C']
    rotate_sequence_W = ['I', 'C', 'I', 'C']

    if rotating == 1:
        state_description = 'case 2 - rotating'
        change_state(2)
        if(regions['left'] < wall_dist or regions['right'] < wall_dist):
            rotating = 0
    elif regions['fright'] == inf and regions['front'] == inf and regions['right'] == inf and regions['bright'] == inf and regions['fleft'] == inf and regions['left'] == inf and regions['bleft'] == inf:
        state_description = 'case 0 - random wandering'
        change_state(0)
    elif (loop_index == loop_index_outer_corner) and (rotate_sequence_V1 == state_outer_inner or rotate_sequence_V2 == state_outer_inner or rotate_sequence_W == state_outer_inner):
        state_description = 'case 2 - rotating'
        change_direction()
        state_outer_inner = [ 0, 0,  0, 'C']
        change_state(2)
    else:
        state_description = 'case 1 - following wall'
        change_state(1)

    # if regions['fright'] < inf_ and regions['fleft'] < inf_ and regions['left'] > inf_ and regions['right'] > inf_ and regions['bright'] > inf_ and regions['bleft'] > inf_:
    #     main_state = 0
    # elif regions['fright'] < inf_ and regions['fleft'] < inf_ and regions['left'] < inf_ and regions['right'] < inf_ and regions['bright'] > inf_ and regions['bleft'] > inf_:
    #     main_state = 1
    # elif regions['fright'] < inf_ and regions['fleft'] < inf_ and regions['left'] < inf_ and regions['right'] < inf_ and regions['bright'] < inf_ and regions['bleft'] < inf_:
    #     main_state = 1
    # elif regions['fright'] > inf_ and regions['fleft'] > inf_ and regions['left'] < inf_ and regions['right'] < inf_ and regions['bright'] < inf_ and regions['bleft'] < inf_:
    #     main_state = 0
    # elif regions['fright'] > inf_ and regions['fleft'] > inf_ and regions['left'] > inf_ and regions['right'] > inf_ and regions['bright'] < inf_ and regions['bleft'] < inf_:
    #     main_state = 2
    # elif regions['fright'] < inf_ and regions['fleft'] > inf_ and regions['left'] > inf_ and regions['right'] > inf_ and regions['bright'] < inf_ and regions['bleft'] > inf_:
    #     main_state = 3

def random_wandering():
    """
    This function defines the linear.x and angular.z velocities for the random wandering of the robot.
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> [0.1, 0.3]
                    msg.angular.z -> [-1, 1]
    """
    global direction, last_vel
    msg = Twist()
    msg.linear.x = max(min( last_vel[0] + random.uniform(-0.01,0.01),0.3),0.1)
    msg.angular.z= max(min( last_vel[1] + random.uniform(-0.1,0.1),1),-1)
    if msg.angular.z == 1 or msg.angular.z == -1:
        msg.angular.z = 0
    last_vel[0] = msg.linear.x
    last_vel[1] = msg.angular.z
    return msg

def following_wall():
    """
    PD control for the wall following state. 
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> 0; 0.5max_speed; 0.4max_speed
                    msg.angular.z -> PD controller response
    """
    global wall_dist, max_speed, direction, p, d, angle, dist_min, dist_front, e, diff_e, angle_min
    msg = Twist()
    if dist_front < wall_dist:
        msg.linear.x = 0
    elif dist_front < wall_dist*2:
        msg.linear.x = 0.5*max_speed
    elif abs(angle_min) > 1.75:
        msg.linear.x = 0.4*max_speed
    else:
        msg.linear.x = max_speed
    msg.angular.z = max(min(direction*(p*e+d*diff_e) + angle*(angle_min-((math.pi)/2)*direction), 2.5), -2.5)
    #print 'Turn Left angular z, linear x %f - %f' % (msg.angular.z, msg.linear.x)
    return msg

def change_direction():
    """
    Toggle direction in which the robot will follow the wall
        1 for wall on the left side of the robot and -1 for the right side
    """
    global direction, last_change_direction, rotating
    print 'Change direction!'c28x_data=os.popen("curl -s http://169.254.254.169/cmd?=C01"+cmd).read().strip()
    elapsed_time = time.time() - last_change_direction_time # Elapsed time since last change direction
    if elapsed_time >= 20:
        last_change_direction = time.time()
        direction = -direction # Wall in the other side now
        rotating = 1

def rotating():
    """
    Rotation movement of the robot. 
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> 0m/s
                    msg.angular.z -> -2 or +2 rad/s
    """
    global direction
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = direction*2
    return msg


def is_outer_corner():
    """
    Assessment of outer corner in the wall. 
    If all the regions except for one of the back regions are infinite then we are in the presence of a possible corner.
    If all the elements in last_kinds_of_wall are 'C' and the last time a real corner was detected is superior or equal to 30 seconds:
        To state_outer_inner a 'C' is appended and 
        The time is restart.
    Returns:
            bool_outer_corner: 0 if it is not a outer corner; 1 if it is a outer corner
    """
    global regions_, last_kinds_of_wall, last_outer_corner_detection_time, index, state_outer_inner, index_state_outer_inner, loop_index, loop_index_outer_corner
    regions = regions_
    bool_outer_corner = 0
    if (regions['fright'] == inf and regions['front'] == inf and regions['right'] == inf and regions['bright'] < inf  and regions['left'] == inf and regions['bleft'] == inf and regions['fleft'] == inf) or (regions['bleft'] < inf and regions['fleft'] == inf and regions['front'] == inf and regions['left'] == inf and regions['right'] == inf and regions['bright'] == inf and regions['fright'] == inf):
        bool_outer_corner = 1 # It is a corner
        last_kinds_of_wall[index]='C'
        elapsed_time = time.time() - last_outer_corner_detection_time # Elapsed time since last corner detection
        if last_kinds_of_wall.count('C') == len(last_kinds_of_wall) and elapsed_time >= 30:
            last_outer_corner_detection_time = time.time()
            loop_index_outer_corner = loop_index
            state_outer_inner = state_outer_inner[1:]
            state_outer_inner.append('C')
            print 'It is a outer corner'
    return bool_outer_corner

def is_inner_corner():
    """
    Assessment of inner corner in the wall. 
    If the three front regions are inferior than the wall_dist.
    If all the elements in last_kinds_of_wall are 'I' and the last time a real corner was detected is superior or equal to 20 seconds:
        To state_outer_inner a 'I' is appended and 
        The time is restart.
    Returns:
            bool_inner_corner: 0 if it is not a inner corner; 1 if it is a inner corner
    """
    global regions_, wall_dist, last_kinds_of_wall, last_inner_corner_detection_time, index, state_outer_inner, index_state_outer_inner, loop_index_inner_corner, loop_index
    regions = regions_
    bool_inner_corner = 0
    if regions['fright'] < wall_dist and regions['front'] < wall_dist and regions['fleft'] < wall_dist:
        bool_inner_corner = 1
        last_kinds_of_wall[index]='I'
        elapsed_time = time.time() - last_inner_corner_detection_time # Elapsed time since last corner detection
        if last_kinds_of_wall.count('I') == len(last_kinds_of_wall) and elapsed_time >= 20:
            last_inner_corner_detection_time = time.time()
            loop_index_inner_corner = loop_index
            state_outer_inner = state_outer_inner[1:]
            state_outer_inner.append('I')
            print 'It is a inner corner'
    return bool_inner_corner
    
def slow_forward():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = 0
    return msg

def turn_right():
    msg = Twist()
    msg.linea.x = 0
    msg.angular.z = -1
    return msg


def main():
    global pub_, active_, hz, loop_index, main_state
    
    rospy.init_node('follow_wall')
    
    pub_ = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    print 'Code is running'
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        loop_index = loop_index + 1
        msg = Twist()
        if(main_state == 0):
            msg = slow_forward()
        elif(main_state == 1):
            # State Dispatcher
            if state_ == 0:
                msg = random_wandering()
            elif state_ == 1:
                msg = following_wall()
            elif state_ == 2:
                msg = rotating()
            else:
                rospy.logerr('Unknown state!')
        elif(main_state == 2 ):
            msg = turn_right()
        elif(main_state == 3 ):
            
            # Configure depth and color streams
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

            # Start streaming
            profile=pipeline.start(config)
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = depth_sensor.get_depth_scale()

            try:
                while True:
                    # Wait for a coherent pair of frames: depth and color
                    frames = pipeline.wait_for_frames()
                    depth_frame = frames.get_depth_frame()
                    color_frame = frames.get_color_frame()
                    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                    color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
                    depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)
                    if not depth_frame or not color_frame:
                        continue
                    color_image=np.asanyarray(color_frame.get_data())
                    depth_image=np.asanyarray(depth_frame.get_data())
                    hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV) 
                    lower = np.array([35,43,46]) 
                    upper = np.array([77, 255, 255]) 
                    mask = cv2.inRange(hsv,lower,upper)
                    moments = cv2.moments(mask)
	                res=cv2.bitwise_and(color_image,color_image,mask=mask)
	                imgray=cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
                    zeros=cv2.countNonZero(imgray)
                    zero=float(zeros)/307200
                	v=0
	                w=0
	                if zero>0.15:
                        m00 = moments['m00']
                    if m00 != 0:
                        centroid_x = int(moments['m10']/m00)#Take X coordinate
                        centroid_y = int(moments['m01']/m00)#Take Y coordinate
                        cv2.circle(res, (centroid_x,centroid_y), 15, (0,0,255))
                        depth_pixel=[centroid_x,centroid_y]
                        if centroid_x<480 and centroid_y<640:  
                            depth_value = depth_image[centroid_x,centroid_y].astype(float)
                            depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_value*depth_scale)
                            depth_point[1]-=0.08
	                        depth_point[2]+=0.15
	                        print(depth_point)
	                        if depth_point[2]>0.2:
		                        v=0.2
		                        w=math.atan(depth_point[2]/depth_point[0])/(depth_point[2]/v)
                    msg=Twist()
                    msg.linear.X=v
                    msg.angular.z=w
                    cmd="%05.2f*%05.2f"%(msg.linear.x,msg.angular.z)
                    print cmd
	                c28x_data=os.popen("curl -s http://169.254.254.169/cmd?=C01"+cmd).read().strip()
                    rate.sleep()
        finally:
            pipeline.stop()




        else:
            rospy.logerr('Unknown main state!')
        
        pub_.publish(msg)
        
        cmd="%05.2f*%05.2f"%(msg.linear.x,msg.angular.z)
        print cmd
	    c28x_data=os.popen("curl -s http://169.254.254.169/cmd?=C01"+cmd).read().strip()
 
        
        rate.sleep()

if __name__ == '__main__':
    main()

