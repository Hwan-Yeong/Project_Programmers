#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
# initialize publisher
cmd_pub = None

# laser detection data
# store the minimum distance in each area (right, fright, front, fleft, left)
# minimun: 0.2m / maximun: 10m
def laser_callback(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }

    # take action based on regions data
    take_action(regions)

def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    safety_dist = 1
    straight_vel = 0.6
    reverse_vel = -0.3
    steering_vel_left = 0.3
    steering_vel_right = -0.3

    # variable to indicate the current state
    state_description = ''

    if regions['front'] > safety_dist and regions['fleft'] > safety_dist and regions['fright'] > safety_dist:
        state_description = 'case 1 : go forward (nothing)'
        linear_x = straight_vel
        angular_z = 0

    elif regions['front'] < safety_dist and regions['fleft'] > safety_dist and regions['fright'] > safety_dist:
        if regions['left'] > regions ['right']:
            state_description = 'case 2-1 : go backward and turn left(front)'
            linear_x = reverse_vel
            angular_z = steering_vel_left
        elif regions['fleft'] < regions ['fright']:
            state_description = 'case 2-1 : go backward and turn right(front)'
            linear_x = reverse_vel
            angular_z = steering_vel_right
        else :
            linear_x = reverse_vel

    elif regions['front'] > safety_dist and regions['fleft'] > safety_dist and regions['fright'] < safety_dist:
        state_description = 'case 3 : turn left (fright)'
        linear_x = 0
        angular_z = steering_vel_left

    elif regions['front'] > safety_dist and regions['fleft'] < safety_dist and regions['fright'] > safety_dist:
        state_description = 'case 4 : turn right (fleft)'
        linear_x = 0
        angular_z = steering_vel_right

    elif regions['front'] < safety_dist and regions['fleft'] > safety_dist and regions['fright'] < safety_dist:
        state_description = 'case 5 : turn left (front and fright)'
        linear_x = 0
        angular_z = steering_vel_left * 2

    elif regions['front'] < safety_dist and regions['fleft'] < safety_dist and regions['fright'] > safety_dist:
        state_description = 'case 6 : turn right (front and fleft)'
        linear_x = 0
        angular_z = steering_vel_right  * 2

    elif regions['front'] < safety_dist and regions['fleft'] < safety_dist and regions['fright'] < safety_dist:
        if regions['left'] > regions ['right']:
            state_description = 'case 7-1 : go backward and turn left(all around)'
            linear_x = reverse_vel
            angular_z = steering_vel_left * 2
        elif regions['fleft'] < regions ['fright']:
            state_description = 'case 7-2 : go backward and turn right(all around)'
            linear_x = reverse_vel
            angular_z = steering_vel_right * 2
        else :
            linear_x = reverse_vel

    elif regions['front'] > safety_dist and regions['fleft'] < safety_dist and regions['fright'] < safety_dist:
        state_description = 'case 8 - go forward(fleft and fright)'
        linear_x = straight_vel / 2
        angular_z = 0

    else:
        state_description = 'unknown case'

    msg.linear.x = linear_x
    msg.angular.z = angular_z
    cmd_pub.publish(msg)

    rospy.loginfo(state_description)

def main():
    global cmd_pub

    rospy.init_node('reading_laser')

    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    scan_sub = rospy.Subscriber('/laser/scan', LaserScan, laser_callback)

    rospy.spin()

if __name__ == '__main__':
    main()