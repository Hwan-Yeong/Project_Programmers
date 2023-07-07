#!/usr/bin/env python3

import rospy
import time
import tf.transformations

from std_msgs.msg import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped

# wp_id : (x, y yaw)
# add to 'Publish Point' button in rviz -> rostopic echo /clicked-point
wp_list = {1 : (4.68, 0.0, 0.0),
           2 : (5.44, -4.34, 2.7917),
           3 : (-6.0, -4.1, 2.8417),
           4 : (-12.0, -0.6, 1.4613),
           5 : (-2.1, -3.02, 0.0525),
           6 : (1.00, -2.73, 1.5247),
           7 : (-1.12, -0.25, 3.033),
           8 : (-8.75, -0.25, 3.1415)}


def talker():
    pub_wp = rospy.Publisher('my_costa_waypoints', PoseWithCovarianceStamped, queue_size=1)
    pub_path_ready = rospy.Publisher('path_ready', Empty, queue_size=1)

    rospy.init_node('waypoint_publisher', anonymous=True)
    rate = rospy.Rate(10) # hz

    my_wp = PoseWithCovarianceStamped()
    my_wp.header.stamp = rospy.Time.now()
    my_wp.header.frame_id = 'map'

    for i in range(len(wp_list)):
        rospy.loginfo("Waypoint" + str(i))
        my_wp.pose.pose.position.x = wp_list[i+1][0]
        my_wp.pose.pose.position.y = wp_list[i+1][1]

        roll = 0.0
        pitch = 0.0
        yaw = wp_list[i+1][2]

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        my_wp.pose.pose.orientation.x = quaternion[0]
        my_wp.pose.pose.orientation.y = quaternion[1]
        my_wp.pose.pose.orientation.z = quaternion[2]
        my_wp.pose.pose.orientation.w = quaternion[3]

        while not rospy.is_shutdown():
            connections = pub_wp.get_num_connections()
            if connections > 0:
                pub_wp.publish(my_wp)
                break
            rospy.loginfo("Wait for 'my_costa_waypoints' topic")
            rate.sleep()

        rospy.loginfo("Published waypoint number " + str(i))
        time.sleep(2)

    start_command = Empty()

    while not rospy.is_shutdown():
        connections = pub_path_ready.get_num_connections()
        if connections > 0:
            pub_path_ready.publish(start_command)
            rospy.loginfo("Sent waypoint list execution command")
            break
        rospy.loginfo("Waiting for 'path_ready' topic")
        rate.sleep()

if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass