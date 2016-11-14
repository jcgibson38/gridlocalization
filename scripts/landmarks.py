#!/usr/bin/env python
import rospy
import rosbag
from visualization_msgs.msg import Marker

# global var #
rate = 0

def init():
    global rate
    rospy.init_node("landmarks", anonymous=True)
    rate = rospy.Rate(10) #hz
    publishtags()

def publishtags():
    global pub,rate
    positions = [(1.25,5.25),(1.25,3.25),(1.25,1.25),(4.25,1.25),(4.25,3.25),(4.25,5.25)]
    while not rospy.is_shutdown():
        for i in range(len(positions)):
            # Configure Marker #
            tag = Marker()
            tag.header.frame_id = "/my_frame"
            tag.header.stamp = rospy.Time.now()
            tag.ns = "basic_shapes"
            tag.id = i
            tag.type = Marker.CYLINDER
            tag.action = Marker.ADD
            tag.pose.position.x = positions[i][0]
            tag.pose.position.y = positions[i][1]
            tag.pose.position.z = 0
            tag.pose.orientation.x = 0.0
            tag.pose.orientation.y = 0.0
            tag.pose.orientation.z = 0.0
            tag.pose.orientation.w = 1.0
            tag.scale.x = 0.25
            tag.scale.y = 0.25
            tag.scale.z = 0.75
            tag.color.r = 0.0
            tag.color.g = 1.0
            tag.color.b = 0.0
            tag.color.a = 1.0
            tag.lifetime = rospy.Duration()
            # Wait for subscriber #
            pub.publish(tag)
            rate.sleep()
        rospy.sleep(1)

if __name__ == '__main__':
    pub = rospy.Publisher("visualization_msgs", Marker, queue_size=2)
    try:
        init()
    except rospy.ROSInterruptException:
        pass
