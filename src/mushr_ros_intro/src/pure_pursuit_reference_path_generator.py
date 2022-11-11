#!/usr/bin/env python

# Every node starts with the above line. This ensures that this script is executed as a Python script.
# Change this to listen to the car/car_pose publisher (the node that publishes the pose of the car)
## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

# rospy is the pure Python client library for ROS. Enables interface with ROS topics, Services, and parameters
import rospy
# Standard ROS messages include comon message types representing primitve data. We are importing strings
from std_msgs.msg import String
# 
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

# Defines how the node interfaces with the rest of ROS
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # Declares our node subscribes to the chatter topc which is of type String
    # When a new message is received callback is invoked with message as the first argument
    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
