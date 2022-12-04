#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
import rospy
from std_msgs.msg import String
import numpy as np
x = []
y = []
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.pose.position)
    x.append(data.pose.position.x)
    y.append(data.pose.position.y)
    #rospy.loginfo(len(x))
def get_waypoints():
    rospy.loginfo("shutdown time!")
    #save x,y in a dictionary as a npz file
    np.savez("/home/george/catkin_ws/src/mushr_ros_intro/trajectory03.npz",x=np.array(x), y=np.array(y))
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/car/car_pose', PoseStamped, callback)
    rospy.on_shutdown(get_waypoints)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()