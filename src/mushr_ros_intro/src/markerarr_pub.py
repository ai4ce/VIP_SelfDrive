#!/usr/bin/env python
import sys
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import numpy as np

#init_node and publisher
topic = '/visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray,queue_size=100)

rospy.init_node('register')

markerArray = MarkerArray()

'''load the trajectory'''
def load_traj():
    # Return N x 2 trajectory
    
    data = np.load('/home/george/catkin_ws/src/mushr_ros_intro/trajectory02.npz')
    x = data['x']
    y = data['y']

    traj = np.stack((x, y))
    traj = np.swapaxes(traj, 0, 1)
    rospy.loginfo(traj.shape)
    return traj

traj = load_traj()

count = 0
MARKERS_MAX = 10000

while not rospy.is_shutdown():
    for i in range(len(traj)):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = traj[i,0]
        marker.pose.position.y = traj[i,1]
        marker.pose.position.z = 0
        markerArray.markers.append(marker)
        
   # Renumber the marker IDs
    
        if(count > MARKERS_MAX):
            sys.exit()
        #markerArray.markers.pop(0)
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1

    

        

   # Publish the MarkerArray
        publisher.publish(markerArray)

        count += 1

        rospy.sleep(0.01)