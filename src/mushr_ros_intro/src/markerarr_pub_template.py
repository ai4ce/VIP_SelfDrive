#!/usr/bin/env python

# sys module provides the ability to manipulate the Python runtime environment
import sys
# roslib is the base dependency of all ROS client libraries an tools
# load_manifest() reads the package manifest and sets up the python library path based on the package dependencies
import roslib; roslib.load_manifest('visualization_marker_tutorials')
# Markers allow programmatic addition of various primitive shapes to the 3D view by sending a marker or marker array message
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
# rospy is a pure Python client library for ROS
import rospy
# What are we using math for?
import math
# numpy supports mult dimensional array and matrices and a collection of high-level mathematical functions to operate on these arrays
import numpy as np

# Set the topic path
topic = '/visualization_marker_array'
# Initialize node publishing to visualization topic using the marker array message
publisher = rospy.Publisher(topic, MarkerArray,queue_size=100)

# Set the node name
rospy.init_node('register')

# Creates an array of markers which is a message type
markerArray = MarkerArray()

# What is the trajectory?
'''load the trajectory'''
def load_traj():
    # Return N x 2 trajectory
    
    # np.load() returns the input array from a disk file with the npy extension
    data = np.load('/home/george/catkin_ws/src/mushr_ros_intro/trajectory01.npz')
    x = data['x']
    y = data['y']

    # stack() is used to join multiple NumPy arrays
    # Unlike concatenate() it joins arrays along a new axis
    traj = np.stack((x, y))
    # sawpaxes() interchanges two axis of an array
    traj = np.swapaxes(traj, 0, 1)

    # The shape of the array is a tuple with each index having the corresponding elements
    # Print the trajectory shape to the screen, write it to the Node's log file, and write it to rosout
    rospy.loginfo(traj.shape)
    return traj

# Collect the trajectory of the reference path
traj = load_traj()

# What is count for?
count = 0
# Why do we enable a maximum number of markers?
MARKERS_MAX = 10000

# While the visualizer node is not shutdown
while not rospy.is_shutdown():
    # Iterate throught the values in the trajectory array
    for i in range(len(traj)):
        # Create a marker message 
        marker = Marker()
        # Configure the marker properties
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
        # Read the coordinates of the marker in from the marker trajectory
        marker.pose.position.x = traj[i,0]
        marker.pose.position.y = traj[i,1]
        marker.pose.position.z = 0
        # Add the marker to our global marker array
        markerArray.markers.append(marker)
        
   # Renumber the marker IDs
        # What is this comparison for?
        if(count > MARKERS_MAX):
            sys.exit()
        #markerArray.markers.pop(0)
        # Initialize a uniwue id variable
        id = 0
        # Give eacb  armer an incremental id
        for m in markerArray.markers:
            m.id = id
            id += 1

    

        

   # Publish the MarkerArray
        publisher.publish(markerArray)

        count += 1

        rospy.sleep(0.01)