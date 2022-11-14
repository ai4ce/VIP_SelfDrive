#!/usr/bin/env python
# The above line allows this program to be run as an executeable

# sys module aloows us to interacti with the Python runtime environment
import sys

# roslib is the base dependency of all ROS client libraries and tools
# Load_manifest() reads the package manifest and sets up the Python library path based on the package dependencies
import roslib; roslib.load_manifest('visualization_marker_tutorials')

# Markers allow programatic addition of shapes to the 3D view
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# rospy is the pure Python client library for ROS
import rospy

# numpy supports arrays and a collection of relevant high-level mathematical functions
import numpy as np

REFERENCE_PATH_INPUT_FILE = "/home/hanwen/catkin_ws/src/mushr_ros_intro/src/reference_path.txt"
# Create the globally accessible marker array
reference_path = MarkerArray()

def reference_path_loader():
    """
    Loads an input set of coordinates from a disk file as waypoints into the global marker array which functions as a reference path.
    """

    # Create local reference path to handle raw coordinate information
    raw_reference_path = []

    # Read in raw coordinate information from the reference path input file to the local reference path array
    with open(REFERENCE_PATH_INPUT_FILE, 'r') as f:
        for line in f:
            line = line.strip()
            raw_coordinates = line.split(',')
            cleaned_coordinates = [float(raw_coordinates[0]), float(raw_coordinates[1])]
            raw_reference_path.append(cleaned_coordinates)

    for coordinates in raw_reference_path:
        # Create a waypoint from a marker
        marker = Marker()
        
        # Configure our waypoint's properties
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

        # Read the raw coordinate infromation into the waypoint's x and y position
        marker.pose.position.x = coordinates[0]
        marker.pose.position.y = coordinates[1]
        # Zero the z position as we are dealing with 2D Space
        marker.pose.position.z = 0

        # Add the waypoint to our global reference path
        reference_path.markers.append(marker)
    
    # Give each waypoint a unique incremental ID
    id = 0
    for waypoint in reference_path.markers:
        waypoint.id = id
        id += 1

def reference_path_visiualizer():
    """
    Publish the reference path.
    """

    # Load the reference path
    reference_path_loader()

    # We are publishing the reference path which is a MakerArray type message
    pub = rospy.Publisher('/visualization_reference_path', MarkerArray, queue_size=100)
    # Set the node name
    # No need to anonymize
    rospy.init_node('reference_path_visualizer')

    # Set the publishing rate to 1 Hz
    rate = rospy.Rate(1)

    # Publish our reference path
    while not rospy.is_shutdown():
        # Publish the reference path to the visualization topic
        pub.publish(reference_path)

        # Create publish confirmation message
        publish_success = "Reference path published at %s" % rospy.get_time()

        # Confirm the reference path was published
        rospy.loginfo(publish_success)
        
        # Maintain our desired publish rate
        rate.sleep()

# Standard Python file execution check
# Maintains a 1 Hz publish rate
if __name__ == '__main__':
    try:
        # Visualize the reference path
        reference_path_visiualizer()
    except rospy.ROSInterruptException:
        pass