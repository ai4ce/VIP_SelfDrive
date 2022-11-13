#!/usr/bin/env python

# Every node starts with the above line. This ensures that this script is executed as a Python script.
# Change this to listen to the car/car_pose publisher (the node that publishes the pose of the car)
## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

# rospy is the pure Python client library for ROS. Enables interface with ROS topics, Services, and parameters
import rospy
# geometry_msgs provides messages for common geometric primitives
# We are importing the PoseStamped to handle the message type of the car pose
from geometry_msgs.msg import PoseStamped

# Select the save desitantion of the reference path text file
REFERENCE_PATH_FILE_OUTPUT_PATH = "/home/hanwen/catkin_ws/src/mushr_ros_intro/src/"

# Stores the collection of waypoints that make up the reference path
reference_path_array = []

# Save the reference path to a text file
def save_path():
    """
    Saves the waypoints into a text file.
    Each waypoint is on a new line.
    """

    # Save the reference path array to a text file
    with open(REFERENCE_PATH_FILE_OUTPUT_PATH + "reference_path.txt", 'w') as f:
        # Create output file header
        f.write("x-coordinate, y-coordinate\n")
        
        # Convert the waypoints to string and then add a newline before they are written to the file
        for wp in reference_path_array:
            f.write(str(wp[0]) + ',' + str((wp[1])) + "\n")
    
# Another function calls a callback when appropriate
# We usually pass a function pointer to the call back function
def callback(data):
    # Log the car pose (for the dev's sake)
    rospy.loginfo(rospy.get_caller_id() + 'Car\'s pose: %s', data.pose)
    # Unpack the x and y position coordinates from the pose
    x, y = float(data.pose.position.x), float(data.pose.position.y)

    # Store them in the global reference array as a subarray
    waypoint = (x,y)
    reference_path_array.append(waypoint)

# Defines how the node interfaces with the rest of ROS
def reference_path_generator():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('reference_path_generator', anonymous=True)

    # Declares our node subscribes to the chatter topc which is of type String
    # When a new message is received callback is invoked with message as the first argument
    rospy.Subscriber('/car/car_pose', PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    reference_path_generator()
    rospy.on_shutdown(save_path)
