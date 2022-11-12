#!/usr/bin/env python

# Pure Python client library for ROS
import rospy
# NumPy adds support for large, multi-dimensional arrays and matrices along with a large collection of high-level mathematical functions
import numpy as np
# ackermann_msgs provides messages for robots using Ackermann steering
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
# sensor_msgs defines messages for commonly used sensors
# LaseScan gives is the singgle scan message from a planar laser range-finder used by our LIDAR
from sensor_msgs.msg import LaserScan
# geometry_msgs provides messages for common geometric primitives
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
)
# tf.transformations is a library for performing calculations on 4x4 matrices
from tf.transformations import quaternion_from_euler
# Simple and easy to use PID controller
from simple_pid import PID

# The forward direction of the car is 180 degrees
# The lidar has a precision of 0.5 degrees so double the traditional degree when converting to lidar points
# Lidar left defines the left side of the car
# Standard left value is (180 + 45) * 2
LIDAR_LEFT = 450
# Standard right value is (180 - 45) * 2
LIDAR_RIGHT = 270
# the range is assigned a number of points needed to dermine the range
LIDAR_RANGE = 30 # number of points needed to determine the range

# Create a PID controller
# The first arugument is the constant multiple applied to the proportional path
# The second arguement is the constant multiple applied to the integral path
# The third arguement is the constant multiple applied to the derivative path
# Setpoint parameter is the reference value the PID is trying to achieve, initialized to 0
pid = PID(2.5, 0.2, 0.1, setpoint=0.0)

def send_init_pose(pub_init_pose, init_pose):
    """
    Publishes the initial pose of the car.
    
    :param pub_init_pose: Publisher object containing information on the publish topic, message type, and queue
    :param init_pose: 3 value list containg the x, y, z euler coordinates of the car's starting position
    """
    # Unpack the values from the init_pose arguement
    x, y, theta = float(init_pose[0]), float(init_pose[1]), float(init_pose[2])
    # quaternion_from_euler() returns a quaternion from Euler angles
    # Quaternion() encapsulate our quaternion in the (x,y,z,w) message type
    q = Quaternion(*quaternion_from_euler(0, 0, theta))
    # Encapsulates our points in the (x,y,z) point message type (no z)
    point = Point(x=x, y=y)
    # Pose() is a pose message
    # pose is a representation of pose in free space, composed of position and orientation
    # PoseWithCovariance represents a pose in free space with uncertainty
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
    # PoseWithCovarianceStamped() expresses an estimated pose with a reference coordinate frame and timestamp
    # We take our pose with covariance and timestamp it
    # We publish our timestamped covariance pose to the node taken as a paramter
    pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))

def lidar_callback(data, args):
    """
    Makes steering and power decisions about our car.
    Publishes those decisions.

    :param data: ?
    :param args: The node that publishes
    """

    # Assign pub_conrols to the node taken as the args arguement
    pub_controls = args
    # Calculate the distance from the car to the object on its left
    # What is this data.ranges[LIDAR_LEFT-LIDAR_RANGE:LIDAR_LEFT+LIDAR_RANGE?
    # np.array() creates an array
    # np.nanmean() computes an arithmetic mean along the specified axis, ignoring NaNs
    # A Nans stand for "Not a Number" and is used to represent missing values
    left_dist = np.nanmean(np.array(data.ranges[LIDAR_LEFT-LIDAR_RANGE:LIDAR_LEFT+LIDAR_RANGE]))
    # Repeat the process above to calculate the distance of the car from the object on its right
    right_dist = np.nanmean(np.array(data.ranges[LIDAR_RIGHT-LIDAR_RANGE:LIDAR_RIGHT+LIDAR_RANGE]))
    # Calculate the control value that the PID compares with the reference value and corrects from
    control_value = - left_dist + right_dist
    # Print the control value to the screen
    # Write the control value to the node's log file
    # Write the control value to rosout
    rospy.loginfo('Control Value: %s', control_value)

    # If the control value is not a number assign it to 0
    if np.isnan(control_value): control_value = 0
    # Compute the steering angle by passing the control value to our PID controller
    # How does this work exactly?
    steering_angle = pid(control_value)
    # Print the steering angle to the screen
    # Write the steering angle to the node's log file
    # Write the steering angle to rosout
    rospy.loginfo('Steering Angle: %s', steering_angle)

    # AckermannDrive() is the driving command for a car-like vehicle using Ackermann steering
    # Steering_angle is set to the retrun value of our pid controller
    # speed is set to the desired forward speed in m/s
    # We assume front-wheel steering and given the difference in left and right front wheel angles, the steering angle corresponds to the yaw of a virtual wheel located at the center of the frotnt axle
    # Positive yaw is to the left
    drive = AckermannDrive(steering_angle=steering_angle, speed=1.0)
    # AckermannDriveStamped() time stamps the drive command
    # Publish the steering and speed information to the node taken in as args
    pub_controls.publish(AckermannDriveStamped(drive=drive))

# Define the program when the script is executed
if __name__ == "__main__":

    # Name our node baseline_pid
    rospy.init_node('baseline_pid')

    # A Parameter server is a shared multi-variate dictionary accessible via network APIs
    # Nodes use the paramter server to store and retrieve paramters at runtime
    # What are the two string path values we are getting here?
    control_topic = rospy.get_param("~control_topic", "/car/mux/ackermann_cmd_mux/input/navigation")
    # Declares that our node is publishing to the control_topic
    # Sets the message type to AckermannDriveStamped
    # Sets a maximum queue size of one so as to drop old steering instructions
    pub_controls = rospy.Publisher(control_topic, AckermannDriveStamped, queue_size=1)

    # Are we collecting the init pose topic?
    init_pose_topic = rospy.get_param("~init_pose_topic", "/initialpose")
    # Declare that the pub_init_pose publisher is published pose with covariances stamped messages, with a queue of 1 (to drop degraded locations) to the earlier collected init_pose_topic
    pub_init_pose = rospy.Publisher(init_pose_topic, PoseWithCovarianceStamped, queue_size=1)

    # Our lidar scan subscriber node subscribes to the car/scan topic which is recieving LaserScan type messages
    # Everytime data is received the lidar_callback function is called
    # The fourth arguement, pub controls, is a the arguement(s) to pass to the call back function
    # In this case we are passing a publisher node to the call back function
    sub_lidar_scan = rospy.Subscriber("/car/scan", LaserScan, lidar_callback, (pub_controls))

    # ROS will sleep for the specified period (in seconds if passed as a float)
    rospy.sleep(1.0)
    # initialize the car pose
    # np.pi is a numpy constant representing pi
    init_pose = [0., 0., np.pi]
    # Publish the intial pose of the car
    # This moves the car to the initial stated pose at the beggining of the execution of the baseline script
    send_init_pose(pub_init_pose, init_pose)

    # use rospy.spin and let lidar callback handle the control
    # Stops Python from exiting until this node is stopped
    rospy.spin()