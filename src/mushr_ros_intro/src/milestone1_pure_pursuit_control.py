#!/usr/bin/env python

# Pure client library for ROS
import rospy
# NumPy adds large array support and a collection of mathematical functions
import numpy as np
# ackermann_msgs provides the steering and velocity drive messages
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
# LaserScan messages transmit LIDAR data
from sensor_msgs.msg import LaserScan
# geometry_msgs provide message types for common geometric primitves
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
# tf.transformations is a library for performing calculatiosn on 4x4 matrices
from tf.transformations import quaternion_from_euler
# Simple and easy to use PID Controller
from simple_pid import PID


# Define the left and rights of the AV in degrees
LIDAR_LEFT = 450
LIDAR_RIGHT = 270
# Define the range of the LIDAR sensor
LIDAR_RANGE = 30

# Initalize PID controller and the corresponding multiples
pid = PID(2.5, 0.2, 0.1, setpoint=0.0)

# Create a function to read in the waypoints from the refence file
# Determine the lookahaed position
# Calculate the ehading vector
# Determine the closest point on the trajectory from car position
# Calculate the trajectory vector
# Calculate the Cross-Track Error
# Use the Stanley Formulation to determine steering angle
# Imporve the look-ahead distance based on the speed of the vehicle
# Visualize reference path in Rviz
# Visualize the lookahead position
# Visualize waypoint markers

def send_init_pose(pub_init_pose, init_pose):
    """
    Publishes the initial pose of the car.
    
    :param pub_init_pose: Publisher object containing information on the publish topic, message type, and queue
    :param init_pose: 3 value list containg the x, y, z euler coordinates of the car's starting position
    """

    # Unpack the initial x,y,z coordinates
    x, y, theta = float(init_pose[0]), float(init_pose[1]), float(init_pose[2])
    # Return a quaternion from Euler angles
    q = Quaternion(*quaternion_from_euler(0, 0, theta))
    # Define the x,y coordinates of the point in the pose
    point = Point(x=x, y=y)
    # Define the initial pose
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
    # Publish the initial pose
    pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))

def lidar_callback(data, args):
    """
    Makes steering and power decisions about our car.
    Publishes those decisions.

    :param data: The data type of the publishing node
    :param args: The node that publishes
    """

    # Localize the publisher node
    pub_controls = args
    # Calculate the distance from the car to the wall at its left
    left_dist = np.nanmean(np.array(data.ranges[LIDAR_LEFT-LIDAR_RANGE:LIDAR_LEFT+LIDAR_RANGE]))
    # Calculate the distance from the car to the wall on its right
    right_dist = np.nanmean(np.array(data.ranges[LIDAR_RIGHT-LIDAR_RANGE:LIDAR_RIGHT+LIDAR_RANGE]))
    # Calculate the control value
    control_value = - left_dist + right_dist
    # Log the control value
    rospy.loginfo('Control Value: %s', control_value)

    # If the control value is not a number set it to 0
    if np.isnan(control_value): control_value = 0
    # Caculate the steering angle using the PID
    steering_angle = pid(control_value)
    # Log the steering angle
    rospy.loginfo('Steering Angle: %s', steering_angle)

    # Create the drive message containing the corrective steering angle and speed
    drive = AckermannDrive(steering_angle=steering_angle, speed=1.0)
    # Publish the steering angle and speed
    pub_controls.publish(AckermannDriveStamped(drive=drive))

if __name__ == "__main__":
    # Initialize the baseline controller node
    rospy.init_node('baseline_pid')

    # Collect the control topic to publish the steering instructions to
    control_topic = rospy.get_param("~control_topic", "/car/mux/ackermann_cmd_mux/input/navigation")
    # Congifure the publisher nodes interaface
    pub_controls = rospy.Publisher(control_topic, AckermannDriveStamped, queue_size=1)

    # Collect the intial pose topic
    init_pose_topic = rospy.get_param("~init_pose_topic", "/initialpose")
    # Publish the inital pose
    pub_init_pose = rospy.Publisher(init_pose_topic, PoseWithCovarianceStamped, queue_size=1)

    # Begin the LIDAR scan and subsequently trigger the steering and drive controls
    sub_lidar_scan = rospy.Subscriber("/car/scan", LaserScan, lidar_callback, (pub_controls))

    # Set the control publish frequency
    rospy.sleep(1.0)

    # initialize the car's initial pose
    init_pose = [0., 0., np.pi]
    send_init_pose(pub_init_pose, init_pose)
    
    # use rospy.spin and let lidar callback handle the control
    rospy.spin()