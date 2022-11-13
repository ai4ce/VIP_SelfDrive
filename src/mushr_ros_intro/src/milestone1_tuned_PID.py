#!/usr/bin/env python
from tarfile import StreamError
import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
)
from tf.transformations import quaternion_from_euler
from simple_pid import PID

LIDAR_LEFT = 400#540 # lidar data index pointing at the left side of the car
LIDAR_RIGHT = 320#180
LIDAR_RANGE = 30 # number of points needed to determine the range

pid = PID(2.5, 0.2, 0.1, setpoint=0.0)

def send_init_pose(pub_init_pose, init_pose):
    x, y, theta = float(init_pose[0]), float(init_pose[1]), float(init_pose[2])
    q = Quaternion(*quaternion_from_euler(0, 0, theta))
    point = Point(x=x, y=y)
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
    pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))

def lidar_callback(data, args):
    pub_controls = args
    left_dist = np.nanmean(np.array(data.ranges[LIDAR_LEFT-LIDAR_RANGE:LIDAR_LEFT+LIDAR_RANGE]))
    right_dist = np.nanmean(np.array(data.ranges[LIDAR_RIGHT-LIDAR_RANGE:LIDAR_RIGHT+LIDAR_RANGE]))
    control_value = - left_dist + right_dist
    rospy.loginfo('Control Value: %s', control_value)

    if np.isnan(control_value): control_value = 0
    steering_angle = pid(control_value)
    rospy.loginfo('Steering Angle: %s', steering_angle)

    
    drive = AckermannDrive(steering_angle=steering_angle, speed=1.0)
    pub_controls.publish(AckermannDriveStamped(drive=drive))

if __name__ == "__main__":

    rospy.init_node('baseline_pid')

    control_topic = rospy.get_param("~control_topic", "/car/mux/ackermann_cmd_mux/input/navigation")
    pub_controls = rospy.Publisher(control_topic, AckermannDriveStamped, queue_size=1)

    init_pose_topic = rospy.get_param("~init_pose_topic", "/initialpose")
    pub_init_pose = rospy.Publisher(init_pose_topic, PoseWithCovarianceStamped, queue_size=1)

    sub_lidar_scan = rospy.Subscriber("/car/scan", LaserScan, lidar_callback, (pub_controls))

    rospy.sleep(1.0)
    # initialize the car pose
    init_pose = [0., 0., np.pi]
    send_init_pose(pub_init_pose, init_pose)

    # use rospy.spin and let lidar callback handle the control
    rospy.spin()