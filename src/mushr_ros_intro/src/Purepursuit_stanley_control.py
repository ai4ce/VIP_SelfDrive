#!/usr/bin/env python
from tarfile import StreamError
from geometry_msgs.msg import PoseStamped
import rospy
from std_msgs.msg import String
import numpy as np
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from tf.transformations import quaternion_from_euler
from simple_pid import PID
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
)
control_topic = rospy.get_param("~control_topic", "/car/mux/ackermann_cmd_mux/input/navigation")
init_pose_topic = rospy.get_param("~init_pose_topic", "/initialpose")
pid = PID(1.3, 0.1, 0.05, setpoint=0.0)

def send_init_pose(pub_init_pose, init_pose):
    x, y, theta = float(init_pose[0]), float(init_pose[1]), float(init_pose[2])
    q = Quaternion(*quaternion_from_euler(0, 0, theta))
    point = Point(x=x, y=y)
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
    pub_init_pose = rospy.Publisher(init_pose_topic, PoseWithCovarianceStamped, queue_size=1)
    #sub_point = rospy.Subscriber("/car/scan", LaserScan, lidar_callback, (pub_controls))
    pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))

#load trajectory from npz file    
def load_traj():
    # Return N x 2 trajectory
    #rospy.loginfo("run through")
    data = np.load('/home/george/catkin_ws/src/mushr_ros_intro/trajectory02.npz')
    x = data['x']
    y = data['y']

    traj = np.stack((x, y))
    traj = np.swapaxes(traj, 0, 1)

    return traj

traj = load_traj()


def callback(data, args):
    #rospy.sleep(1.0)
    
    #rospy.loginfo(traj)
    marker_pub_start = args[0]
    marker_pub_end = args[1]
    marker_traj_start = args[2]
    marker_traj_end = args[3]
    pub_controls = args[4]
    load_traj = args[5]
    # extract orientation from subscriber
    x = (data.pose.orientation.x)
    y = (data.pose.orientation.y)
    z = (data.pose.orientation.z)
    w = (data.pose.orientation.w)

    # convert quaternion to Euler for viewing
    r = R.from_quat([x,y,z,w])
    yaw = r.as_euler('zyx',degrees=False)[0]
    # rospy.loginfo('Euler %s', r.as_euler('zyx',degrees=True))

    # get current car position
    x_now = data.pose.position.x
    y_now = data.pose.position.y 
    current_pos = np.array([x_now, y_now])
    lookahead_distance = 0.75
    car_speed = 2
    lookahead_pos = np.array([x_now + lookahead_distance * np.cos(yaw), y_now + lookahead_distance * np.sin(yaw)])
    # rospy.loginfo('Current Position: %s', current_pos[0])
    # rospy.loginfo('Lookahead Position: %s', lookahead_pos[1])

    """ Lookahead Waypoints """
    N = 1
    # find neightboards of lookahead point from pre-record distance
    distance_ahead = np.linalg.norm(lookahead_pos - traj, axis=1)
    # find top 10 neighboards using distance
    idx_ahead = np.argpartition(distance_ahead, N)[:N]
    topN_dist_ahead = distance_ahead[idx_ahead]
    # rospy.loginfo('Top N Smallest Distance: %s', topN_dist)
    topN_position_ahead = traj[idx_ahead, :]
    # rospy.loginfo('Top N Points: %s', topN_position)

    """ Current Waypoints """
    # calculate distance between current car posisiton and waypoints
    distance_current = np.linalg.norm(current_pos - traj, axis=1)
    # find top 10 neighboards using distance
    idx_current = np.argpartition(distance_current, N)[:N]
    topN_dist_current = distance_ahead[idx_current]
    # rospy.loginfo('Top N Smallest Distance: %s', topN_dist)
    topN_position_current = traj[idx_current, :]
    # rospy.loginfo('Top N Points: %s', topN_position)

    """ Car Heading Vecotr"""
    heading_vector = lookahead_pos - current_pos
    heading_vector = heading_vector / np.linalg.norm(heading_vector)
    # rospy.loginfo('Heading Vecotr: %s', heading_vector)

    """ Trajectory Heading Vecotr"""
    target_position_end = np.mean(topN_position_ahead, axis=0)
    target_position_start = np.mean(topN_position_current, axis=0)
    rospy.loginfo('target_position_start:%s',target_position_start[0])
    trajector_vecotor = target_position_end - target_position_start
    trajector_vecotor = trajector_vecotor / np.linalg.norm(trajector_vecotor)
    
    '''Tatget point vector'''
    purepersuit_point_vector = target_position_end - current_pos
    # rospy.loginfo('Trajectory Vector: %s', trajector_vecotor)
    
   
    
    """ add marker to show trajectory vector """
    marker_traj_start.pose.position.x = target_position_start[0]
    marker_traj_start.pose.position.y = target_position_start[1]
    marker_pub_start.publish(marker_traj_start)
    
    marker_traj_end.pose.position.x = target_position_end[0]#lookahead_pos[0] #
    marker_traj_end.pose.position.y = target_position_end[1]#lookahead_pos[1] #
    marker_pub_end.publish(marker_traj_end)

    # Heading Error
    # heading_error = np.arccos(np.dot(heading_vector, trajector_vecotor))
    heading_error = np.arctan2(np.dot(np.cross(heading_vector, trajector_vecotor), np.array([0,0,1])), np.dot(heading_vector, trajector_vecotor))
    rospy.loginfo('Heading Error: %s\n', heading_error[2])
    pure_pursuit_heading_error = np.arctan2(np.dot(np.cross(heading_vector, purepersuit_point_vector), np.array([0,0,1])), np.dot(heading_vector, purepersuit_point_vector))
    #rospy.loginfo('pure persuit Heading Error: %s\n', pure_pursuit_heading_error[2])
    #crosstracking error
    cross_track_error = np.linalg.norm(current_pos - target_position_start)
   
    
    """ Pure-pursuit control"""
    L = 0.45
    pure_pursuit_steering_angle =np.arctan(2*L*np.sin(pure_pursuit_heading_error[2])/(lookahead_distance))
    # rospy.loginfo('pure_pursuit_steering_angle:%s\n',pure_pursuit_steering_angle)
    
    """ stanly control """
    k = 1
    #position = sign((Bx - Ax) * (Y - Ay) - (By - Ay) * (X - Ax)), determin the car position is at left or right side of the trajectorys
    sign = ((lookahead_pos[0] - current_pos[0]) *(target_position_start[1] - current_pos[1])) - ((lookahead_pos[1]-current_pos[1])*(target_position_start[0]-current_pos[0]))
    #position = np.array([[(lookahead_pos[0] - current_pos[0]),(target_position_start[0]-current_pos[0])],[(lookahead_distance[1]-current_pos[1]),(target_position_start[1] - current_pos[1])]])
    rospy.loginfo('sign: %s',sign)
    if sign<0: cross_track_error = - cross_track_error
    stanly_steering_angle =  (heading_error[2]) + np.arctan(k*cross_track_error/(car_speed)) 
    #rospy.loginfo('stanly_steering_angle:%s\n',stanly_steering_angle)
    
    """PID control"""
    control_value = -heading_error[2]
    if np.isnan(control_value): control_value = 0
    steering_angle = pid(control_value)
    #rospy.loginfo('Steering Angle: %s', steering_angle)

    
    drive = AckermannDrive(steering_angle=stanly_steering_angle, speed = car_speed)
    pub_controls.publish(AckermannDriveStamped(drive=drive))
    
def create_marker(id, color):
    marker = Marker()

    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.Time.now()
    marker.type = 2
    marker.id = id
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    if color == 'g':
        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
    if color == 'r':
        # Set the color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
    return marker
            
    

    
def listener():

    rospy.init_node('listener', anonymous=True)
    marker_traj_start = create_marker(id=0, color='r')
    marker_traj_end = create_marker(id=1, color='g')
    marker_pub_start = rospy.Publisher("/trajector_vecotor_start", Marker, queue_size = 2)
    marker_pub_end = rospy.Publisher("/trajector_vecotor_end", Marker, queue_size = 2)
    
    
    
    
    pub_controls = rospy.Publisher(control_topic, AckermannDriveStamped, queue_size=1)
    
    pub_init_pose = rospy.Publisher(init_pose_topic, PoseWithCovarianceStamped, queue_size=1)
    #sub_point = rospy.Subscriber("/car/scan", LaserScan, lidar_callback, (pub_controls))
    rospy.sleep(1.0)
    init_pose = [0., 0., np.pi]
    send_init_pose(pub_init_pose, init_pose)
    #rospy.loginfo("run through")
    
    rospy.Subscriber('/car/car_pose', PoseStamped, callback, (marker_pub_start, marker_pub_end, marker_traj_start, marker_traj_end,pub_controls,traj))
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    
    
    