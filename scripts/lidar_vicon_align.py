#!/usr/bin/env python3
import rospy
from robotdatapy import transform
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from geometry_msgs.msg import PoseStamped

class LidarViconAlign:
    def __init__(self):
 
        # create a dlio subscriber to subscribe to the topic /dlio/odom_node/pose
        self.dlio_initialization_subscriber_ = rospy.Subscriber("/dlio/odom_node/pose", PoseStamped, self.initialize_dlio_subscriber, queue_size=10)

        # create a vicon subscriber to subscribe to the topic /NX02/world
        self.vicon_initialization_subscriber_ = rospy.Subscriber("/NX02/world", PoseStamped, self.initialize_vicon_subscriber, queue_size=10)

        # create a publisher to publish to the topic /pose_aligner/pose_aligned
        self.publisher_ = rospy.Publisher("/pose_aligner/pose_aligned", PoseStamped, queue_size=10)
 
        # initialization variables
        self.dlio_initial_pose_ = [0, 0, 0, 0, 0, 0] # x, y, z, roll, pitch, yaw
        self.vicon_initial_pose_ = [0, 0, 0, 0, 0, 0] # x, y, z, roll, pitch, yaw
        self.T_dlio_ = None
        self.T_vicon_ = None

        # flag for initialization
        self.dlio_initialization_cnt_ = 0
        self.vicon_initialization_cnt_ = 0

        # this is user defined fixed transformation from lidar to vicon
        x_offset = 0.0
        y_offset = 0.0
        z_offset = -0.3
        roll_offset = 0.0
        pitch_offset = 0.0
        yaw_offset = 0.0
        quatertion_offset = Rot.from_euler('xyz', [roll_offset, pitch_offset, yaw_offset], degrees=False).as_quat()
        self.T_lidar_vicon_ = transform.xyz_quat_to_transform([x_offset, y_offset, z_offset], quatertion_offset)

    # callback for the dlio subscriber
    def callback_dlio_subscriber(self, msg):
        """Transform the dlio pose to the vicon frame and publish it

        Args:
            msg (PoseStamped): The dlio pose message

        """
         
        # get transformation matrix from msg
        quaternion_dlio = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        T_dlio = transform.xyz_quat_to_transform([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], quaternion_dlio)

        # apply the transformation
        T_result = self.T_vicon_ @ np.linalg.inv(self.T_dlio_) @ T_dlio @ self.T_lidar_vicon_

        # get the pose from the transformation matrix
        pose = transform.transform_to_xyz_quat(T_result)

        # publish the pose
        msg_out = PoseStamped()
        msg_out.header = msg.header
        msg_out.pose.position.x = pose[0]
        msg_out.pose.position.y = pose[1]
        msg_out.pose.position.z = pose[2]
        msg_out.pose.orientation.x = pose[3]
        msg_out.pose.orientation.y = pose[4]
        msg_out.pose.orientation.z = pose[5]
        msg_out.pose.orientation.w = pose[6]
        self.publisher_.publish(msg_out)

    # initialize the dlio subscriber
    def initialize_dlio_subscriber(self, msg):

        # if not initialized, keep collecting data
        if self.dlio_initialization_cnt_ < 100:
            
            # x, y, z
            self.dlio_initial_pose_[0] += msg.pose.position.x
            self.dlio_initial_pose_[1] += msg.pose.position.y
            self.dlio_initial_pose_[2] += msg.pose.position.z
            
            # roll, pitch, yaw
            quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
            euler = Rot.from_matrix(quaternion).as_euler('ZYX', degrees=False)[::-1]
            self.dlio_initial_pose_[3] += euler[0]
            self.dlio_initial_pose_[4] += euler[1]
            self.dlio_initial_pose_[5] += euler[2]

            # increment the counter
            self.dlio_initialization_cnt_ += 1

            # if not initialized, return
            return
        
        # once collected enough data, calculate the average
        self.dlio_initial_pose_[0] /= 100
        self.dlio_initial_pose_[1] /= 100
        self.dlio_initial_pose_[2] /= 100
        self.dlio_initial_pose_[3] /= 100
        self.dlio_initial_pose_[4] /= 100
        self.dlio_initial_pose_[5] /= 100

        # get transformation matrix from xyzrpy
        quaternion_dlio = Rot.from_euler('xyz', self.dlio_initial_pose_[3:], degrees=False).as_quat()
        self.T_dlio_ = transform.xyz_quat_to_transform(self.dlio_initial_pose_[:3], quaternion_dlio)

        # print the initial pose
        print("Dlio Initial Pose: ", self.dlio_initial_pose_)

        # destroy the subscriber
        self.destroy_subscription(self.dlio_initialization_subscriber_)

        # start the actual subscriber
        self.dlio_subscriber_ = self.create_subscription(PoseStamped, "/dlio/odom_node/pose", self.callback_dlio_subscriber, 10)

    # initialize the vicon subscriber
    def initialize_vicon_subscriber(self, msg):
            
            # if not initialized, keep collecting data
            if self.vicon_initialization_cnt_ < 100:
                
                # x, y, z
                self.vicon_initial_pose_[0] += msg.pose.position.x
                self.vicon_initial_pose_[1] += msg.pose.position.y
                self.vicon_initial_pose_[2] += msg.pose.position.z
                
                # roll, pitch, yaw
                quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
                euler = Rot.from_matrix(quaternion).as_euler('ZYX', degrees=False)[::-1]
                self.vicon_initial_pose_[3] += euler[0]
                self.vicon_initial_pose_[4] += euler[1]
                self.vicon_initial_pose_[5] += euler[2]
    
                # increment the counter
                self.vicon_initialization_cnt_ += 1
    
                # if not initialized, return
                return
            
            # once collected enough data, calculate the average
            self.vicon_initial_pose_[0] /= 100
            self.vicon_initial_pose_[1] /= 100
            self.vicon_initial_pose_[2] /= 100
            self.vicon_initial_pose_[3] /= 100
            self.vicon_initial_pose_[4] /= 100
            self.vicon_initial_pose_[5] /= 100

            # get transformation matrix from xyzrpy
            quaternion_vicon = Rot.from_euler('xyz', self.vicon_initial_pose_[3:], degrees=False).as_quat()
            self.T_vicon_ = transform.xyz_quat_to_transform(self.vicon_initial_pose_[:3], quaternion_vicon)
    
            # print the initial pose
            print("Vicon Initial Pose: ", self.vicon_initial_pose_)
    
            # destroy the subscriber
            self.destroy_subscription(self.vicon_initialization_subscriber_)

def main(args=None):
    rospy.init_node('lidar_vicon_align', args)
    LidarViconAlign()
    rospy.spin()
 
if __name__ == "__main__":
    main()