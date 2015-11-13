#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This node republish Gazebo ground_truth_odom as a pose2d msg

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Pose2D
from math import sqrt, pow, fmod, pi, cos, sin
import os.path

class Odom2Pose2d:
	
	def __init__(self):
		
		# some parameters
		self.frame_id = rospy.get_param("frame_id", '/odom')
		self.child_frame_id = rospy.get_param("child_frame_id", '/base_link')
		self.tg_topic = rospy.get_param("ground_truth_odom", '/ground_truth_odom')
		
		self.tg_pose_topic = rospy.get_param("ground_truth_odom_pose2d", '/ground_truth_odom_pose2d')
		
		self.loop_rate = rospy.get_param("loop_rate", 30)
		
		self.x_offset = rospy.get_param("x_offset", 0.0)
		self.y_offset = rospy.get_param("y_offset", 0.0)

		self.listgt = rospy.Subscriber(self.tg_topic, Odometry, self.odom_cb)
		
		self.pubgt = rospy.Publisher(self.tg_pose_topic, Pose2D, queue_size=1)
	
	def odom_cb(self, msg):

		self.odom_gt = msg
		
		# apply offset
		self.odom_gt.pose.pose.position.x = self.odom_gt.pose.pose.position.x - self.x_offset
		self.odom_gt.pose.pose.position.y = self.odom_gt.pose.pose.position.y - self.y_offset
		
		quat = (self.odom_gt.pose.pose.orientation.x,
			    self.odom_gt.pose.pose.orientation.y,
			    self.odom_gt.pose.pose.orientation.z,
			    self.odom_gt.pose.pose.orientation.w)
		euler_ang = tf.transformations.euler_from_quaternion(quat)
		
		pose_msg = Pose2D()
		
		pose_msg.x = self.odom_gt.pose.pose.position.x
		pose_msg.y = self.odom_gt.pose.pose.position.y
		pose_msg.theta = euler_ang[2]			
		
		self.pubgt.publish(pose_msg)

if __name__ == '__main__':
	rospy.init_node('gazebo_ground_pose2d', anonymous=True)
    
	o2p = Odom2Pose2d()
	
	rate = rospy.Rate(30)
	while not rospy.is_shutdown():
		rate.sleep()
	
	rospy.loginfo("Job's done.")
