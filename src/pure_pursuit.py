#!/home/aaron/py27/bin/python

from __future__ import print_function

import rospy
import sys

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# from gazebo_msgs.msg import LinkStates
from rosgraph_msgs.msg import Clock
from tf.transformations import euler_from_quaternion
import numpy as np
import tf

import utils_pp
import utils_viz

print("Python Version: %s" % sys.version)

###################################################
# SET USER PARAMETERS
###################################################
num_input_pts = 8

# Coordinate limits of input waypoints
x_world = 10
y_world = 10

# Size of the plot (adjust for your screen size)
x_fig = 10
y_fig = 10
###################################################


class PurePursuitNode:
	def __init__(self):
		self.clock_now = 0
		self.clock_last_motion_update = 0
		self.clock_last_rviz_update = 0

		self.waypts = np.load("waypts.npy")

		self.markerVisualization_obj = utils_viz.MarkerVisualization()
		self.pure_pursuit_obj = utils_pp.PurePursuit(self.waypts)

		self.sub_clock = rospy.Subscriber(
			'/clock', Clock, self.callback_clock)
		self.sub_odom = rospy.Subscriber(
			'/odometry/filtered', Odometry, self.callback_odom)
		# self.sub_gazebo_link = rospy.Subscriber(
		# 	'/gazebo/link_states', LinkStates, self.callback_gazebo_link_states)

		self.pub_cmd_vel = rospy.Publisher(
			'cmd_vel', Twist, queue_size=1)
		self.pub_curvature = rospy.Publisher(
			'curvature', Float32, queue_size=1)
		# self.pub_gazebo_to_odom = rospy.Publisher(
		# 	'/odometry/filtered', Odometry, queue_size=1)

		self.br = tf.TransformBroadcaster()

	def callback_clock(self, data):
		self.clock_now = data.clock.secs * 1000 + int(data.clock.nsecs / 1e6)

	def callback_odom(self, data):
		# Fetch the quaternion from the Odometry topic
		r_quat_ros = data.pose.pose.orientation
		r_quaternion_list = [r_quat_ros.x, r_quat_ros.y, r_quat_ros.z, r_quat_ros.w]

		# Convert quaternion to Euler (we will only use yaw)
		(roll, pitch, yaw) = euler_from_quaternion(r_quaternion_list)

		# Every 10 ms, publish visualization tools to rviz
		if (self.clock_now - self.clock_last_rviz_update > 10):
			self.markerVisualization_obj.publish_marker_waypts(self.waypts)
			self.markerVisualization_obj.publish_lines_waypts(self.waypts)
			self.markerVisualization_obj.publish_marker_robot_pose(data.pose.pose)
			self.markerVisualization_obj.publish_marker_lookahead_circle(
				data.pose.pose, self.pure_pursuit_obj.lookahead)
			self.markerVisualization_obj.publish_marker_goal(
				self.pure_pursuit_obj.point_goal)
			self.markerVisualization_obj.publish_marker_pts_curv(
				self.waypts, self.pure_pursuit_obj.waypts_curvature)

			self.clock_last_rviz_update = self.clock_now

		# Every 10 ms, calculate trajectory and move the robot
		if (self.clock_now - self.clock_last_motion_update > 10):
			robot_pose = data.pose.pose.position
			self.pure_pursuit_obj.update_pos(robot_pose.x, robot_pose.y, yaw)
			self.pure_pursuit_obj.find_lookahead_pt()
			self.pure_pursuit_obj.find_curvature()
			x_vel, ang_vel = self.pure_pursuit_obj.motion_update()
			self.pure_pursuit_obj.check_reset()

			# Create a velocity message that will move the robot
			twist = Twist()
			twist.linear.x = x_vel
			twist.linear.y = 0
			twist.linear.z = 0

			twist.angular.x = 0
			twist.angular.y = 0
			twist.angular.z = ang_vel

			self.pub_cmd_vel.publish(twist)
			self.pub_curvature.publish(self.pure_pursuit_obj.curvature)

			self.clock_last_motion_update = self.clock_now


if __name__ == "__main__":
	# Collect waypoints from user input graph
	utils_pp.collectPts(num_input_pts, x_world, y_world, x_fig, y_fig)

	try:
		rospy.init_node('pure_pursuit_node')
		pure_pursuit_node_obj = PurePursuitNode()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
