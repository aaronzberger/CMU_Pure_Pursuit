#!/home/aaron/py27/bin/python

from __future__ import print_function

import rospy

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from CMU_Path_Planning_Node.msg import path
import tf

import utils_pp
import utils_viz

##################################
# USER PARAMETERS
##################################
lookahead_distance = 2.0
target_vel = 0.4

visualize = True

# In milliseconds
min_update_time = 10
##################################


class PurePursuitNode:
	def __init__(self):
		self.clock_now = 0
		self.clock_last_motion_update = 0
		self.clock_last_rviz_update = 0

		if visualize:
			self.markerVisualization_obj = utils_viz.MarkerVisualization()
		self.pure_pursuit_obj = utils_pp.PurePursuit(lookahead_distance, target_vel)

		self.sub_trajectory = rospy.Subscriber(
			'/path_planned', path, self.callback_path)
		self.sub_clock = rospy.Subscriber(
			'/clock', Clock, self.callback_clock)

		self.pub_cmd_vel = rospy.Publisher(
			'cmd_vel', Twist, queue_size=1)
		self.pub_curvature = rospy.Publisher(
			'curvature', Float32, queue_size=1)

		self.br = tf.TransformBroadcaster()

	def callback_clock(self, data):
		self.clock_now = data.clock.secs * 1000 + int(data.clock.nsecs / 1e6)

	def callback_path(self, data):
		# Every 10 ms, calculate trajectory and move the robot
		if (self.clock_now - self.clock_last_motion_update > min_update_time):
			self.pure_pursuit_obj.update_waypts(data.pts)
			self.pure_pursuit_obj.find_lookahead_pt()
			self.pure_pursuit_obj.find_curvature()
			x_vel, ang_vel = self.pure_pursuit_obj.motion_update()

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

		# Every 10 ms, publish visualization tools to rviz
		if visualize:
			if (self.clock_now - self.clock_last_rviz_update > min_update_time):
				self.markerVisualization_obj.publish_marker_waypts(data.pts)
				self.markerVisualization_obj.publish_lines_waypts(data.pts)
				self.markerVisualization_obj.publish_marker_robot_pose()
				self.markerVisualization_obj.publish_marker_lookahead_circle(
					self.pure_pursuit_obj.lookahead)
				self.markerVisualization_obj.publish_marker_goal(
					self.pure_pursuit_obj.point_goal)

				self.clock_last_rviz_update = self.clock_now


if __name__ == "__main__":
	try:
		rospy.init_node('pure_pursuit_node')
		pure_pursuit_node_obj = PurePursuitNode()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
