import sys
import numpy as np


class PurePursuit():
	def __init__(self, lookahead, target_vel):
		'''
		Parameters:
			lookahead (double): the lookahead distance
			target_vel (double): max velocity for the robot
		'''
		self.lookahead = lookahead

		self.target_vel = target_vel
		self.ang_vel = 0
		self.ang_vel_thresh = 2

		self.waypt_number = 0
		self.current_waypt_location = 0

		self.x_pos = self.y_pos = self.theta = 0

		self.waypts = []
		self.num_waypts = 0
		self.point_goal = []

		self.curvature = 0

		self.reset_flag = True

	def geom_point_to_array(self, pt):
		return np.array([pt.x, pt.y])

	def update_waypts(self, waypts):
		self.num_waypts = len(waypts)
		self.waypts = waypts
		self.point_goal = self.geom_point_to_array(waypts[0])

	def update_pos(self, x, y, theta):
		self.x_pos = x
		self.y_pos = y
		self.theta = theta

	def angle_bw_2lines(self, a, b, c):
		ba = a - b
		bc = c - b
		cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
		angle = np.arccos(cosine_angle)
		return angle

	def calculate_dist_to(self, pt):
		return np.linalg.norm(self.xy_pos() - self.geom_point_to_array(pt))

	def xy_pos(self):
		return np.array([self.x_pos, self.y_pos])

	def find_closest_point(self):
		'''
		Determine the closest waypoint to the robot.
		This function returns the coordinate of the closest waypoint, and sets the
		waypt_number class variable.
		'''
		min_dist = sys.float_info.max
		for i in range(self.num_waypts):
			if (min_dist > self.calculate_dist_to(self.waypts[i])):
				min_dist = self.calculate_dist_to(self.waypts[i])
				self.waypt_number = i

		return self.waypts[self.waypt_number]

	def find_lookahead_pt(self):
		'''
		Imagine a circle around the robot with a radius of the lookahead distance.
		The waypoints create a series of line segments. The intersection
		of these line segments and this circle is the lookahead point.

		Sets the point_goal class variable
		'''

		if not self.reset_flag:
			current_waypt_number = int(np.floor(self.current_waypt_location))
			print("Current Waypt:", current_waypt_number)
			if (current_waypt_number + 1 < self.num_waypts):
				# Find a visible waypoint
				for i in range(self.num_waypts - 1):
					if (current_waypt_number + 1 >= self.num_waypts):
						break

					waypt_path_segment = \
						self.geom_point_to_array(self.waypts[current_waypt_number + 1]) \
						- self.geom_point_to_array(self.waypts[current_waypt_number])

					waypt_offset = self.geom_point_to_array(
						self.waypts[current_waypt_number]) - self.xy_pos()

					# Circle and line-segment intersection
					a = np.dot(waypt_path_segment, waypt_path_segment)
					b = 2 * (np.dot(waypt_path_segment, waypt_offset))
					c = np.dot(waypt_offset, waypt_offset) - np.square(self.lookahead)

					t = np.roots([a, b, c])

					# How far along the waypt_path_segment the intersection point is
					t = np.max(t)

					if ((t <= 1) and (t >= 0) and np.isreal(t) and (
						current_waypt_number + t > self.current_waypt_location)):

						self.point_goal = self.geom_point_to_array(
							self.waypts[current_waypt_number]) + t * waypt_path_segment

						self.current_waypt_location = current_waypt_number + t

						break
					current_waypt_number += 1
		else:
			self.point_goal = self.geom_point_to_array(self.waypts[0])
			if self.calculate_dist_to(self.waypts[0]) < 0.5:
				self.reset_flag = False
		print("Point goal:", self.point_goal)
		return self.point_goal

	def find_curvature(self):
		'''
		After finding the goal point, the robot calculates the
		constant curvature needed to reach that point.

		Sets the curvature class variable
		'''
		# Using point-slope form, y/x = tan(theta).
		# After conversion to standard form:
		a = -np.tan(self.theta)
		b = 1
		c = self.x_pos * np.tan(self.theta) - self.y_pos

		# Point-line distance formula: |ax + by + c| / sqrt(a^2 + b^2)
		distance = abs(a * self.point_goal[0] + b * self.point_goal[1] + c) \
			/ np.sqrt(np.square(a) + np.square(b))

		sign_pos = self.point_goal - self.xy_pos()

		# Determine which direction to turn the robot
		side = sign_pos[0] * np.sin(self.theta) - sign_pos[1] * np.cos(self.theta)
		side = np.sign(side)

		self.curvature = 2 * distance / np.square(self.lookahead) * side * -1

	def motion_update(self):
		'''
		After finding the goal point, the robot calculates the
		constant curvature needed to reach that point.
		This function returns a tuple containing the linear and angular velocities
		to reach the coordinate specified by the point_goal class variable

		Returns:
			double: x velocity
			double: angular velocity
		'''
		narrowed_curvature = np.clip(self.curvature, -1, +1)

		vel_decay = np.absolute(narrowed_curvature)
		vel_decay = 0.5 + (1 - vel_decay) * 0.5

		self.ang_vel = self.target_vel * self.curvature * vel_decay

		self.ang_vel = np.clip(
			self.ang_vel, -self.ang_vel_thresh, self.ang_vel_thresh)

		return self.target_vel * vel_decay, self.ang_vel

	def check_reset(self):
		'''
		If the robot is close to the first waypoint, meaning it has done a full loop,
		make the next goal the first waypoint, so the robot can loop.
		'''
		if (np.linalg.norm(
			self.xy_pos() - self.geom_point_to_array(self.waypts[-1])) < 0.2):

			print("Reset triggered")
			self.reset_flag = True
			self.point_goal = self.geom_point_to_array(self.waypts[0])
			self.waypt_number = 0
			self.current_waypt_location = 0
