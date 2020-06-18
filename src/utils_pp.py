import sys
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation


###########################################################################
# Create a plot where user can input points
# Read in points to a numpy array and write it to a file called waypts.npy
###########################################################################
def collectPts(num_input_pts, x_world, y_world, x_fig, y_fig):
	fig = plt.figure(figsize=(x_fig,y_fig))
	ax1 = fig.add_subplot(1,1,1)
	ax1.set_ylim(0, y_world)
	ax1.set_xlim(0, x_world)
	waypts = np.asarray(plt.ginput(n=num_input_pts, timeout=30,))
	waypts = waypts.round(4)
	np.save("waypts", waypts)
	plt.close()


class PurePursuit():

	def __init__(self, waypts):

		self.waypts = waypts
		self.num_waypts = waypts.shape[0]
		self.lookahead = 1.5

		self.waypt_number = 0
		self.current_waypt_location = 0

		self.pt_pos = waypts[0]

		self.x_pos = self.pt_pos[0]
		self.y_pos = self.pt_pos[1]
		self.theta = self.angle_bw_2lines(
			np.array([self.waypts[0][0]+1, self.waypts[0][1]]), self.waypts[0], self.waypts[1])
		self.x_head_pos = 0
		self.y_head_pos = 0

		self.point_goal = self.waypts[0]
		self.arc_c = self.waypts[0]
		self.curvature = 0

		self.dt = 0.050

		self.vel = 1.5
		self.vel_left = 0
		self.vel_right = 0
		self.target_vel = self.vel
		self.ang_vel = 0
		self.ang_vel_thresh = 2

		self.bot_width = 0.4

		self.x_navPath = []
		self.y_navPath = []

		self.reset_flag = True

		self.waypts_curvature = [0]
		self.find_pt_to_pt_curvature()

		print("initialize", self.x_pos)


	#####################################################################################
	# Determine the curvature through every waypoint based on the last and next waypoints
	#
	# This function adds values to the waypts_curvature array.
	#####################################################################################
	def find_pt_to_pt_curvature(self):
		for i in range(1, self.num_waypts - 1):
			x1 = self.waypts[i-1][0]
			y1 = self.waypts[i-1][1]
			x2 = self.waypts[i][0]
			y2 = self.waypts[i][1]
			x3 = self.waypts[i+1][0]
			y3 = self.waypts[i+1][1]

			m1 = x2 - x1
			m2 = y2 - y1
			n1 = x3 - x2
			n2 = y3 - y2
			m3 = ((x2*x2 + y2*y2) - (x1*x1 + y1*y1)) / 2
			n3 = ((x3*x3 + y3*y3) - (x2*x2 + y2*y2)) / 2

			a = (m3*n2-m2*n3) / (m1*n2-m2*n1)
			b = (m3*n1-m1*n3) / (m2*n1-m1*n2)

			r = np.sqrt((x2-a) * (x2-a) + (y2-b) * (y2-b))
			curvature = 1 / r

			self.waypts_curvature.append(curvature)

		self.waypts_curvature.append(0)
		print(self.waypts_curvature)


	def reset_pos(self):
		self.x_pos = self.pt_pos[0]
		self.y_pos = self.pt_pos[1]
		self.theta = self.angle_bw_2lines(np.array([self.waypts[0][0]+1,self.waypts[0][1]]),self.waypts[0], self.waypts[1])
		self.point_goal = self.waypts[0]
		self.waypt_number = 0
		self.current_waypt_location = 0


	def get_pos(self):
		return (self.x_pos, self.y_pos, self.theta)


	def update_pos(self, x, y, theta):
		self.x_pos = x
		self.y_pos = y
		self.theta = theta


	def get_head_pos(self):
		self.x_head_pos = self.x_pos+self.lookahead*np.cos(self.theta)
		self.y_head_pos = self.y_pos+self.lookahead*np.sin(self.theta)
		return(self.x_head_pos, self.y_head_pos)


	def angle_bw_2lines(self, a, b, c):
		ba = a - b
		bc = c - b
		cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
		angle = np.arccos(cosine_angle)
		return angle


	def calculate_dist_to(self, pt):
		return np.linalg.norm(self.xy_pos() - pt)


	def xy_pos(self):
		return np.array([self.x_pos, self.y_pos])


	#####################################################################################
	# Determine the closest waypoint to the robot
	#
	# This function returns the coordinate of the closest waypoint, and sets the
	# waypt_number class variable.
	#####################################################################################
	def find_closest_point(self):
		min_dist = sys.float_info.max
		for i in range(self.num_waypts):
			if (min_dist > self.calculate_dist_to(self.waypts[i])):
				min_dist = self.calculate_dist_to(self.waypts[i])
				self.waypt_number = i
		
		return self.waypts[self.waypt_number]


	#####################################################################################
	# Imagine a circle around the robot with a radius equal to the lookahead distance.
	# The waypoints create a series of line segments. 
	# The intersection of these line segments and this circle is the lookahead point.
	#
	# This function sets the point_goal class variable and updates reset_flag
	#####################################################################################
	def find_lookahead_pt(self):

		if not self.reset_flag:
			current_waypt_number = int(np.floor(self.current_waypt_location))
			if (current_waypt_number + 1 < self.num_waypts):

				# Starting at the current waypoint number, loop through the remaining 
				# waypoints until one is found that is visible by the robot.
				for i in range(self.num_waypts - 1):
					# print("current_waypt_number", current_waypt_number)
					# print ("i", i)

					if (current_waypt_number + 1 >= self.num_waypts):
						break

					waypt_path_segment = self.waypts[current_waypt_number + 1] - self.waypts[current_waypt_number]
					waypt_offset = self.waypts[current_waypt_number] - self.xy_pos()
					# print("waypt_path_segment" ,waypt_path_segment)
					# print("waypt_offset" ,waypt_offset)
					# print("waypt", self.waypts[current_waypt_number])
					# print("xy_pos", self.xy_pos())

					# Circle and line-segment collision detection
					a = np.dot(waypt_path_segment, waypt_path_segment)
					b = 2 * (np.dot(waypt_path_segment, waypt_offset))
					c = np.dot(waypt_offset, waypt_offset) - np.square(self.lookahead)
					# print("a b c" ,a,b,c)

					# Shortcut to using the quadratic formula
					t = np.roots([a, b, c])

					# The value of t represents how far along the waypt_path_segment the intersection point is
					t = np.max(t)

					# If the t value is < 0 or > 1, the robot cannot even see the line segment created by the next and current waypoint. 
					# We must move on and look at the next line segment.
					if ((t <= 1) and (t >= 0) and np.isreal(t) and (current_waypt_number + t > self.current_waypt_location)):
						self.point_goal = self.waypts[current_waypt_number] + t * waypt_path_segment
						# print("point_goal", self.point_goal)
						self.current_waypt_location = current_waypt_number + t
						# print("Running Index", self.current_waypt_location)
						break
					current_waypt_number = current_waypt_number + 1
		else:
			self.point_goal = self.waypts[0]
			if self.calculate_dist_to(self.waypts[0]) < 0.5:
				self.reset_flag = False

		return self.point_goal


	#####################################################################################
	# After finding the goal point, the robot calculates the 
	# constant curvature needed to reach that point.
	#
	# This function sets the curvature class variable
	#####################################################################################
	def find_curvature(self):

		# Using point-slope form, y/x = tan(theta). After conversion to standard form:
		a = -np.tan(self.theta)
		b = 1
		c = self.x_pos*np.tan(self.theta) - self.y_pos

		# Point-line distance formula: |ax + by + c| / sqrt(a^2 + b^2)
		distance = abs(a * self.point_goal[0] + b * self.point_goal[1] + c) / np.sqrt(np.square(a) + np.square(b))

		sign_pos = self.point_goal - self.xy_pos()

		# Determine which direction to turn the robot
		side = sign_pos[0] * np.sin(self.theta) - sign_pos[1] * np.cos(self.theta)
		side = np.sign(side)

		self.curvature = 2 * distance / np.square(self.lookahead) * side * -1


	#####################################################################################
	# After finding the goal point, the robot calculates the 
	# constant curvature needed to reach that point.
	#
	# This function returns a tuple containing the linear and angular velocities
	# to reach the coordinate specified by the point_goal class variable
	#####################################################################################
	def motion_update(self):
		narrowed_curvature = np.clip(self.curvature, -1, +1)

		vel_decay = np.absolute(narrowed_curvature)
		vel_decay = 0.5 + (1 - vel_decay) * 0.5
		# print("vel_decay", vel_decay)
		print("current_waypt_location", self.current_waypt_location)
		print("index", self.waypt_number)

		self.ang_vel = self.target_vel * self.curvature * vel_decay

		self.ang_vel = np.clip(self.ang_vel, -self.ang_vel_thresh, self.ang_vel_thresh)

		return self.target_vel * vel_decay, self.ang_vel


	#####################################################################################
	# If the robot is close to the first waypoint, meaning it has done a full loop,
	# make the next goal the first waypoint, so the robot can continue.
	#
	# This function changes multiple class variables that are used for determining
	# where the robot should move next.
	#####################################################################################
	def check_reset(self):
		if (np.linalg.norm(self.xy_pos() - self.waypts[-1]) < 0.2):
			self.reset_flag = True
			self.point_goal = self.waypts[0]
			self.waypt_number = 0
			self.current_waypt_location = 0
			self.x_navPath = []
			self.y_navPath = []