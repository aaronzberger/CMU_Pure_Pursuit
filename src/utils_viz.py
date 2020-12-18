import rospy

from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


class MarkerVisualization():
	def __init__(self):
		self.pub_marker_waypts = rospy.Publisher(
			'/visualization_waypoint', MarkerArray, queue_size=1)
		self.pub_lines_waypts = rospy.Publisher(
			'/visualization_waypoint_lines', MarkerArray, queue_size=1)
		self.pub_marker_robot_pose = rospy.Publisher(
			'/visualization_robot_pose', MarkerArray, queue_size=1)
		self.pub_marker_lookahead_circle = rospy.Publisher(
			'/visualization_lookahead', MarkerArray, queue_size=1)
		self.pub_marker_goal = rospy.Publisher(
			'/visualization_goal', MarkerArray, queue_size=1)
		self.pub_marker_pts_curv = rospy.Publisher(
			'/visualization_pts_curv', MarkerArray, queue_size=1)

		self.sub_odom = rospy.Subscriber(
			'/odometry/filtered', Odometry, self.callback_odom)

	def callback_odom(self, data):
		self.pose_now = data.pose.pose

		# Fetch the quaternion from the Odometry topic
		quat = self.pose_now.orientation
		quat_list = [quat.x, quat.y, quat.z, quat.w]

		# Convert quaternion to Euler (we will only use yaw)
		(_, _, self.yaw_now) = euler_from_quaternion(quat_list)

	def publish_marker_waypts(self, waypoints):
		markers_array_msg = MarkerArray()
		markers_array = []

		for i, waypoint in enumerate(waypoints):
			marker = Marker()
			marker.ns = "waypoints"
			marker.id = i
			marker.header.frame_id = "odom"
			marker.type = marker.CYLINDER
			marker.action = marker.ADD

			marker.scale.x = 0.5
			marker.scale.y = 0.5
			marker.scale.z = 0.05

			marker.color.r = 0.0
			marker.color.g = 1.0
			marker.color.b = 0.0
			marker.color.a = 1.0

			marker.pose.position.x = waypoint.x + self.pose_now.position.x
			marker.pose.position.y = waypoint.y + self.pose_now.position.y
			marker.pose.position.z = 0

			marker.pose.orientation.x = 0.0
			marker.pose.orientation.y = 0.0
			marker.pose.orientation.z = 0.0
			marker.pose.orientation.w = 1.0

			marker.lifetime = rospy.Duration(0)
			markers_array.append(marker)

		markers_array_msg.markers = markers_array
		self.pub_marker_waypts.publish(markers_array_msg)

	def publish_lines_waypts(self, waypoints):
		markers_array_msg = MarkerArray()
		markers_array = []

		marker = Marker()
		marker.ns = "waypoints_line"
		marker.id = 1
		marker.header.frame_id = "odom"
		marker.type = marker.LINE_STRIP
		marker.action = marker.ADD

		marker.scale.x = 0.06

		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.color.a = 0.5

		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0

		for waypoint in waypoints:
			pt = Point()
			pt.x = waypoint.x + self.pose_now.position.x
			pt.y = waypoint.y + self.pose_now.position.y
			marker.points.append(pt)

		marker.lifetime = rospy.Duration(0)
		markers_array.append(marker)

		markers_array_msg.markers = markers_array
		self.pub_lines_waypts.publish(markers_array_msg)

	def publish_marker_robot_pose(self):
		markers_array_msg = MarkerArray()
		markers_array = []

		marker = Marker()
		marker.ns = "robot_pose"
		marker.id = 1
		marker.header.frame_id = "odom"
		marker.type = marker.ARROW
		marker.action = marker.ADD

		marker.scale.x = 1
		marker.scale.y = 0.1
		marker.scale.z = 0.1

		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 0.0
		marker.color.a = 1

		marker.pose = self.pose_now

		marker.lifetime = rospy.Duration(0)
		markers_array.append(marker)

		markers_array_msg.markers = markers_array
		self.pub_marker_robot_pose.publish(markers_array_msg)

	def publish_marker_lookahead_circle(self, lookahead):
		markers_array_msg = MarkerArray()
		markers_array = []

		marker = Marker()
		marker.ns = "lookahead_circle"
		marker.id = 1
		marker.header.frame_id = "odom"
		marker.type = marker.CYLINDER
		marker.action = marker.ADD

		marker.scale.x = lookahead * 2
		marker.scale.y = lookahead * 2
		marker.scale.z = 0.05

		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.color.a = 0.4

		marker.pose = self.pose_now

		marker.lifetime = rospy.Duration(0)
		markers_array.append(marker)

		markers_array_msg.markers = markers_array
		self.pub_marker_lookahead_circle.publish(markers_array_msg)

	def publish_marker_goal(self, point_goal):
		markers_array_msg = MarkerArray()
		markers_array = []

		marker = Marker()
		marker.ns = "goal_pt"
		marker.id = 1
		marker.header.frame_id = "odom"
		marker.type = marker.SPHERE
		marker.action = marker.ADD

		marker.scale.x = 0.3
		marker.scale.y = 0.3
		marker.scale.z = 0.3

		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 0.0
		marker.color.a = 1.0

		marker.pose.position.x = point_goal[0] + self.pose_now.position.x
		marker.pose.position.y = point_goal[1] + self.pose_now.position.y
		marker.pose.position.z = 0

		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0

		marker.lifetime = rospy.Duration(0)
		markers_array.append(marker)

		markers_array_msg.markers = markers_array
		self.pub_marker_goal.publish(markers_array_msg)
