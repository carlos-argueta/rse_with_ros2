#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy # Import QoS

from action_msgs.msg import GoalStatus
import binascii 

import numpy as np
import time
import math

# Import messages
from geometry_msgs.msg import Point, Pose, Twist, Quaternion, PoseStamped
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Path # Import Path message
from std_msgs.msg import Header # Import Header for timestamps/frames

# Import the custom action
from waypoint_follower_interfaces.action import FollowWaypoints 


def calculate_los_point(ship_pos, prev_wp, current_wp, next_wp, prev_shortest_distance, n_ship_lengths, ship_length, logger):
	""" Calculates the Line-of-Sight (LOS) point along the path segment. """
	x, y = ship_pos
	x_k_minus_1, y_k_minus_1 = prev_wp
	x_k, y_k = current_wp
	lookahead_distance = n_ship_lengths * ship_length

	delta_x_path = x_k - x_k_minus_1
	delta_y_path = y_k - y_k_minus_1

	surge_vel_gain = 1.0

	if np.isclose(delta_x_path, 0) and np.isclose(delta_y_path, 0): # Waypoints are the same
		logger.warn("Waypoints are the same, using current waypoint as LOS.")
		return current_wp, np.inf, surge_vel_gain # Reset distance for segment logic

	if np.isclose(delta_x_path, 0): # Vertical line
		x_los = x_k
		term = lookahead_distance**2 - (x_los - x)**2
		if term < 0:
			logger.debug("No LOS intersection (vertical, term < 0)")
			# Fallback: project ship position onto the line segment
			return project_onto_segment(ship_pos, prev_wp, current_wp, logger), prev_shortest_distance, surge_vel_gain
		sqrt_term = np.sqrt(term)
		y_los1 = y + sqrt_term
		y_los2 = y - sqrt_term
		intersections = [np.array([x_los, y_los1]), np.array([x_los, y_los2])]
	else: # Non-vertical line
		m = delta_y_path / delta_x_path
		c = y_k - m * x_k
		# Quadratic formula components for intersection of circle and line
		A = m**2 + 1
		B = 2 * (m * (c - y) - x)
		C = (c - y)**2 + x**2 - lookahead_distance**2
		discriminant = B**2 - 4 * A * C
		if discriminant < 0:
			logger.debug("No LOS intersection (non-vertical, discriminant < 0)")
			# Fallback: project ship position onto the line segment
			return project_onto_segment(ship_pos, prev_wp, current_wp, logger), prev_shortest_distance, surge_vel_gain

		sqrt_discriminant = np.sqrt(discriminant)
		x_los1 = (-B + sqrt_discriminant) / (2 * A)
		x_los2 = (-B - sqrt_discriminant) / (2 * A)
		y_los1 = m * x_los1 + c
		y_los2 = m * x_los2 + c
		intersections = [np.array([x_los1, y_los1]), np.array([x_los2, y_los2])]

	# Filter intersections: Choose the one 'ahead' on the segment
	valid_intersections = []
	vec_path = current_wp - prev_wp
	dist_segment_sq = np.sum(vec_path**2)

	# Choose the intersection point that is "ahead" along the path direction
	# We want the intersection point p_los such that the vector from prev_wp to p_los
	# has a positive projection onto the path vector vec_path.
	# Also, consider the point closer to the *next* waypoint if both are valid.
	best_los_point = None
	min_dist_to_current_wp = np.inf

	for p_los in intersections:
		vec_los_to_prev = p_los - prev_wp
		dot_prod = np.dot(vec_los_to_prev, vec_path)

		# Check if point is roughly on the extended line segment in the forward direction
		# Allow projection factor to be slightly negative or greater than 1,
		# but prioritize points closer to the current_wp within the lookahead circle.
		# Simplified check: Ensure it's generally 'forward'
		if dist_segment_sq > 1e-9 and dot_prod / dist_segment_sq > -0.1: # Allows points slightly behind projection
			 dist_to_current = np.linalg.norm(p_los - current_wp)
			 # Heuristic: Prefer points closer to the current target waypoint
			 # This helps select the 'forward' intersection point naturally in most cases
			 if dist_to_current < min_dist_to_current_wp:
				 min_dist_to_current_wp = dist_to_current
				 best_los_point = p_los
				 # valid_intersections.append(p_los) # Keep track if needed

	if best_los_point is None:
		logger.debug("No 'forward' LOS intersection found, using projection.")
		return project_onto_segment(ship_pos, prev_wp, current_wp, logger), prev_shortest_distance, surge_vel_gain

	p_los = best_los_point
	shortest_dist = np.linalg.norm(p_los - current_wp) # Distance from chosen LOS pt to current WP

	# Speed damping logic: If we are getting *further* away from the target WP
	# along the path than the previous LOS point was, damp the speed.
	# This usually happens when overshooting the lookahead circle intersection logic.
	if shortest_dist > prev_shortest_distance + 1e-3: # Add tolerance
		logger.debug(f"LOS point distance increasing ({shortest_dist:.2f} > {prev_shortest_distance:.2f}), potentially overshot. Damping speed.")
		surge_vel_gain = 0.8 # Dampen speed
		# Keep the previous shortest distance to avoid continuous damping if stuck
		shortest_dist = prev_shortest_distance
		# Optional: Revert p_los to a safer point, e.g., projection or previous best?
		# p_los = project_onto_segment(ship_pos, prev_wp, current_wp, logger) # Safer fallback
	else:
		# Normal operation, update shortest distance
		 prev_shortest_distance = shortest_dist


	# logger.debug(f"Calculated LOS: {p_los}, Shortest Dist: {shortest_dist:.2f}, Gain: {surge_vel_gain:.2f}")
	return p_los, shortest_dist, surge_vel_gain


def project_onto_segment(point, seg_start, seg_end, logger):
	"""Projects a point onto the line segment defined by start and end points."""
	ap = point - seg_start
	ab = seg_end - seg_start
	ab_squared = np.dot(ab, ab)
	if np.isclose(ab_squared, 0):
		logger.warn("Segment length is zero in projection.")
		return seg_start # Segment is a point
	projection_factor = np.dot(ap, ab) / ab_squared
	projection_factor = np.clip(projection_factor, 0.0, 1.0) # Clamp to segment
	projected_point = seg_start + projection_factor * ab
	# logger.debug(f"Projected {point} onto {seg_start}->{seg_end} -> {projected_point}")
	return projected_point

def calculate_los_angle(ship_pos, p_los):
	""" Calculates the LOS angle (psi_los). """
	if p_los is None: return None
	psi_los = np.arctan2(p_los[1] - ship_pos[1], p_los[0] - ship_pos[0])
	return psi_los

def check_waypoint_acceptance(ship_pos, current_wp, acceptance_radius):
	""" Checks if the ship is within the circle of acceptance. """
	return np.sum((current_wp - ship_pos)**2) <= acceptance_radius**2

def wrap_angle(angle):
	"""Wraps an angle to the range [-pi, pi]."""
	return (angle + np.pi) % (2 * np.pi) - np.pi

def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
	"""Converts Euler angles (yaw, pitch, roll) to Quaternion."""
	cy = math.cos(yaw * 0.5)
	sy = math.sin(yaw * 0.5)
	cp = math.cos(pitch * 0.5)
	sp = math.sin(pitch * 0.5)
	cr = math.cos(roll * 0.5)
	sr = math.sin(roll * 0.5)

	q = Quaternion()
	q.w = cy * cp * cr + sy * sp * sr
	q.x = cy * cp * sr - sy * sp * cr
	q.y = sy * cp * sr + cy * sp * cr
	q.z = sy * cp * cr - cy * sp * sr
	return q

class ReferenceModel:
	""" Generates smoothed reference signals. (Identical to original) """
	def __init__(self, initial_psi=0.0, omega_n=0.5, logger=None):
		if omega_n <= 0: raise ValueError("omega_n must be positive.")
		self.omega_n = omega_n
		self.omega_n_sq = omega_n**2
		self.omega_n_cb = omega_n**3
		self.state = np.array([initial_psi, 0.0, 0.0], dtype=float)
		self._last_psi_los = initial_psi
		self._unwrapped_psi_los = initial_psi
		self.logger = logger if logger else rclpy.logging.get_logger('ReferenceModel')


	def _unwrap_angle(self, current_psi_los):
		diff = wrap_angle(current_psi_los - self._last_psi_los)
		self._unwrapped_psi_los += diff
		self._last_psi_los = current_psi_los
		return self._unwrapped_psi_los

	def _state_derivative(self, state, unwrapped_target):
		psi_d, r_d, rd_dot = state
		d_psi_d = r_d
		d_r_d = rd_dot
		d_rd_dot = (self.omega_n_cb * unwrapped_target -
					  self.omega_n_cb * psi_d -
					  3 * self.omega_n_sq * r_d -
					  3 * self.omega_n * rd_dot)
		return np.array([d_psi_d, d_r_d, d_rd_dot])

	def update(self, psi_los, dt):
		if dt <= 0:
		   self.logger.warn("ReferenceModel update called with dt <= 0. Skipping.")
		   return self.state[0], self.state[1], self.state[2] # Return current state
		unwrapped_target = self._unwrap_angle(psi_los)
		state_dot = self._state_derivative(self.state, unwrapped_target)
		self.state += state_dot * dt
		return self.state[0], self.state[1], self.state[2]

	def get_state(self):
		return self.state[0], self.state[1], self.state[2]

	def get_filtered_angle_wrapped(self):
		return wrap_angle(self.state[0])

class WaypointFollowerActionServer(Node):

	def __init__(self):
		super().__init__('waypoint_follower_action_server')
		self._action_server = ActionServer(
			self,
			FollowWaypoints,
			'follow_waypoints',
			execute_callback=self.execute_callback,
			goal_callback=self.goal_callback,
			handle_accepted_callback=self.handle_accepted_callback,
			cancel_callback=self.cancel_callback,
			callback_group=ReentrantCallbackGroup() # Allows concurrent goal handling / callbacks
			)
		self.get_logger().info('Waypoint Follower Action Server Ready.')

		self.declare_parameter('ship_length', 10.0)
		self.declare_parameter('lookahead_multiplier', 3.0) # n_Lpp
		self.declare_parameter('acceptance_radius_multiplier', 1.0) # Multiplier for ship_length
		self.declare_parameter('ref_model_omega_n', 3.0)
		self.declare_parameter('simulation_dt', 0.1)
		self.declare_parameter('timeout_factor', 1.5) # Factor * estimated_time for timeout
		self.declare_parameter('frame_id', 'map') # Coordinate frame for publishers

		# === Publishers ===
		# Define a latching QoS profile for the path (so RViz can see it anytime)
		latching_qos = QoSProfile(
			depth=1,
			durability=DurabilityPolicy.TRANSIENT_LOCAL, # Latching behavior
			reliability=ReliabilityPolicy.RELIABLE,
			history=HistoryPolicy.KEEP_LAST
		)
		# Publisher for the planned path
		self.path_publisher_ = self.create_publisher(
			Path,
			'planned_path',
			qos_profile=latching_qos # Use latching QoS
		)
		# Publisher for the current robot pose
		self.pose_publisher_ = self.create_publisher(
			PoseStamped,
			'current_pose',
			10 # Standard QoS for frequently updated pose
		)

		# Store active goal handle 
		self._goal_handle = None
		self.frame_id_ = self.get_parameter('frame_id').get_parameter_value().string_value # Store frame_id

	def goal_callback(self, goal_request):
		"""Accept or reject a client request to begin an action."""
		self.get_logger().info('Received goal request')
		if len(goal_request.waypoints) < 2:
			 self.get_logger().warn('Rejecting goal: Not enough waypoints provided (minimum 2).')
			 return GoalResponse.REJECT
		# # Example Policy: Reject new goal if already active
		# if self._goal_handle is not None and self._goal_handle.is_active:
		#      self.get_logger().warn('Rejecting goal: Another goal is already active.')
		#      return GoalResponse.REJECT

		return GoalResponse.ACCEPT

	def handle_accepted_callback(self, goal_handle):
		"""Handles accepted goals."""
		if self._goal_handle is not None and self._goal_handle.is_active:
			self.get_logger().info('Previous goal still active, aborting it.')
			# Replace the handle. The old execute loop should exit.
			self._goal_handle.abort() # Signal the old one to stop

		self._goal_handle = goal_handle
		self.get_logger().info('Starting execution for new goal...')
		# Start execution in a background task 
		goal_handle.execute()

	def cancel_callback(self, goal_handle):
		"""Accept or reject a client request to cancel an action."""
		self.get_logger().info('Received cancel request.')
		return CancelResponse.ACCEPT # Accept all cancellations

	async def execute_callback(self, goal_handle):
		"""Executes the action's main logic."""
		start_node_time = self.get_clock().now()
		
		goal_id_str = binascii.hexlify(bytes(goal_handle.goal_id.uuid)).decode('utf-8')
		self.get_logger().info(f'Executing goal {goal_id_str}...')

		# --- Extract Goal & Parameters ---
		ros_waypoints = goal_handle.request.waypoints
		waypoints = [np.array([wp.x, wp.y]) for wp in ros_waypoints]
		u_d = goal_handle.request.linear_speed
		ship_length = self.get_parameter('ship_length').get_parameter_value().double_value
		n_Lpp = self.get_parameter('lookahead_multiplier').get_parameter_value().double_value
		acceptance_radius = self.get_parameter('acceptance_radius_multiplier').get_parameter_value().double_value * ship_length
		omega_n_filter = self.get_parameter('ref_model_omega_n').get_parameter_value().double_value
		dt = self.get_parameter('simulation_dt').get_parameter_value().double_value
		timeout_factor = self.get_parameter('timeout_factor').get_parameter_value().double_value
		self.frame_id_ = self.get_parameter('frame_id').get_parameter_value().string_value # Update frame_id

		# --- Publish Path ---
		path_msg = Path()
		path_msg.header.stamp = self.get_clock().now().to_msg()
		path_msg.header.frame_id = self.frame_id_
		for wp in waypoints:
			pose_stamped = PoseStamped()
			pose_stamped.header = path_msg.header # Use same timestamp/frame
			pose_stamped.pose.position.x = wp[0]
			pose_stamped.pose.position.y = wp[1]
			pose_stamped.pose.position.z = 0.0 # Assume 2D
			# Set default orientation (facing forward along X in its own frame)
			pose_stamped.pose.orientation = euler_to_quaternion(0.0)
			path_msg.poses.append(pose_stamped)
		self.path_publisher_.publish(path_msg)
		self.get_logger().info(f"Published planned path with {len(path_msg.poses)} waypoints to topic 'planned_path'.")

		# --- Initialization ---
		ship_pos = waypoints[0].copy()
		initial_psi = np.arctan2(waypoints[1][1] - ship_pos[1], waypoints[1][0] - ship_pos[0])
		ship_psi = initial_psi
		ship_nu = np.array([0.0, 0.0, 0.0], dtype=float)
		current_wp_index = 1
		active_los_angle = ship_psi
		ref_model = ReferenceModel(initial_psi=ship_psi, omega_n=omega_n_filter, logger=self.get_logger())
		feedback_msg = FollowWaypoints.Feedback()
		result = FollowWaypoints.Result()
		total_path_length = sum(np.linalg.norm(waypoints[i] - waypoints[i-1]) for i in range(1, len(waypoints)))
		estimated_time = (total_path_length / u_d) if u_d > 0.1 else 600.0
		max_sim_time = estimated_time * timeout_factor
		self.get_logger().info(f"Estimated path length: {total_path_length:.2f}m, Estimated time: {estimated_time:.2f}s, Timeout: {max_sim_time:.2f}s")
		sim_time = 0.0
		shortest_distance = np.inf

		# --- Simulation Loop ---
		while rclpy.ok():

			# Check if the goal has been aborted externally (e.g., by preemption)
			# This is crucial for stopping execution when preempted.
			if goal_handle.status == GoalStatus.STATUS_ABORTED:
				self.get_logger().info(f'Goal {goal_id_str} was aborted externally. Stopping execution.')
				# Result indicating failure - the state is already ABORTED
				result.success = False
				result.elapsed_time = (self.get_clock().now() - start_node_time).to_msg()
				return result # Exit the callback cleanly
		   
			# --- Check for Cancellation ---
			if goal_handle.is_cancel_requested:
				# Assuming cancel_callback accepted, server transitions state.
				# We mark it canceled from here.
				goal_handle.canceled()
				result.success = False
				result.elapsed_time = (self.get_clock().now() - start_node_time).to_msg()
				self.get_logger().info(f'Goal {goal_id_str} canceled.')
				return result # Exit the callback cleanly

			# --- Check for Timeout ---
			if sim_time > max_sim_time:
				goal_handle.abort() # Abort initiated from within execution
				result.success = False
				result.elapsed_time = (self.get_clock().now() - start_node_time).to_msg()
				self.get_logger().error(f'Goal {goal_id_str} aborted due to timeout ({max_sim_time:.1f}s).')
				return result # Exit the callback cleanly

			loop_start_time = time.monotonic() # Use monotonic clock for dt calculation consistency

			# --- Waypoint Management ---
			if current_wp_index >= len(waypoints):
				# Check status again before declaring success
				if goal_handle.status == GoalStatus.STATUS_EXECUTING:
					self.get_logger().info(f"End of path reached for goal {goal_id_str} at t={sim_time:.1f}s.")
					ship_nu = np.array([0.0, 0.0, 0.0]) # Stop the "robot"
					# ... (publish final pose) ...
					goal_handle.succeed()
					result.success = True
					result.elapsed_time = (self.get_clock().now() - start_node_time).to_msg()
					self.get_logger().info(f'Goal {goal_id_str} succeeded!')
					
				else:
					# Goal state changed before we could succeed (e.g., aborted/canceled very late)
					self.get_logger().warn(f"Goal {goal_id_str} finished path but final status is {goal_handle.status}. Not marking as succeeded.")
					result.success = False
					result.elapsed_time = (self.get_clock().now() - start_node_time).to_msg()
					
				
				# Update feedback one last time before exiting
				feedback_msg.current_pose.position.x = ship_pos[0]
				feedback_msg.current_pose.position.y = ship_pos[1]
				feedback_msg.current_pose.position.z = 0.0 # Assume 2D
				feedback_msg.current_pose.orientation = euler_to_quaternion(ship_psi)
				feedback_msg.current_velocity.linear.x = ship_nu[0]
				feedback_msg.current_velocity.linear.y = ship_nu[1]
				feedback_msg.current_velocity.angular.z = ship_nu[2]
				feedback_msg.current_waypoint_index = current_wp_index -1 # Last completed index
				goal_handle.publish_feedback(feedback_msg)

				# Publish final pose before exiting
				final_pose_msg = PoseStamped()
				final_pose_msg.header.stamp = self.get_clock().now().to_msg()
				final_pose_msg.header.frame_id = self.frame_id_
				final_pose_msg.pose.position.x = ship_pos[0]
				final_pose_msg.pose.position.y = ship_pos[1]
				final_pose_msg.pose.position.z = 0.0
				final_pose_msg.pose.orientation = euler_to_quaternion(ship_psi)
				self.pose_publisher_.publish(final_pose_msg)
				
				return result

			prev_wp = waypoints[current_wp_index - 1]
			current_wp = waypoints[current_wp_index]
			# Determine next_wp for LOS calculation lookahead, handle end of path
			next_wp_index = min(current_wp_index + 1, len(waypoints) - 1)
			next_wp = waypoints[next_wp_index]


			if check_waypoint_acceptance(ship_pos, current_wp, acceptance_radius):
				self.get_logger().info(f"Time {sim_time:.1f}s: Reached waypoint {current_wp_index}")
				current_wp_index += 1
				shortest_distance = np.inf # Reset shortest distance for the new segment
				# Skip rest of loop iteration to start fresh with the new waypoint index
				continue

			# --- LOS Calculation ---
			los_point, shortest_distance, surge_vel_gain = calculate_los_point(
				ship_pos, prev_wp, current_wp, next_wp, shortest_distance, n_Lpp, ship_length, self.get_logger()
				)

			calculated_psi_los = None
			if los_point is not None:
				calculated_psi_los = calculate_los_angle(ship_pos, los_point)
				if calculated_psi_los is not None:
					active_los_angle = calculated_psi_los
				else:
					self.get_logger().warn("Failed to calculate LOS angle, maintaining previous heading reference.")
					# Keep active_los_angle as is
			else:
				 self.get_logger().warn("Failed to calculate LOS point, maintaining previous heading reference.")
				 # Keep active_los_angle as is


			# --- Reference Signal Generation ---
			# Ensure dt is positive before updating reference model
			effective_dt = dt # Use the configured simulation step time
			if effective_dt > 0:
				 psi_d_unwrapped, r_d, rd_dot = ref_model.update(active_los_angle, effective_dt)
			else:
				 psi_d_unwrapped, r_d, rd_dot = ref_model.get_state() # Get current state if dt=0


			# --- Simple Kinematic "Control" ---
			# In a real robot, publish Twist commands here instead of simulating kinematics
			ship_nu[0] = u_d * surge_vel_gain  # Apply speed gain
			ship_nu[1] = 0.0                   # Zero sway speed (assumption)
			ship_nu[2] = r_d                   # Directly set yaw rate to reference yaw rate

			# --- Ship Kinematic Update (Simulation) ---
			# This section simulates the robot's movement based on commanded velocities.
			# In a real robot, this update would come from odometry/sensors.
			psi = ship_psi
			u, v, r = ship_nu
			x_dot = u * np.cos(psi) - v * np.sin(psi)
			y_dot = u * np.sin(psi) + v * np.cos(psi)
			psi_dot = r

			ship_pos[0] += x_dot * effective_dt
			ship_pos[1] += y_dot * effective_dt
			ship_psi += psi_dot * effective_dt
			ship_psi = wrap_angle(ship_psi) # Keep psi within [-pi, pi]

			# --- Publish Current Pose ---
			pose_msg = PoseStamped()
			pose_msg.header.stamp = self.get_clock().now().to_msg()
			pose_msg.header.frame_id = self.frame_id_
			pose_msg.pose.position.x = ship_pos[0]
			pose_msg.pose.position.y = ship_pos[1]
			pose_msg.pose.position.z = 0.0 # Assume 2D
			pose_msg.pose.orientation = euler_to_quaternion(ship_psi)
			self.pose_publisher_.publish(pose_msg)

			# --- Publish Feedback ---
			feedback_msg.current_pose.position.x = ship_pos[0]
			feedback_msg.current_pose.position.y = ship_pos[1]
			feedback_msg.current_pose.position.z = 0.0 # Assume 2D
			feedback_msg.current_pose.orientation = euler_to_quaternion(ship_psi)
			feedback_msg.current_velocity.linear.x = ship_nu[0]
			feedback_msg.current_velocity.linear.y = ship_nu[1] # v
			feedback_msg.current_velocity.angular.z = ship_nu[2] # r
			feedback_msg.current_waypoint_index = current_wp_index
			goal_handle.publish_feedback(feedback_msg)
			# self.get_logger().debug(f"Sim Time: {sim_time:.1f}, Pos: ({ship_pos[0]:.1f}, {ship_pos[1]:.1f}), Psi: {math.degrees(ship_psi):.1f}, Vel: ({ship_nu[0]:.1f}, {ship_nu[2]:.2f})")


			# --- Timing Control ---
			loop_end_time = time.monotonic()
			computation_time = loop_end_time - loop_start_time
			sleep_duration = max(0, dt - computation_time)
			time.sleep(sleep_duration) 
			# Increment simulation time
			sim_time += dt # Increment by fixed simulation step

		# Should not reach here if rclpy.ok() is false, but as a fallback:
		goal_handle.abort()
		result.success = False
		result.elapsed_time = (self.get_clock().now() - start_node_time).to_msg()
		self.get_logger().warn('Loop exited unexpectedly (rclpy not ok?). Goal aborted.')
		return result


def main(args=None):
	rclpy.init(args=args)
	try:
		waypoint_follower_server = WaypointFollowerActionServer()
		# Use MultiThreadedExecutor to handle callbacks concurrently
		executor = MultiThreadedExecutor()
		rclpy.spin(waypoint_follower_server, executor=executor)
	except KeyboardInterrupt:
		pass
	except Exception as e:
		 rclpy.logging.get_logger('main').error(f"Unhandled exception: {e}", exc_info=True)
	finally:
		if rclpy.ok():
			 # Make sure to destroy the node explicitly on exit
			 if 'waypoint_follower_server' in locals() and waypoint_follower_server:
				   waypoint_follower_server.destroy_node()
			 rclpy.shutdown()

if __name__ == '__main__':
	main()