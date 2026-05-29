#!/usr/bin/env python3
import math
import os
import time

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import numpy
from enum import Enum
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

from turtlebot3_msgs.srv import Dqn, Goal


ROS_DISTRO = os.environ.get('ROS_DISTRO')
class CurrentGoal(Enum):
    BOOST = 0
    GOAL = 1
    IDLE = -1

boost_multiplier = 2.0

class RLEnvironment(Node):

    def __init__(self):
        super().__init__('rl_environment')
        # Decision
        self.current_goal = CurrentGoal.IDLE
        self.velocity_multiplier = 1.0

        # Pose state
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.boost_pose_x = 0.0
        self.boost_pose_y = 0.0
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0
        self.robot_pose_theta = 0.0

        # Derived state
        self.goal_angle = 0.0
        self.boost_angle = 0.0
        self.goal_distance = 1.0
        self.boost_distance = 1.0
        self.min_obstacle_distance = 10.0
        self.scan_ranges = []
        self.front_ranges = []
        self.front_angles = []

        # Lifecycle flags
        self._goal_initialized = False
        self._have_odom = False
        self._busy = False   # prevents overlapping service calls
        self.done = False
        self.succeed = False
        self.fail = False

        # RL leftovers we don't really use now but keep for compatibility
        self.action_size = 5
        self.max_step = 800
        self.local_step = 0
        self.angular_vel = [1.5, 0.75, 0.0, -0.75, -1.5]
        self.init_goal_distance = 0.5
        self.stop_cmd_vel_timer = None

        qos = QoSProfile(depth=10)

        # Publishers
        if ROS_DISTRO == 'humble':
            self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        else:
            self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', qos)
        

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_sub_callback, qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_sub_callback, qos_profile_sensor_data
        )

        # Service clients
        self.clients_callback_group = MutuallyExclusiveCallbackGroup()
        self.task_succeed_client = self.create_client(
            Goal, 'task_succeed', callback_group=self.clients_callback_group
        )
        self.task_failed_client = self.create_client(
            Goal, 'task_failed', callback_group=self.clients_callback_group
        )
        self.initialize_environment_client = self.create_client(
            Goal, 'initialize_env', callback_group=self.clients_callback_group
        )

        # In __init__, replace the three service server definitions with:
        self.services_cbg = MutuallyExclusiveCallbackGroup()
        self.rl_agent_interface_service = self.create_service(
            Dqn, 'rl_agent_interface', self.rl_agent_interface_callback,
            callback_group=self.services_cbg
        )
        self.make_environment_service = self.create_service(
            Empty, 'make_environment', self.make_environment_callback
        )
        self.reset_environment_service = self.create_service(
            Dqn, 'reset_environment', self.reset_environment_callback
        )

        # Auto-initialize the goal (no need to call make_environment manually)
        self._init_timer = self.create_timer(1.0, self._try_initialize)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self._log_counter = 0

    # ---------- Goal initialization (non-blocking) ----------
    def _try_initialize(self):
        if self._goal_initialized or self._busy:
            return
        if not self.initialize_environment_client.service_is_ready():
            self.get_logger().info('Waiting for initialize_env service...')
            return
        self._busy = True
        self.get_logger().info('Initializing goal...')
        future = self.initialize_environment_client.call_async(Goal.Request())
        future.add_done_callback(self._init_done)

    def _init_done(self, future):
        self._busy = False
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'init failed: {e}')
            return
        if result and result.success:
            self.goal_pose_x = result.pose_x
            self.goal_pose_y = result.pose_y
            self.boost_pose_x = result.boost_pose_x
            self.boost_pose_y = result.boost_pose_y
            self._goal_initialized = True
            self._init_timer.cancel()
            self.get_logger().info(
                f'Goal initialized at ({self.goal_pose_x:.2f}, {self.goal_pose_y:.2f})'
            )
            self.get_logger().info(
                f'Boost initialized at ({self.boost_pose_x:.2f}, {self.boost_pose_y:.2f})'
            )

    # ---------- Async wrappers for task_succeed / task_failed ----------
    def _request_new_goal(self, client, label):
        if self._busy:
            return
        if not client.service_is_ready():
            return
        self._busy = True
        self.get_logger().info(f'Requesting {label}...')
        future = client.call_async(Goal.Request())
        future.add_done_callback(lambda f: self._goal_update_done(f, label))

    def _goal_update_done(self, future, label):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'{label} failed: {e}')
            self._busy = False
            return

        # Save OLD goal position before overwriting
        old_goal_x = self.goal_pose_x
        old_goal_y = self.goal_pose_y

        if result is not None:
            self.goal_pose_x = result.pose_x
            self.goal_pose_y = result.pose_y
            self.boost_pose_x = result.boost_pose_x
            self.boost_pose_y = result.boost_pose_y
            ...

        self.current_goal = CurrentGoal.IDLE
        self.done = False
        self.succeed = False
        self.fail = False
        self.local_step = 0

        # Wait until robot has moved away from the OLD goal
        reset_ok = False
        timeout = time.time() + 3.0
        while time.time() < timeout:
            dist_from_old_goal = math.hypot(
                self.robot_pose_x - old_goal_x,
                self.robot_pose_y - old_goal_y
            )
            if dist_from_old_goal > 0.5:
                reset_ok = True
                break
            time.sleep(0.05)

        if not reset_ok:
            self.get_logger().warn('Timeout waiting for robot to reset position')

        self._busy = False

    # ---------- Sensor callbacks ----------
    def scan_sub_callback(self, scan):
        self.scan_ranges = []
        self.front_ranges = []
        self.front_angles = []
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        for i, distance in enumerate(scan.ranges):
            angle = angle_min + i * angle_increment
            if distance == float('Inf'):
                distance = 3.5
            elif numpy.isnan(distance):
                distance = 3.5  # treat NaN as far, not zero!
            self.scan_ranges.append(distance)
            if (0 <= angle <= math.pi / 2) or (3 * math.pi / 2 <= angle <= 2 * math.pi):
                self.front_ranges.append(distance)
                self.front_angles.append(angle)
        self.min_obstacle_distance = min(self.scan_ranges) if self.scan_ranges else 10.0

    def odom_sub_callback(self, msg):
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y
        _, _, self.robot_pose_theta = self.euler_from_quaternion(
            msg.pose.pose.orientation
        )

        dx = self.goal_pose_x - self.robot_pose_x
        dy = self.goal_pose_y - self.robot_pose_y
        dx_boost = self.boost_pose_x - self.robot_pose_x
        dy_boost = self.boost_pose_y - self.robot_pose_y
        self.goal_distance = math.hypot(dx, dy)
        self.boost_distance = math.hypot(dx_boost,dy_boost)

        path_theta = math.atan2(dy, dx)
        path_theta_boost = math.atan2(dy_boost, dx_boost)
        angle = path_theta - self.robot_pose_theta + math.pi
        angle_boost = path_theta_boost - self.robot_pose_theta + math.pi
        # Wrap to [-pi, pi]
        self.goal_angle = math.atan2(math.sin(angle), math.cos(angle))
        self.boost_angle = math.atan2(math.sin(angle_boost), math.cos(angle_boost))
        self._have_odom = True

    # ---------- Control loop ----------
    def controller(self, r_dist, r_angle):
        k_linear = 0.5
        k_angular = 1.5
        std_linear = 0.11
        max_angular = 2.0
        align_threshold = 0.2  # rad — must be well aligned before moving

        angle_factor = max(0.0, 1.0 - abs(r_angle) / align_threshold)

        v = min(k_linear * r_dist, std_linear) * angle_factor
        omega = k_angular * r_angle
        omega = max(-max_angular, min(max_angular, omega))

        return v, omega

    def control_loop(self):
        # Log every 10th tick to reduce spam
        self._log_counter += 1
        if self._log_counter % 10 == 0:
            self.get_logger().info(
                f'goal=({self.goal_pose_x:.2f},{self.goal_pose_y:.2f}) '
                f'goal dist={self.goal_distance:.2f} ang={self.goal_angle:.2f} '
                f'boost=({self.boost_pose_x:.2f},{self.boost_pose_y:.2f}) '
                f'boost dist={self.boost_distance:.2f} ang={self.boost_angle:.2f} '
                f'min_obs={self.min_obstacle_distance:.2f} done={self.done}'
            )

        # Don't act until we know where we are AND where the goal is
        if not self._goal_initialized or not self._have_odom:
            return

        # Don't act while waiting for a new goal from a previous episode end
        if self._busy:
            self._publish_stop()
            return

        # Arrived?
        if self.goal_distance < 0.20 and self.current_goal==CurrentGoal.GOAL and not self.done:
            self.get_logger().info('Goal reached!')
            self.succeed = True
            self.done = True
            self.velocity_multiplier = 1.0
            self._publish_stop()
            self._request_new_goal(self.task_succeed_client, 'task_succeed')
            return

        if self.boost_distance < 0.20 and self.current_goal==CurrentGoal.BOOST and not self.done:
            self.get_logger().info('Boost reached!')
            self.velocity_multiplier = boost_multiplier
            self._publish_stop()
            self.current_goal = CurrentGoal.GOAL
            return

        # Collision?
        obstacle_is_boost = abs(self.min_obstacle_distance - self.boost_distance) < 0.15
        target_distance = self.boost_distance if self.current_goal == CurrentGoal.BOOST else self.goal_distance
        if self.min_obstacle_distance < 0.15 and target_distance > 0.25 and not self.done and not obstacle_is_boost:
            self.get_logger().info('Collision!')
            self.fail = True
            self.done = True
            self.velocity_multiplier = 1.0
            self._publish_stop()
            self._request_new_goal(self.task_failed_client, 'task_failed')
            return

        # Wait
        if self.done:
            self._publish_stop()
            return
        
        # Control the robot towards the current goal.
        if self.current_goal == CurrentGoal.BOOST:
            v,omega = self.controller(self.boost_distance,self.boost_angle)
        elif self.current_goal == CurrentGoal.GOAL:
            v,omega = self.controller(self.goal_distance,self.goal_angle)
        elif self.current_goal == CurrentGoal.IDLE:
            return
        self._publish_cmd(self.velocity_multiplier*v, omega)

    def _publish_cmd(self, v, omega):
        if ROS_DISTRO == 'humble':
            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(omega)
        else:
            msg = TwistStamped()
            msg.twist.linear.x = float(v)
            msg.twist.angular.z = float(omega)
        self.cmd_vel_pub.publish(msg)

    def _publish_stop(self):
        self._publish_cmd(0.0, 0.0)

    # ---------- Compatibility stubs (unchanged behavior for the RL path) ----------
    def make_environment_callback(self, request, response):
        # No-op: initialization happens automatically in _try_initialize.
        self.get_logger().info('make_environment called (no-op, auto-init in use)')
        return response
    
    def reset_environment_callback(self, request, response):
        while self._busy:
            time.sleep(0.02)
        self.current_goal = CurrentGoal.IDLE
        self.done = False
        self.succeed = False
        self.fail = False
        self.local_step = 0
        self.velocity_multiplier = 1.0
        response.state = [
            float(self.robot_pose_x), float(self.robot_pose_y),
            float(self.goal_pose_x), float(self.goal_pose_y),
            float(self.boost_pose_x), float(self.boost_pose_y)
        ]
        return response

    def rl_agent_interface_callback(self, request, response):
        action = request.action

        # Calculate optimal action
        d_goal = math.hypot(self.goal_pose_x - self.robot_pose_x,
                            self.goal_pose_y - self.robot_pose_y)
        d_boost = math.hypot(self.boost_pose_x - self.robot_pose_x,
                            self.boost_pose_y - self.robot_pose_y)
        d_boost_to_goal = math.hypot(self.goal_pose_x - self.boost_pose_x,
                                    self.goal_pose_y - self.boost_pose_y)

        boost_is_optimal = (d_boost + d_boost_to_goal / boost_multiplier) < d_goal

        correct_action = 0 if boost_is_optimal else 1

        if action != correct_action:
            # Wrong decision — fail immediately without executing
            response.state = [
                float(self.robot_pose_x), float(self.robot_pose_y),
                float(self.goal_pose_x), float(self.goal_pose_y),
                float(self.boost_pose_x), float(self.boost_pose_y)
            ]
            response.reward = -100.0
            response.done = True
            return response

        # Correct decision — execute and wait for episode end
        if action == 0:
            self.current_goal = CurrentGoal.BOOST
        else:
            self.current_goal = CurrentGoal.GOAL
            self.velocity_multiplier = 1.0

        while not self.done:
            time.sleep(0.05)

        reward = 100.0 - self.local_step * 0.1
        response.state = [
            float(self.robot_pose_x), float(self.robot_pose_y),
            float(self.goal_pose_x), float(self.goal_pose_y),
            float(self.boost_pose_x), float(self.boost_pose_y)
        ]
        response.reward = reward
        response.done = True
        return response

    def euler_from_quaternion(self, quat):
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
        pitch = math.asin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = RLEnvironment()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()