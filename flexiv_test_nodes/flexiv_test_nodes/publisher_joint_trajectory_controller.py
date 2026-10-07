# Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rcl_interfaces.msg import ParameterDescriptor


class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__("publisher_position_trajectory_controller")
        # Declare all parameters
        self.declare_parameter("controller_name", "joint_trajectory_controller")
        self.declare_parameter("wait_sec_between_publish", 6)
        self.declare_parameter("goal_duration_sec", 1.0)
        self.declare_parameter("goal_names", ["pos1", "pos2"])
        self.declare_parameter("joints", [""])
        self.declare_parameter("check_starting_point", False)
        self.declare_parameter("starting_point_limits", False)
        self.declare_parameter("sine_sweep", False)
        self.declare_parameter("sine_amplitude_rad", 0.035)
        self.declare_parameter("sine_frequency_hz", 0.3)
        self.declare_parameter("sine_cycles", 1)
        self.declare_parameter("sine_sample_period_sec", 0.05)

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        self.goal_duration_sec = float(self.get_parameter("goal_duration_sec").value)
        goal_names = self.get_parameter("goal_names").value
        self.joints = self.get_parameter("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value
        self.starting_point = {}
        self.sine_sweep = self.get_parameter("sine_sweep").value
        self.sine_amplitude_rad = float(self.get_parameter("sine_amplitude_rad").value)
        self.sine_frequency_hz = float(self.get_parameter("sine_frequency_hz").value)
        self.sine_cycles = int(self.get_parameter("sine_cycles").value)
        self.sine_sample_period_sec = float(
            self.get_parameter("sine_sample_period_sec").value
        )
        if self.sine_sweep:
            self.check_sine_sweep(wait_sec_between_publish)

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        # starting point stuff
        if self.check_starting_point:
            # declare nested params
            for name in self.joints:
                param_name_tmp = "starting_point_limits" + "." + name
                self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])
                self.starting_point[name] = self.get_parameter(param_name_tmp).value

            for name in self.joints:
                if len(self.starting_point[name]) != 2:
                    raise Exception('"starting_point" parameter is not set correctly!')
            self.joint_state_sub = self.create_subscription(
                JointState, "joint_states", self.joint_state_callback, 10
            )
        # initialize starting point status
        self.starting_point_ok = not self.check_starting_point

        self.joint_state_msg_received = False

        # Read all positions from parameters
        self.goals = []
        for name in goal_names:
            self.declare_parameter(
                name, descriptor=ParameterDescriptor(dynamic_typing=True)
            )
            goal = self.get_parameter(name).value
            if goal is None or len(goal) == 0:
                raise Exception(f'Values for goal "{name}" not set!')

            float_goal = []
            for value in goal:
                float_goal.append(float(value))
            self.goals.append(float_goal)

        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.get_logger().info(
            'Publishing {} goals on topic "{}" every {} s'.format(
                len(goal_names), publish_topic, wait_sec_between_publish
            )
        )

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        if self.starting_point_ok:
            traj = JointTrajectory()
            traj.joint_names = self.joints
            point = JointTrajectoryPoint()
            point.positions = self.goals[self.i]
            point.time_from_start = _duration(self.goal_duration_sec)

            traj.points.append(point)
            if self.sine_sweep:
                traj.points.extend(self.sine_sweep_points(self.goals[self.i]))
            self.publisher_.publish(traj)

            self.i += 1
            self.i %= len(self.goals)

        elif self.check_starting_point and not self.joint_state_msg_received:
            self.get_logger().warn(
                'Start configuration could not be checked! Check "joint_state" topic!'
            )
        else:
            self.get_logger().warn(
                "Start configuration is not within configured limits!"
            )

    def joint_state_callback(self, msg):
        if not self.joint_state_msg_received:
            # check start state
            limit_exceeded = [False] * len(msg.name)
            for idx, enum in enumerate(msg.name):
                if enum not in self.starting_point:
                    continue
                if (msg.position[idx] < self.starting_point[enum][0]) or (
                    msg.position[idx] > self.starting_point[enum][1]
                ):
                    self.get_logger().warn(
                        f"Starting point limits exceeded for joint {enum} !"
                    )
                    limit_exceeded[idx] = True

            if any(limit_exceeded):
                self.starting_point_ok = False
            else:
                self.starting_point_ok = True

            self.joint_state_msg_received = True
        else:
            return

    def check_sine_sweep(self, wait_sec_between_publish):
        if self.sine_frequency_hz <= 0.0 or self.sine_sample_period_sec <= 0.0:
            raise Exception(
                '"sine_frequency_hz" and "sine_sample_period_sec" must be positive!'
            )
        sweep_duration_sec = (
            self.goal_duration_sec + self.sine_cycles / self.sine_frequency_hz
        )
        if wait_sec_between_publish >= sweep_duration_sec:
            return
        self.get_logger().warn(
            f"Each sine sweep takes {sweep_duration_sec:.1f} s but a new one"
            f" is published every {wait_sec_between_publish} s;"
            " the sweeps will interrupt each other."
        )

    def sine_sweep_points(self, center):
        """Sweep every joint by 1 - cos around `center`, from rest back to rest."""
        omega = 2.0 * math.pi * self.sine_frequency_hz
        sweep_duration_sec = self.sine_cycles / self.sine_frequency_hz
        steps = max(1, round(sweep_duration_sec / self.sine_sample_period_sec))
        points = []
        for k in range(1, steps + 1):
            t = k * sweep_duration_sec / steps
            phase = omega * t
            offset = self.sine_amplitude_rad * (1.0 - math.cos(phase))
            point = JointTrajectoryPoint()
            point.positions = [q + offset for q in center]
            point.velocities = [
                self.sine_amplitude_rad * omega * math.sin(phase)
            ] * len(center)
            point.accelerations = [
                self.sine_amplitude_rad * omega * omega * math.cos(phase)
            ] * len(center)
            point.time_from_start = _duration(self.goal_duration_sec + t)
            points.append(point)
        return points


def _duration(sec):
    return Duration(sec=int(sec), nanosec=int((sec % 1.0) * 1e9))


def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
