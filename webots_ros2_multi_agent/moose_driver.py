# Copyright 1996-2023 Cyberbotics Ltd.
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

"""ROS2 Mavic 2 Pro driver."""

import math
import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class MooseDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Sensors
        self.__gps = self.__robot.getDevice('gps')
        # self.__gyro = self.__robot.getDevice('gyro')
        # self.__imu = self.__robot.getDevice('inertial unit')

        # Motors
        self.__motors = [
            self.__robot.getDevice('left motor 1'),
            self.__robot.getDevice('left motor 2'),
            self.__robot.getDevice('left motor 3'),
            self.__robot.getDevice('left motor 4'),
            self.__robot.getDevice('right motor 1'),
            self.__robot.getDevice('right motor 2'),
            self.__robot.getDevice('right motor 3'),
            self.__robot.getDevice('right motor 4')
        ]

        for motor in self.__motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0)

        # State
        self.__target_twist = Twist()

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node(self.__robot.getName() + '_driver')
        self.__node.create_subscription(Twist, self.__robot.getName() + '/cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
 
        # Apply control
        self.__motors[0].setVelocity(command_motor_left)
        self.__motors[1].setVelocity(command_motor_left)
        self.__motors[2].setVelocity(command_motor_left)
        self.__motors[3].setVelocity(command_motor_left)

        self.__motors[4].setVelocity(command_motor_right)
        self.__motors[5].setVelocity(command_motor_right)
        self.__motors[6].setVelocity(command_motor_right)
        self.__motors[7].setVelocity(command_motor_right)
