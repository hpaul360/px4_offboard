###################################################################################
# MIT License
#
# Copyright (c) 2024 Hannibal Paul
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
###################################################################################

__author__ = "Hannibal Paul"

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleControlMode
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np

class OffboardSetpoint(Node):

    def __init__(self):
        super().__init__('offboard_setpoint')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers and Subscribers
        self.control_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.mode_sub = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.mode_callback, qos_profile)

        # Set publish rate timer
        pub_rate = 10.0  # Hz. Set rate of > 2Hz for OffboardControlMode 
        self.timer = self.create_timer(1/pub_rate, self.setpoint_callback)

        # Other parameters
        self.offboard_mode = False

        # Polygon parameters - set these according the requirements
        self.center_point = [0, 0] # Center of the polygon x,y in m
        self.circum_radius = 0.5 # in m
        self.n_sides = 4 # Number of sides of a polygon
        self.meter_per_sec = 0.5 # Estimated expected speed in m/s
        self.rotation_angle = 45 # in deg (Self note: Keep 45 for square.)

        # Automatically set
        self.speed_factor = 9.76 # Obtained by flight (trial and error)
        self.points_per_line = int(2*np.pi*self.circum_radius*self.speed_factor/(self.n_sides*self.meter_per_sec))
        self.counter = 0

    def mode_callback(self, msg):
        self.offboard_mode = msg.flag_control_offboard_enabled

    def polygon_path(self, center_point, n_sides, circum_radius, points_per_line, rotation_angle):
        # Get vertices
        theta = np.linspace(0, 2*np.pi, n_sides+1) + np.radians(rotation_angle)
        x = circum_radius * np.cos(theta) + center_point[0]
        y = circum_radius * np.sin(theta) + center_point[1]

        # Add points between vertices
        path_coordinates = []
        for i in range(n_sides):
            x_interp = np.linspace(x[i], x[i+1], points_per_line+2)[0:-1]
            y_interp = np.linspace(y[i], y[i+1], points_per_line+2)[0:-1]
            path_coordinates.extend(list(zip(x_interp, y_interp)))

        return path_coordinates

    def setpoint_callback(self):
        # info: https://docs.px4.io/main/en/flight_modes/offboard.html

        # Set and publish control flags
        control_mode = OffboardControlMode()
        # Timestamp is automatically set inside PX4
        control_mode.timestamp = 0
        # First field that has a non-zero value (from top to bottom)
        # defines what valid estimate is required
        control_mode.position = True
        control_mode.velocity = False
        control_mode.acceleration  = False
        control_mode.attitude = False
        control_mode.body_rate = False
        control_mode.thrust_and_torque = False
        control_mode.direct_actuator = False
        self.control_pub.publish(control_mode)
        
        # Get full polygon path
        path = self.polygon_path(self.center_point, self.n_sides, self.circum_radius, self.points_per_line, self.rotation_angle)

        # Start sending setpoints if in offboard mode
        if self.offboard_mode:
            # Trajectory setpoint - NED local world frame
            setpoint_traj = TrajectorySetpoint()
            # Timestamp is automatically set inside PX4
            setpoint_traj.timestamp = 0
            px, py = path[self.counter]
            pz = -2.0 # in m
            setpoint_traj.position = [px, py, pz] # [x, y, z] in meters
            #setpoint_traj.velocity # in m/s
            #setpoint_traj.acceleration # in m/s^2
            #setpoint_traj.jerk # m/s^3 (for logging only)
            setpoint_traj.yaw = 0.0*np.pi/180 # in rad
            #setpoint_traj.yawspeed = 0.0 # in rad/s

            # Publish
            self.setpoint_pub.publish(setpoint_traj)

            # Control the path points counter
            self.counter+=1
            if self.counter == len(path):
                self.counter = 0
        else:
            self.counter = 0


def main(args=None):
    rclpy.init(args=args)

    control = OffboardSetpoint()

    rclpy.spin(control)

    control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()