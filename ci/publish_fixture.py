#!/usr/bin/env python3
"""Publish a small ROS 2 fixture for CI smoke tests.

Used by `.github/workflows/ros2.yml` Job B:
- `ros2 bag record` records `/ci/vel`, `/ci/imu`, `/ci/twist` into an MCAP
  bag, which `rosbag_loader::real_bag_loads_in_ci` then loads.
- `ros2_live::live_smoke_receives_data` subscribes to `/ci/vel` live.

Publishes at ~100 Hz; run until interrupted (the workflow backgrounds it and
kills it after the tests). Requires a sourced ROS 2 Jazzy env.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def main() -> None:
    rclpy.init()
    node = Node("ci_fixture_publisher")
    pub_vel = node.create_publisher(Float64, "/ci/vel", 10)
    pub_imu = node.create_publisher(Imu, "/ci/imu", 10)
    pub_twist = node.create_publisher(Twist, "/ci/twist", 10)

    t = 0.0
    while rclpy.ok():
        vel = Float64()
        vel.data = math.sin(2.0 * math.pi * 1.0 * t)
        pub_vel.publish(vel)

        imu = Imu()
        imu.linear_acceleration.x = 9.81 + 0.1 * math.sin(t)
        imu.angular_velocity.z = 0.5 * math.cos(t)
        pub_imu.publish(imu)

        twist = Twist()
        twist.linear.x = 1.0 + 0.2 * math.sin(t)
        pub_twist.publish(twist)

        t += 0.01
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.01)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
