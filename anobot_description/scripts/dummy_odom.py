#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class DummyOdom(Node):
    def __init__(self):
        super().__init__('dummy_odom_node')
        
        # Current simulated position of the AGV
        self.x = 0.5
        self.y = 0.0
        self.yaw = 0.0
        
        # Velocity storage
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        
        # Transform Broadcaster to feed RViz
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to the keyboard teleop commands
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Run the kinematic update loop at 50Hz (every 0.02 seconds)
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.update_position)

    def cmd_vel_callback(self, msg):
        # Save velocities from keyboard
        self.vx = msg.linear.x
        self.vy = msg.linear.y # Tank drive won't use this, but good to have
        self.wz = msg.angular.z

    def update_position(self):
        # 1. Simple Tank/Skid-Steer Kinematics integration over time
        dt = self.timer_period
        
        # Calculate changes based on current orientation (Yaw)
        self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        self.y += (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        self.yaw += self.wz * dt

        # 2. Build the TF frame message for RViz
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Convert Euler Yaw angle to Quaternion for ROS2 TF
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)

        # Broadcast the coordinate shift to RViz live!
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DummyOdom()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
