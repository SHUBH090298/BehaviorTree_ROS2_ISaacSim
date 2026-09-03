#!/usr/bin/env python3
"""Stub perception agent (v1).

Publishes the same contract the YOLO agent will publish tomorrow:
  /part_detected  Bool
  /part_point     PointStamped   (target position in the robot base frame)
  heartbeat       String @ 2 Hz

The position is fixed. That is the only difference from v2.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PointStamped


class PerceptionAgent(Node):
    def __init__(self):
        super().__init__("perception_agent")
        self.declare_parameter("detect_delay", 1.5)
        self.declare_parameter("frame_id", "panda_link0")
        self.declare_parameter("x", 0.45)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.02)

        self.delay = self.get_parameter("detect_delay").value
        self.frame = self.get_parameter("frame_id").value
        self.xyz = (self.get_parameter("x").value,
                    self.get_parameter("y").value,
                    self.get_parameter("z").value)

        self.detected = False
        self.elapsed = 0.0

        self.det_pub = self.create_publisher(Bool, "/part_detected", 10)
        self.pt_pub = self.create_publisher(PointStamped, "/part_point", 10)
        self.hb_pub = self.create_publisher(String, "/agent/perception/heartbeat", 10)
        self.create_subscription(Bool, "/set_part", self._on_set_part, 10)

        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._heartbeat)
        self.get_logger().info(
            f"perception_agent v1 (dummy) up — fixed target {self.xyz}")

    def _on_set_part(self, msg: Bool):
        self.detected = msg.data
        self.elapsed = self.delay if msg.data else 0.0
        self.get_logger().info(f"part set to {msg.data}")

    def _tick(self):
        if not self.detected:
            self.elapsed += 0.1
            if self.elapsed >= self.delay:
                self.detected = True
                self.get_logger().info("part detected (dummy)")

        self.det_pub.publish(Bool(data=self.detected))

        if self.detected:
            p = PointStamped()
            p.header.stamp = self.get_clock().now().to_msg()
            p.header.frame_id = self.frame
            p.point.x, p.point.y, p.point.z = self.xyz
            self.pt_pub.publish(p)

    def _heartbeat(self):
        self.hb_pub.publish(String(data="perception"))


def main():
    rclpy.init()
    node = PerceptionAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
