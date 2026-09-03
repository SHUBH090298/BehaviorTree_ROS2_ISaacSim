#!/usr/bin/env python3
"""Stub skill agent.

Stands in for a motion-planning / manipulation skill executor. Accepts a
named skill, reports running -> success|failure, and honours cancellation.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class SkillAgent(Node):
    def __init__(self):
        super().__init__("skill_agent")
        self.declare_parameter("skill_duration", 3.0)
        self.declare_parameter("fail_after", 1.2)
        self.duration = self.get_parameter("skill_duration").value
        self.fail_after = self.get_parameter("fail_after").value

        self.state = "idle"
        self.elapsed = 0.0
        self.fault = False

        self.status_pub = self.create_publisher(String, "/skill_status", 10)
        self.hb_pub = self.create_publisher(String, "/agent/skill/heartbeat", 10)
        self.create_subscription(String, "/skill_request", self._on_request, 10)
        self.create_subscription(Bool, "/inject_fault", self._on_fault, 10)

        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._heartbeat)

        self.get_logger().info("skill_agent up")

    def _on_fault(self, msg: Bool):
        self.fault = msg.data
        self.get_logger().warn(f"fault injection {'ARMED' if msg.data else 'cleared'}")

    def _on_request(self, msg: String):
        if msg.data == "cancel":
            if self.state == "running":
                self.get_logger().warn("skill cancelled by orchestrator")
            self.state = "idle"
            self.elapsed = 0.0
            return
        self.state = "running"
        self.elapsed = 0.0
        self.get_logger().info(f"executing skill '{msg.data}'")

    def _tick(self):
        if self.state == "running":
            self.elapsed += 0.1
            if self.fault and self.elapsed >= self.fail_after:
                self.state = "failure"
                self.get_logger().error("skill FAILED")
            elif self.elapsed >= self.duration:
                self.state = "success"
                self.get_logger().info("skill succeeded")
        self.status_pub.publish(String(data=self.state))

    def _heartbeat(self):
        self.hb_pub.publish(String(data="skill"))


def main():
    rclpy.init()
    node = SkillAgent()
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
