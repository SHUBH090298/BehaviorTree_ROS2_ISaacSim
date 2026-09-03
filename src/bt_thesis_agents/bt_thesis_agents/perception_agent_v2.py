#!/usr/bin/env python3
"""Perception agent v2 — YOLOv8 object detection.

Publishes exactly the contract of v1:
  /part_detected  Bool
  /part_point     PointStamped
  heartbeat       String @ 2 Hz
plus /perception/debug_image, a diagnostic the orchestrator does not consume.

Pixel -> world uses an affine map calibrated with three known points (see the
handbook, 3.1). The camera looks straight down at a flat surface, so the map is
exact up to measurement error.

Note: the debug image is built by hand rather than via cv_bridge's
cv2_to_imgmsg(). The venv's pip-installed opencv-python (5.0.0.93) numbers its
image type constants differently from the OpenCV that apt's cv_bridge
extension was compiled against, so cv2_to_imgmsg's internal lookup table
raises KeyError on the outgoing conversion (incoming imgmsg_to_cv2 is
unaffected and still used below). Since the array's format is already known
exactly (BGR, 3-channel, uint8), building the Image message directly sidesteps
the mismatch entirely rather than chasing another version pin.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np


class PerceptionAgentV2(Node):
    def __init__(self):
        super().__init__("perception_agent")     # SAME node name as v1

        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("target_class", "banana")
        self.declare_parameter("confidence", 0.40)
        self.declare_parameter("frame_id", "panda_link0")
        self.declare_parameter("table_z", 0.02)
        self.declare_parameter("lost_after", 0.5)   # s without a detection
        self.declare_parameter("publish_debug", True)
        # affine pixel -> world, from handbook 3.1
        for k, v in (("a", 0.00127259), ("b", -0.00000055), ("c", -0.81339097),
                     ("d", -0.00000331), ("e", -0.00129983 ), ("f", 0.47213187)):
            self.declare_parameter(k, v)

        g = lambda n: self.get_parameter(n).value
        self.target_class = g("target_class")
        self.conf_thresh  = g("confidence")
        self.frame        = g("frame_id")
        self.table_z      = g("table_z")
        self.lost_after   = g("lost_after")
        self.debug        = g("publish_debug")
        self.coef = tuple(g(k) for k in ("a", "b", "c", "d", "e", "f"))

        self.bridge = CvBridge()
        self.get_logger().info(f"loading {g('model')} …")
        self.model = YOLO(g("model"))
        self.get_logger().info("model ready")

        self.detected = False
        self.last_seen = self.get_clock().now()

        self.det_pub = self.create_publisher(Bool, "/part_detected", 10)
        self.pt_pub  = self.create_publisher(PointStamped, "/part_point", 10)
        self.hb_pub  = self.create_publisher(String, "/agent/perception/heartbeat", 10)
        self.dbg_pub = self.create_publisher(Image, "/perception/debug_image", 2)

        self.create_subscription(Image, "/rgb", self._on_image,
                                 qos_profile_sensor_data)
        self.create_timer(0.5, self._heartbeat)

        self.get_logger().info(
            f"perception_agent v2 (YOLO) up — looking for '{self.target_class}'")

    def _to_world(self, u, v):
        a, b, c, d, e, f = self.coef
        return (a * u + b * v + c, d * u + e * v + f)

    @staticmethod
    def _bgr8_to_imgmsg(img: np.ndarray, header) -> Image:
        """Build a bgr8 sensor_msgs/Image directly, bypassing cv_bridge's
        outgoing conversion (see module docstring for why)."""
        msg = Image()
        msg.header = header
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        return msg

    def _on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        res = self.model(frame, verbose=False)[0]

        best = None
        for box in res.boxes:
            if res.names[int(box.cls)] != self.target_class:
                continue
            conf = float(box.conf)
            if conf < self.conf_thresh:
                continue
            if best is None or conf > float(best.conf):
                best = box

        now = self.get_clock().now()

        if best is not None:
            x1, y1, x2, y2 = best.xyxy[0].tolist()
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            wx, wy = self._to_world(u, v)

            p = PointStamped()
            p.header.stamp = now.to_msg()
            p.header.frame_id = self.frame
            p.point.x, p.point.y, p.point.z = wx, wy, self.table_z
            self.pt_pub.publish(p)

            if not self.detected:
                self.get_logger().info(
                    f"{self.target_class} acquired at "
                    f"({wx:.3f}, {wy:.3f}) conf={float(best.conf):.2f}  "
                    f"[pixel u={u:.1f} v={v:.1f}]")
            self.detected = True
            self.last_seen = now
        else:
            # Hysteresis: a single dropped frame should not look like a fault.
            if self.detected and \
               (now - self.last_seen).nanoseconds / 1e9 > self.lost_after:
                self.detected = False
                self.get_logger().warn(f"{self.target_class} lost")

        self.det_pub.publish(Bool(data=self.detected))

        if self.debug:
            annotated = np.ascontiguousarray(res.plot()[:, :, :3], dtype=np.uint8)
            out = self._bgr8_to_imgmsg(annotated, msg.header)
            self.dbg_pub.publish(out)

    def _heartbeat(self):
        self.hb_pub.publish(String(data="perception"))


def main():
    rclpy.init()
    node = PerceptionAgentV2()
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