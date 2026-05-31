#!/usr/bin/env python3
"""
color_detector_node.py

Robust perception node for the TurtleBot3 Color Follower system.

Key engineering challenge solved:
    The house environment contains reddish-brown walls that create false
    positives with naive HSV filtering. This node rejects wall false-positives
    using two additional filters:

    1. Saturation threshold: walls are brownish-red (low saturation ~60-100).
       Real red targets are highly saturated (>150). Increasing the minimum
       saturation cleanly separates targets from background surfaces.

    2. Circularity filter: walls produce elongated contours (low circularity).
       A circular target scores close to 1.0. Only contours with
       circularity > 0.45 are accepted as valid targets.

Topics published:
    /color_follower/detected          std_msgs/Bool
    /color_follower/normalized_error  std_msgs/Float32  [-1.0, 1.0]
    /color_follower/target_area       std_msgs/Float32  [0.0, 1.0]
    /color_follower/debug_image       sensor_msgs/Image
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math

COLOR_RANGES = {
    "red": [
        (np.array([0,   200,  70]), np.array([10,  255, 255])),
        (np.array([170, 200,  70]), np.array([180, 255, 255])),
    ],
    "green": [
        (np.array([40,  150,  70]), np.array([80,  255, 255])),
    ],
    "blue": [
        (np.array([100, 150,  70]), np.array([130, 255, 255])),
    ],
    "yellow": [
        (np.array([20,  150,  70]), np.array([35,  255, 255])),
    ],
}

DEBUG_COLORS = {
    "red":    (0,   0,   255),
    "green":  (0,   255, 0),
    "blue":   (255, 0,   0),
    "yellow": (0,   255, 255),
}


def compute_circularity(contour) -> float:
    """
    Circularity = 4pi * area / perimeter^2
    Perfect circle = 1.0, elongated shape approaches 0.0
    Using > 0.45 rejects wall segments while accepting circular targets.
    """
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return 0.0
    return (4.0 * math.pi * area) / (perimeter ** 2)


class ColorDetectorNode(Node):

    def __init__(self):
        super().__init__("color_detector_node")

        self.declare_parameter("image_topic",       "/camera/image_raw")
        self.declare_parameter("target_color",      "red")
        self.declare_parameter("min_area",          800.0)
        self.declare_parameter("min_circularity",   0.45)
        self.declare_parameter("smoothing_alpha",   0.5)
        self.declare_parameter("show_debug_window", False)

        self.image_topic  = self.get_parameter("image_topic").value
        self.target_color = self.get_parameter("target_color").value.lower()
        self.min_area     = self.get_parameter("min_area").value
        self.min_circ     = self.get_parameter("min_circularity").value
        self.alpha        = self.get_parameter("smoothing_alpha").value
        self.show_debug   = self.get_parameter("show_debug_window").value

        if self.target_color not in COLOR_RANGES:
            self.get_logger().warn(
                f"Unknown color {self.target_color}. Defaulting to red."
            )
            self.target_color = "red"

        qos = QoSProfile(depth=5)
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos
        )

        self.pub_detected = self.create_publisher(Bool,    "/color_follower/detected",         10)
        self.pub_error    = self.create_publisher(Float32, "/color_follower/normalized_error",  10)
        self.pub_area     = self.create_publisher(Float32, "/color_follower/target_area",       10)
        self.pub_debug    = self.create_publisher(Image,   "/color_follower/debug_image",       qos)

        self.smoothed_cx   = None
        self.smoothed_area = None

        self.get_logger().info(
            f"ColorDetectorNode started | target: {self.target_color.upper()} | "
            f"min_area: {self.min_area} | min_circularity: {self.min_circ}"
        )

    def _build_mask(self, hsv):
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for (lower, upper) in COLOR_RANGES[self.target_color]:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,   kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=1)
        return mask

    def _smooth(self, cx, area):
        if self.smoothed_cx is None:
            self.smoothed_cx, self.smoothed_area = cx, area
        else:
            a = self.alpha
            self.smoothed_cx   = a * cx   + (1.0 - a) * self.smoothed_cx
            self.smoothed_area = a * area + (1.0 - a) * self.smoothed_area

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        h, w   = frame.shape[:2]
        f_area = float(w * h)
        hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask   = self._build_mask(hsv)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        detected   = False
        norm_error = 0.0
        norm_area  = 0.0
        debug      = frame.copy()

        cv2.line(debug, (w // 2, 0), (w // 2, h), (200, 200, 200), 1)

        best_contour = None
        best_area    = 0.0

        for c in contours:
            area = float(cv2.contourArea(c))
            if area < self.min_area:
                continue
            circularity = compute_circularity(c)
            if circularity < self.min_circ:
                cv2.drawContours(debug, [c], -1, (100, 100, 100), 1)
                self.get_logger().debug(
                    f"Rejected: area={area:.0f} circularity={circularity:.2f} - wall segment"
                )
                continue
            if area > best_area:
                best_area    = area
                best_contour = c

        if best_contour is not None:
            M  = cv2.moments(best_contour)
            cx = float(M["m10"] / M["m00"]) if M["m00"] != 0 else w / 2.0
            self._smooth(cx, best_area)

            norm_error = (self.smoothed_cx - w / 2.0) / (w / 2.0)
            norm_area  = self.smoothed_area / f_area
            detected   = True

            color  = DEBUG_COLORS.get(self.target_color, (0, 255, 0))
            cx_int = int(self.smoothed_cx)
            circ   = compute_circularity(best_contour)

            cv2.drawContours(debug, [best_contour], -1, color, 2)
            cv2.circle(debug, (cx_int, h // 2), 8, color, -1)
            cv2.putText(
                debug,
                f"{self.target_color.upper()} err={norm_error:+.2f} circ={circ:.2f}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2
            )

            self.get_logger().info(
                f"TARGET | err={norm_error:+.3f} circularity={circ:.2f} area={norm_area:.4f}",
                throttle_duration_sec=0.5
            )
        else:
            self.smoothed_cx   = None
            self.smoothed_area = None

        self.pub_detected.publish(Bool(data=detected))
        self.pub_error.publish(Float32(data=float(norm_error)))
        self.pub_area.publish(Float32(data=float(norm_area)))

        try:
            self.pub_debug.publish(
                self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            )
        except CvBridgeError:
            pass

        if self.show_debug:
            try:
                cv2.imshow("Color Follower - Camera", debug)
                cv2.imshow("Color Follower - Mask",   mask)
                cv2.waitKey(1)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    main()
