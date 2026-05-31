#!/usr/bin/env python3
"""
smc_controller_node.py

Semi-autonomous motion controller using Sliding Mode Control (SMC).

Design philosophy — shared autonomy:
    WAITING state : SMC does nothing, human drives freely via teleop
    Target found  : SMC takes over automatically for precise alignment
    Target lost   : returns to WAITING, human drives again

This mirrors real-world semi-autonomous robots where humans handle
strategic navigation and the robot handles precision tasks autonomously.

State Machine:
    WAITING ──detected──► ALIGNING ──centered──► APPROACHING ──close──► HOLDING
       ▲                      │                       │
       │                      └──── lost timeout ─────┘
       └──────────────── LOST ◄─────────────────────────
"""

import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Twist


class State(Enum):
    WAITING     = "WAITING"
    ALIGNING    = "ALIGNING"
    APPROACHING = "APPROACHING"
    HOLDING     = "HOLDING"
    LOST        = "LOST"


class SMCController:
    """
    Sliding Mode Controller for visual servoing.

    Control objective: drive lateral error e -> 0
    where e = normalized centroid error in [-1.0, 1.0]

    Sliding surface:
        s = e_dot + lambda * e

    Control law (tanh boundary layer to suppress chattering):
        u = -k_s * tanh(s / phi) - k_eq * e

    Parameters:
        lambda_s : sliding surface slope
        k_s      : switching gain
        k_eq     : equivalent control gain
        phi      : boundary layer thickness
    """

    def __init__(self, lambda_s=1.5, k_s=0.5, k_eq=0.3, phi=0.1):
        self.lambda_s = lambda_s
        self.k_s      = k_s
        self.k_eq     = k_eq
        self.phi      = phi
        self._prev_error = 0.0
        self._prev_time  = time.monotonic()

    def reset(self):
        self._prev_error = 0.0
        self._prev_time  = time.monotonic()

    def compute(self, error: float) -> float:
        now   = time.monotonic()
        dt    = max(now - self._prev_time, 1e-4)
        e_dot = (error - self._prev_error) / dt
        s     = e_dot + self.lambda_s * error
        control = -(self.k_s * math.tanh(s / self.phi) + self.k_eq * error)
        self._prev_error = error
        self._prev_time  = now
        return control


class SMCMotionControllerNode(Node):

    def __init__(self):
        super().__init__("smc_motion_controller_node")

        self.declare_parameter("cmd_vel_topic",      "/cmd_vel")
        self.declare_parameter("linear_speed",        0.15)
        self.declare_parameter("max_angular_speed",   0.6)
        self.declare_parameter("center_tolerance",    0.08)
        self.declare_parameter("max_area_threshold",  0.035)
        self.declare_parameter("lost_timeout",        1.5)
        self.declare_parameter("smc_lambda",          1.5)
        self.declare_parameter("smc_k_s",             0.5)
        self.declare_parameter("smc_k_eq",            0.3)
        self.declare_parameter("smc_phi",             0.1)

        cmd_topic         = self.get_parameter("cmd_vel_topic").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.max_angular  = self.get_parameter("max_angular_speed").value
        self.center_tol   = self.get_parameter("center_tolerance").value
        self.max_area     = self.get_parameter("max_area_threshold").value
        self.lost_timeout = self.get_parameter("lost_timeout").value

        self.smc = SMCController(
            lambda_s = self.get_parameter("smc_lambda").value,
            k_s      = self.get_parameter("smc_k_s").value,
            k_eq     = self.get_parameter("smc_k_eq").value,
            phi      = self.get_parameter("smc_phi").value,
        )

        self.state    = State.WAITING
        self.detected = False
        self.error    = 0.0
        self.area     = 0.0
        self.last_detected_time = time.monotonic()

        self.create_subscription(Bool,    "/color_follower/detected",         self._cb_detected, 10)
        self.create_subscription(Float32, "/color_follower/normalized_error", self._cb_error,    10)
        self.create_subscription(Float32, "/color_follower/target_area",      self._cb_area,     10)

        self.cmd_pub   = self.create_publisher(Twist,  cmd_topic,               10)
        self.state_pub = self.create_publisher(String, "/color_follower/state", 10)

        self.create_timer(0.05, self._control_loop)

        self.get_logger().info("SMCMotionControllerNode started")
        self.get_logger().info("Mode: SEMI-AUTONOMOUS — drive freely, SMC takes over on detection")
        self.get_logger().info(
            f"SMC: lambda={self.smc.lambda_s} "
            f"k_s={self.smc.k_s} k_eq={self.smc.k_eq} phi={self.smc.phi}"
        )

    def _cb_detected(self, msg: Bool):
        prev = self.detected
        self.detected = msg.data
        if self.detected:
            self.last_detected_time = time.monotonic()
        if not prev and self.detected:
            self.smc.reset()

    def _cb_error(self, msg: Float32):
        self.error = msg.data

    def _cb_area(self, msg: Float32):
        self.area = msg.data

    def _transition(self, new_state: State):
        if new_state != self.state:
            self.get_logger().info(
                f"State: {self.state.value} -> {new_state.value}"
            )
            self.state = new_state
            if new_state in (State.WAITING, State.LOST):
                self.smc.reset()

    def _update_state(self):
        elapsed = time.monotonic() - self.last_detected_time

        if self.state == State.WAITING:
            if self.detected:
                self._transition(State.ALIGNING)

        elif self.state == State.ALIGNING:
            if not self.detected and elapsed > self.lost_timeout:
                self._transition(State.LOST)
            elif self.detected and abs(self.error) <= self.center_tol:
                self._transition(State.APPROACHING)

        elif self.state == State.APPROACHING:
            if not self.detected and elapsed > self.lost_timeout:
                self._transition(State.LOST)
            elif self.detected:
                if self.area >= self.max_area:
                    self._transition(State.HOLDING)
                elif abs(self.error) > self.center_tol:
                    self._transition(State.ALIGNING)

        elif self.state == State.HOLDING:
            if not self.detected and elapsed > self.lost_timeout:
                self._transition(State.LOST)
            elif self.detected and self.area < self.max_area * 0.8:
                self._transition(State.APPROACHING)

        elif self.state == State.LOST:
            self._transition(State.WAITING)

    def _control_loop(self):
        self._update_state()

        self.state_pub.publish(String(data=self.state.value))

        if self.state == State.WAITING:
            # Human has full control via teleop — SMC publishes nothing
            self.get_logger().info(
                "[ WAITING ] Drive robot toward target — SMC will take over on detection",
                throttle_duration_sec=2.0
            )
            return

        elif self.state == State.LOST:
            # Brief stop then return to WAITING
            stop = Twist()
            self.cmd_pub.publish(stop)
            return

        twist = Twist()

        if self.state == State.ALIGNING:
            angular = self.smc.compute(self.error)
            angular = max(-self.max_angular, min(self.max_angular, angular))
            twist.angular.z = angular
            twist.linear.x  = 0.0

        elif self.state == State.APPROACHING:
            angular = self.smc.compute(self.error)
            angular = max(-self.max_angular, min(self.max_angular, angular))
            twist.angular.z = angular
            centered_factor = max(0.0, 1.0 - abs(self.error) / self.center_tol)
            twist.linear.x  = self.linear_speed * centered_factor

        elif self.state == State.HOLDING:
            twist.angular.z = 0.0
            twist.linear.x  = 0.0

        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f"[{self.state.value:12s}] "
            f"err={self.error:+.3f} "
            f"area={self.area:.4f} "
            f"w={twist.angular.z:+.3f} "
            f"v={twist.linear.x:.3f}",
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = SMCMotionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            stop = Twist()
            node.cmd_pub.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()