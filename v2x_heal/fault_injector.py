import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class FaultInjector(Node):

    def __init__(self):
        super().__init__("v2x_heal_fault_injector")

        # ------------------------------------------------------------
        # PARAMETERS
        # ------------------------------------------------------------

        # 0.0 = select a random fault automatically.
        # Any positive value = use that exact yaw fault.
        self.declare_parameter("fault_yaw_deg", 0.0)

        # Time after demo begins before fault is injected.
        self.declare_parameter("fault_delay_sec", 12.0)

        requested_fault = float(
            self.get_parameter("fault_yaw_deg").value
        )

        self.fault_delay_sec = float(
            self.get_parameter("fault_delay_sec").value
        )

        # ------------------------------------------------------------
        # RANDOM FAULT SELECTION
        # ------------------------------------------------------------

        if requested_fault <= 0.0:
            self.fault_yaw_deg = float(
                random.choice([2, 4, 6, 8, 10, 12])
            )
            self.random_fault_selected = True
        else:
            self.fault_yaw_deg = requested_fault
            self.random_fault_selected = False

        # ------------------------------------------------------------
        # ROS PUBLISHERS
        # ------------------------------------------------------------

        self.fault_state_pub = self.create_publisher(
            Bool,
            "/v2x_heal/fault_active",
            10
        )

        self.fault_yaw_pub = self.create_publisher(
            Float32,
            "/v2x_heal/camera_yaw_fault_deg",
            10
        )

        self.start_time = time.monotonic()

        self.fault_activated = False

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        # ------------------------------------------------------------
        # STARTUP LOGS
        # ------------------------------------------------------------

        self.get_logger().info(
            "V2X-HEAL Fault Injector initialized."
        )

        if self.random_fault_selected:
            self.get_logger().info(
                f"Random Camera Yaw Fault Selected: "
                f"{self.fault_yaw_deg:.1f}°"
            )
        else:
            self.get_logger().info(
                f"Configured Camera Yaw Fault: "
                f"{self.fault_yaw_deg:.1f}°"
            )

        self.get_logger().info(
            f"Fault injection scheduled at T+"
            f"{self.fault_delay_sec:.1f} s"
        )

    # ------------------------------------------------------------
    # TIMER
    # ------------------------------------------------------------

    def timer_callback(self):

        elapsed = time.monotonic() - self.start_time

        state_msg = Bool()
        yaw_msg = Float32()

        if elapsed >= self.fault_delay_sec:

            state_msg.data = True
            yaw_msg.data = self.fault_yaw_deg

            if not self.fault_activated:

                self.fault_activated = True

                self.get_logger().warning(
                    "Fault State Changed: "
                    "ACTIVATED (HARDWARE CAMERA DRIFT)"
                )

                self.get_logger().warning(
                    f"Injected Camera Yaw Drift: "
                    f"+{self.fault_yaw_deg:.1f}°"
                )

        else:

            state_msg.data = False
            yaw_msg.data = 0.0

        self.fault_state_pub.publish(state_msg)
        self.fault_yaw_pub.publish(yaw_msg)


def main(args=None):

    rclpy.init(args=args)

    node = FaultInjector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if rclpy.ok():
            node.get_logger().info(
                "V2X-HEAL Fault Injector stopped."
            )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()