#!/usr/bin/env python3
"""
drone_node.py
-------------
ROS 2 Node representing the Crazyflie Drone (Callsign: Tiger).

Handshake Sequence:
  Step 1: Drone transmits:
          tiger to charlie
          tiger in
          tiger in
  Step 2: Awaits Charlie's acknowledgment:
          charlie to tiger
          message received
          message received
  Step 3: Drone transmits final confirmation:
          tiger to charile
          communication successful
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DroneTigerNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.get_logger().info('🚁 [Tiger / Drone Node] Online and listening on ROS 2.')

        # Publisher for Tiger -> Charlie
        self.publisher_ = self.create_publisher(String, '/tiger_to_charlie', 10)

        # Subscriber for Charlie -> Tiger
        self.subscription_ = self.create_subscription(
            String,
            '/charlie_to_tiger',
            self.charlie_callback,
            10
        )

        self.step = 0
        self.handshake_complete = False

        # Periodic timer to initiate handshake after nodes discover each other
        self.timer = self.create_timer(1.0, self.send_initial_callout)

    def send_initial_callout(self):
        if self.step == 0:
            msg_text = "tiger to charlie\ntiger in\ntiger in"
            msg = String()
            msg.data = msg_text
            self.publisher_.publish(msg)
            self.get_logger().info(
                f"\n{'='*40}\n[Tiger -> Charlie] TRANSMITTING:\n{msg_text}\n{'='*40}"
            )
            self.step = 1
            # Resend every 2 seconds if no reply yet
            self.timer.cancel()
            self.timer = self.create_timer(2.0, self.retry_if_no_reply)

    def retry_if_no_reply(self):
        if self.step == 1 and not self.handshake_complete:
            msg_text = "tiger to charlie\ntiger in\ntiger in"
            msg = String()
            msg.data = msg_text
            self.publisher_.publish(msg)
            self.get_logger().info(f"[Tiger -> Charlie] Retrying callout...")

    def charlie_callback(self, msg):
        received_text = msg.data.strip()
        self.get_logger().info(
            f"\n{'='*40}\n[Tiger <- Charlie] RECEIVED:\n{received_text}\n{'='*40}"
        )

        if "message received" in received_text and self.step == 1:
            self.step = 2
            self.timer.cancel()

            # Step 3: Send final confirmation
            final_text = "tiger to charile\ncommunication successful"
            final_msg = String()
            final_msg.data = final_text
            self.publisher_.publish(final_msg)
            self.get_logger().info(
                f"\n{'='*40}\n[Tiger -> Charlie] TRANSMITTING:\n{final_text}\n{'='*40}"
            )
            self.get_logger().info("✅ [Tiger] Communication handshake completed successfully!")
            self.handshake_complete = True


def main(args=None):
    rclpy.init(args=args)
    node = DroneTigerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
