#!/usr/bin/env python3
"""
ugv_node.py
-----------
ROS 2 Node representing the Husarion ROSbot XL UGV (Callsign: Charlie).

Handshake Sequence:
  Step 1: Charlie listens for Tiger's callout:
          tiger to charlie
          tiger in
          tiger in
  Step 2: Charlie acknowledges:
          charlie to tiger
          message received
          message received
  Step 3: Charlie receives final confirmation:
          tiger to charile
          communication successful
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class UgvCharlieNode(Node):
    def __init__(self):
        super().__init__('ugv_node')
        self.get_logger().info('🚜 [Charlie / UGV Node] Online and listening on ROS 2.')

        # Publisher for Charlie -> Tiger
        self.publisher_ = self.create_publisher(String, '/charlie_to_tiger', 10)

        # Subscriber for Tiger -> Charlie
        self.subscription_ = self.create_subscription(
            String,
            '/tiger_to_charlie',
            self.tiger_callback,
            10
        )

        self.handshake_complete = False

    def tiger_callback(self, msg):
        received_text = msg.data.strip()
        self.get_logger().info(
            f"\n{'='*40}\n[Charlie <- Tiger] RECEIVED:\n{received_text}\n{'='*40}"
        )

        # Step 2: Upon receiving "tiger in", reply with acknowledgement
        if "tiger in" in received_text and not self.handshake_complete:
            reply_text = "charlie to tiger\nmessage received\nmessage received"
            reply_msg = String()
            reply_msg.data = reply_text
            self.publisher_.publish(reply_msg)
            self.get_logger().info(
                f"\n{'='*40}\n[Charlie -> Tiger] TRANSMITTING:\n{reply_text}\n{'='*40}"
            )

        # Step 3: Upon receiving "communication successful", confirm channel
        elif "communication successful" in received_text:
            self.get_logger().info(
                "✅ [Charlie] Final confirmation received. Communication channel established successfully!"
            )
            self.handshake_complete = True


def main(args=None):
    rclpy.init(args=args)
    node = UgvCharlieNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
