#!/usr/bin/env python3
"""
run_handshake_demo.py
---------------------
Spins both DroneTigerNode and UgvCharlieNode in an MultiThreadedExecutor
to demonstrate the full 3-step handshake in a single command.
"""

import time
import rclpy
from rclpy.executors import MultiThreadedExecutor

from drone_node import DroneTigerNode
from ugv_node import UgvCharlieNode


def main():
    rclpy.init()

    ugv_node = UgvCharlieNode()
    drone_node = DroneTigerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(ugv_node)
    executor.add_node(drone_node)

    print("\n" + "=" * 60)
    print("  ROS 2 DRONE <-> UGV HANDSHAKE DEMONSTRATION")
    print("  Tiger (Crazyflie Drone) <---> Charlie (ROSbot XL UGV)")
    print("=" * 60 + "\n")

    # Spin until both nodes complete the handshake
    start_time = time.time()
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            if drone_node.handshake_complete and ugv_node.handshake_complete:
                time.sleep(0.5)
                executor.spin_once(timeout_sec=0.1)
                print("\n" + "=" * 60)
                print("🎉 BOTH NODES CONFIRMED: FULL HANDSHAKE SUCCESSFUL!")
                print("=" * 60 + "\n")
                break
            if time.time() - start_time > 15.0:
                print("❌ Handshake timed out after 15 seconds.")
                break
    except KeyboardInterrupt:
        pass
    finally:
        drone_node.destroy_node()
        ugv_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
