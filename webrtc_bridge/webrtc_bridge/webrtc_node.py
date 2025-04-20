#!/usr/bin/env python3
"""Receive WebRTC messages and re-publish as ROS 2 msgs."""
import asyncio

import cv2

import rclpy
from rclpy.node import Node


class WebRTCBridge(Node):
    def __init__(self):
        super().__init__('webrtc_bridge')
        # TODO: publish ros2 messages from webrtc
        self.get_logger().info('ROS2 node started')

    async def spin(self):
        # based on https://github.com/m2-farzan/ros2-asyncio/tree/main
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            await asyncio.sleep(1e-4)


async def async_main(node):
    await node.spin()

    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=None)
    node = WebRTCBridge()
    asyncio.run(async_main(node))
    # future = asyncio.wait([async_main(node)])
    # asyncio.get_event_loop().run_until_complete(future)


if __name__ == '__main__':
    main()
