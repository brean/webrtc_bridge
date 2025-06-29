#!/usr/bin/env python3
"""Receive WebRTC messages and re-publish as ROS 2 msgs."""
import asyncio

from aiortc import MediaStreamTrack, RTCPeerConnection
from aiortc.contrib.signaling import TcpSocketSignaling

from av import VideoFrame

import numpy as np

import rclpy

from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class WebRTCBridge(Node):
    """ROS 2 node to provide images from an WebRTC client."""

    def __init__(self):
        """Set up ROS 2 image publisher."""
        super().__init__('webrtc_bridge')
        # TODO: publish ros2 messages from webrtc
        self.get_logger().info('ROS2 node started')
        self.bridge = CvBridge()
        self.track = None
        self.img_publisher = self.create_publisher(Image, 'image_raw', 1)

    async def spin(self):
        """Update ROS 2 loop utilizing asyncio."""
        # based on https://github.com/m2-farzan/ros2-asyncio/tree/main
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            await asyncio.sleep(1e-4)
        self.get_logger().info('not spinning anymore')

    async def handle_track(self, track):
        self.get_logger().info('Inside handle track.')
        self.track = track
        frame_count = 0
        while True:
            try:
                # self.get_logger().info('Waiting for frame...')
                frame = await asyncio.wait_for(track.recv(), timeout=5.0)
                frame_count += 1
                # self.get_logger().info(f'Received frame {frame_count}')
                if isinstance(frame, VideoFrame):
                    frame = frame.to_ndarray(format='rgb24')
                elif isinstance(frame, np.ndarray):
                    self.get_logger().info('Frame type: numpy array')
                else:
                    self.get_logger().info(
                        f'Unexpected frame type: {type(frame)}')
                    continue
                # publish as ROS message
                img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.img_publisher.publish(img)
                self.get_logger().info(
                    f'Published video frame {frame_count} '
                    f'{frame.shape[1]}:{frame.shape[0]}')
            except asyncio.TimeoutError:
                self.get_logger().info('Timeout waiting for frame, continuing...')
            except Exception as e:
                self.get_logger().info(f'Error in handle_track: {str(e)}')
                if 'Connection' in str(e) or str(e) == '':
                    break
        self.get_logger().info('Exiting handle_track')

    async def run_signal(self, pc, signaling):
        await signaling.connect()

        @pc.on('track')
        def on_track(track):
            if isinstance(track, MediaStreamTrack):
                self.get_logger().info(f'Receiving {track.kind} track')
                asyncio.ensure_future(self.handle_track(track))

        @pc.on('datachannel')
        def on_datachannel(channel):
            self.get_logger().info(f'Data channel established: {channel.label}')

        @pc.on('connectionstatechange')
        async def on_connectionstatechange():
            self.get_logger().info(f'Connection state is {pc.connectionState}')
            if pc.connectionState == 'connected':
                self.get_logger().info('WebRTC connection established successfully')

        self.get_logger().info('Waiting for offer from sender...')
        offer = await signaling.receive()
        self.get_logger().info('Offer received')
        await pc.setRemoteDescription(offer)
        self.get_logger().info('Remote description set')

        answer = await pc.createAnswer()
        self.get_logger().info('Answer created')
        await pc.setLocalDescription(answer)
        self.get_logger().info('Local description set')

        await signaling.send(pc.localDescription)
        self.get_logger().info('Answer sent to sender')

        self.get_logger().info('Waiting for connection to be established...')
        while pc.connectionState != 'connected':
            await asyncio.sleep(0.5)

        self.get_logger().info('Connection established, waiting for frames...')
        while True:
            # Wait forever to receive frames
            await asyncio.sleep(1)

        self.get_logger().info('Closing connection')


async def async_ros(node):
    await node.spin()
    node.destroy_node()
    rclpy.shutdown()


async def webrtc_main(node):
    """Create WebRTC signaling server using aiortc."""
    signaling = TcpSocketSignaling('cam', 9999)
    pc = RTCPeerConnection()
    logger = node.get_logger()
    try:
        await node.run_signal(pc, signaling)
    except Exception as e:
        logger.info(f'Error in main: {str(e)}')
    finally:
        logger.info('Closing peer connection')
        await pc.close()


async def async_main(args=None):
    """Create ROS node and start WebRTC and ROS loops."""
    rclpy.init(args=None)
    node = WebRTCBridge()
    # asyncio.run(async_main(node))
    async with asyncio.TaskGroup() as tg:
        tg.create_task(webrtc_main(node))
        tg.create_task(async_ros(node))


def main():
    """Entrypoint to start the global async loop."""
    asyncio.run(async_main())


if __name__ == '__main__':
    main()
