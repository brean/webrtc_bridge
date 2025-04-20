#!/usr/bin/env python3
"""Receive WebRTC messages and re-publish as ROS 2 msgs."""
import asyncio

import cv2

import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from aiortc.contrib.signaling import TcpSocketSignaling
from av import VideoFrame

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class WebRTCBridge(Node):
    def __init__(self):
        super().__init__('webrtc_bridge')
        # TODO: publish ros2 messages from webrtc
        self.get_logger().info('ROS2 node started')
        self.bridge = CvBridge()
        self.track = None
        self.img_publisher = self.create_publisher(Image, 'images', 10)

    async def spin(self):
        # based on https://github.com/m2-farzan/ros2-asyncio/tree/main
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            await asyncio.sleep(1e-4)

    async def handle_track(self, track):
        print("Inside handle track")
        self.track = track
        frame_count = 0
        while True:
            try:
                print("Waiting for frame...")
                frame = await asyncio.wait_for(track.recv(), timeout=5.0)
                frame_count += 1
                print(f"Received frame {frame_count}")
                if isinstance(frame, VideoFrame):
                    frame = frame.to_ndarray(format="bgr24")
                elif isinstance(frame, np.ndarray):
                    print(f"Frame type: numpy array")
                else:
                    print(f"Unexpected frame type: {type(frame)}")
                    continue
                # publish as ROS message
                img = self.bridge.cv2_to_imgmsg(frame, 'bgr24')
                img.header.stamp = self.get_clock().now().to_msg()
                self.img_publisher.publish(img)
            except asyncio.TimeoutError:
                print("Timeout waiting for frame, continuing...")
            except Exception as e:
                print(f"Error in handle_track: {str(e)}")
                if "Connection" in str(e) or str(e) == '':
                    break
        print("Exiting handle_track")


async def run(node, pc, signaling):
    await signaling.connect()

    @pc.on("track")
    def on_track(track):
        if isinstance(track, MediaStreamTrack):
            print(f"Receiving {track.kind} track")
            asyncio.ensure_future(node.handle_track(track))

    @pc.on("datachannel")
    def on_datachannel(channel):
        print(f"Data channel established: {channel.label}")

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"Connection state is {pc.connectionState}")
        if pc.connectionState == "connected":
            print("WebRTC connection established successfully")

    print("Waiting for offer from sender...")
    offer = await signaling.receive()
    print("Offer received")
    await pc.setRemoteDescription(offer)
    print("Remote description set")

    answer = await pc.createAnswer()
    print("Answer created")
    await pc.setLocalDescription(answer)
    print("Local description set")

    await signaling.send(pc.localDescription)
    print("Answer sent to sender")

    print("Waiting for connection to be established...")
    while pc.connectionState != "connected":
        await asyncio.sleep(0.1)

    print("Connection established, waiting for frames...")
    await asyncio.sleep(100)  # Wait for 35 seconds to receive frames

    print("Closing connection")


async def async_main(node):
    await node.spin()

    node.destroy_node()
    rclpy.shutdown()


async def webrtc_main(node):
    # create WebRTC signaling server using aiortc
    signaling = TcpSocketSignaling("127.0.0.1", 9999)
    pc = RTCPeerConnection()

    try:
        await run(node, pc, signaling)
    except Exception as e:
        print(f"Error in main: {str(e)}")
    finally:
        print("Closing peer connection")
        await pc.close()


async def _async_main(args=None):
    rclpy.init(args=None)
    node = WebRTCBridge()
    # asyncio.run(async_main(node))
    async with asyncio.TaskGroup() as tg:
        tg.create_task(async_main(node))
        tg.create_task(webrtc_main(node))


def main():
    asyncio.run(_async_main())


if __name__ == '__main__':
    main()
