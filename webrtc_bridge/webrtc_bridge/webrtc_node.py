import asyncio

import aiohttp
import json
import asyncio

from av import VideoFrame
import numpy as np
import cv2

import rclpy

from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from aiortc import \
    MediaStreamTrack, VideoStreamTrack, \
    RTCPeerConnection, RTCSessionDescription


class VideoTrack(VideoStreamTrack):
    """Custom VideoTrack handling."""
    async def recv(self):
        frame = await self.recv()
        return frame


class WebRTCBridge(Node):
    """ROS 2 node to provide images from an WebRTC client."""
    # TODO: this could also be a client?!

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

    def forward_img(self, frame):
        img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.img_publisher.publish(img)


async def handle_track(node, track, receiver_id):
    frame_count = 0
    while True:
        try:
            frame = await asyncio.wait_for(track.recv(), timeout=5.0)
            frame_count += 1
            # self.get_logger().info(f"Received frame {frame_count}")
            if isinstance(frame, VideoFrame):
                frame = frame.to_ndarray(format="rgb24")
            elif not isinstance(frame, np.ndarray):
                node.get_logger().error(
                    f"Unexpected frame type: {type(frame)}")
                continue
            node.get_logger().info(
                f'Published video frame {frame_count} '
                f'{frame.shape[1]}:{frame.shape[0]}')
            
            node.forward_img(frame)
        except asyncio.TimeoutError:
            node.get_logger().error(
                "Timeout waiting for frame, continuing...")
        except Exception as e:
            node.get_logger().error(f"Error in handle_track: {str(e)}")
            if "Connection" in str(e) or str(e) == '':
                break


async def receive_video(node, sender_id, receiver_id, offer, websocket):
    pc = RTCPeerConnection()
    pc.addTrack(VideoTrack())

    @pc.on("track")
    def on_track(track):
        if isinstance(track, MediaStreamTrack):
            asyncio.ensure_future(handle_track(node, track, receiver_id))

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    await websocket.send_json({
        'type': 'answer',
        'sender_id': sender_id,
        'receiver_id': receiver_id,
        'answer': {
            'type': answer.type,
            'sdp': answer.sdp}
    })

    while pc.connectionState != "connected":
        await asyncio.sleep(0.5)


async def websocket_handler(node):
    """Get streaming information from WebRTC, connect to first Sender."""
    session = aiohttp.ClientSession()
    receiver_id = None
    receiving = False
    # TODO: We need some UI to select any one of the
    # server that provide RTC streams, so we can send request_offer
    # then the sender should create an offer that we can answer to
    async with session.ws_connect('ws://localhost:8080/receiver') as ws:
        async for message in ws:
            data = json.loads(message.data)
            data_type = data['type']
            if data_type == 'registered':
                receiver_id = data['receiver_id']
                node.get_logger().info(f'registered receiver as {receiver_id}')
            elif data_type == 'sender':
                sender_id = data['sender_id']
                # TODO: for now we assume there is only ONE sender
                # however we might want to add some ui to select a
                # specific sender, for now we just request the offer
                # from the first sender we receive
                if receiving:
                    continue
                node.get_logger().info(f'request offer from {sender_id}')
                receiving = True
                await ws.send_json({
                    'type': 'request_offer',
                    'receiver_id': receiver_id,
                    'sender_id': sender_id
                })
            elif data_type == 'offer':
                if not receiver_id:
                    node.get_logger().info('no receiver_id set')
                    continue
                if data.get('receiver_id') != receiver_id:
                    node.get_logger().info(
                        'reciever ids does not match: '
                        f'{data.get("receiver_id")} - {receiver_id}')
                    continue
                rtc_offer = RTCSessionDescription(**data['offer'])
                node.get_logger().info(f'receiving video from {sender_id}')
                asyncio.create_task(receive_video(
                    node, sender_id, receiver_id, rtc_offer, ws))
            else:
                node.get_logger().info(
                    f'received unknown data type "{data_type}"')


async def async_ros(node):
    await node.spin()
    node.destroy_node()
    rclpy.shutdown()


async def async_main(args=None):
    """Create ROS node and start WebRTC and ROS loops."""
    rclpy.init(args=None)
    node = WebRTCBridge()
    # asyncio.run(async_main(node))
    async with asyncio.TaskGroup() as tg:
        tg.create_task(websocket_handler(node))
        tg.create_task(async_ros(node))


def main():
    """Entrypoint to start the global async loop."""
    asyncio.run(async_main())


if __name__ == '__main__':
    main()
