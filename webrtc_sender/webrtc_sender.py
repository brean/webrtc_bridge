"""Get local webcam and provide its data as webrtc-client.

Based on https://github.com/eknathmali/
Real-Time-Video-Streaming-with-WebRTC-and-Python.git
"""
import asyncio
import fractions

from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.signaling import TcpSocketSignaling

from av import VideoFrame

import cv2


class CustomVideoStreamTrack(VideoStreamTrack):
    """Provide Video Stream from Camera using cv2."""

    def __init__(self, camera_id: int):
        """Set up video capture and reset frame count."""
        super().__init__()
        self.cap = cv2.VideoCapture(camera_id)
        # TODO: add ROS 2 messages to configure frame rate and resolution.
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.frame_count = 0

    async def recv(self):
        """
        Get frame from caputre device and return.

        The name "rcev" for receive might be a bit misleading, as we
        are not receiving images but instead provide the next frame.
        """
        self.frame_count += 1
        ret, frame = self.cap.read()
        if not ret:
            print('Failed to read frame from camera.')
            return None
        video_frame = VideoFrame.from_ndarray(frame, format='rgb24')
        video_frame.pts = self.frame_count
        video_frame.time_base = fractions.Fraction(1, 30)
        return video_frame


async def setup_webrtc_and_run(host: str, port: int, camera_id: int):
    """Set up TcpSocketSignaling server and our custom VideoStreamTrack."""
    signaling = TcpSocketSignaling(host, port)
    pc = RTCPeerConnection()
    video_sender = CustomVideoStreamTrack(camera_id)
    pc.addTrack(video_sender)

    try:
        await signaling.connect()

        @pc.on('datachannel')
        def on_datachannel(channel):
            print(f'Data channel established: {channel.label}')

        @pc.on('connectionstatechange')
        async def on_connectionstatechange():
            print(f'Connection state is {pc.connectionState}')

        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        await signaling.send(pc.localDescription)

        while True:
            obj = await signaling.receive()
            if isinstance(obj, RTCSessionDescription):
                await pc.setRemoteDescription(obj)
                print('Remote description set')
            elif obj is None:
                print('Signaling ended')
                break
        print('Closing connection')
    finally:
        await pc.close()


async def main():
    """Connect to server and provide Camera images."""
    # TODO: get address and port of some server
    host = '0.0.0.0'
    port = 9999
    camera_id = 0
    while True:
        await setup_webrtc_and_run(host, port, camera_id)

if __name__ == '__main__':
    asyncio.run(main())
