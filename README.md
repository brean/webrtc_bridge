# webrtc-bridge
ROS 2 camera streaming

## What is this?
This is an example implementation of a ROS 2 camera streaming node using 
- [OpenCV 2](https://opencv.org/) to get image data as stream
- Forward the data using [aiortc](https://github.com/aiortc/aiortc) to send via WebRTC
- Receive the data and forward it as [ROS](https://ros.org) node

## Installation
You can either run this locally in your [ROS 2](https://docs.ros.org/en/jazzy/Installation.html) environment or you can use [Docker CE](https://docs.docker.com/engine/install/).

This webrtc-bridge is build with multiple scenarios in mind, with different machines that can send and receive video streams. Because of this the actual dependencies and installations can vary from machine to machine. Each example in the documentation has a section about the required installation steps.

NOTE: If you use Docker you need a Linux system that uses Wayland (Fedora 25 or Ubuntu 22.04 or newer).

## Communication Overview
Here are some examples how you could use this bridge to send, receive and process image data

## Example #1: Direct WebRTC connection
You use this system without ROS to get a video stream from a machine with a camera to your browser via web(socket) server:

Although we are running a ROS 2 node we only use the raw websocket to send and receive a video stream.

This is a minimal setup where we just receive a video stream, send its data over WebRTC and just forward the received data to ROS. You can visualize the images with RVIz or the rqt-image viewer.

Start with [Example 01: Simple Direct Communication](docs/01_simple_direct.md)

## Example #2: Forward image data to ROS for processing
In this example we have ROS on the sender and receiver side and we run some image processing on both with ros. The images get feed from a camera via OpenCV and are directly published as ROS images, then a ROS node that subscribes to the images locally forwards them via WebRTC to another Machine that also runs a ROS 2 node for image processing.

Because we are already in a complex ROS network the ROS nodes also exchange the Signaling information for WebRTC via ROS.

This could be useful for a scenario where you want to use the camera to do obstacle avoidance on your robot but also want to process the image on a more capable Computer or the cloud.

Continue with [Example 02: Forward and process data](docs/02_forward_ros.md)

### Example #3: Use the bridge with different sender and receiver
In this example we use the bridge to connect multiple systems with a camera, this could be useful for manual remote control of a robot as well as image processing in the cloud.

We use the WebSocket as well as a ROS node to exchange Signaling information, as we want to distribute the images for multiple receiver and display/process them in multiple places

We also include [svelte-robot-control](svelte-robot-control) to provide a user interface that also sends control commands back to the robot.

Next is [Example 03: Multiple Receiver](docs/03_multiple_receiver.md)

