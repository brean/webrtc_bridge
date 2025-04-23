# webrtc-bridge
ROS 2 camera streaming

## What is this?
This is an example implementation of a ROS 2 camera streaming node using 
- [OpenCV 2](https://opencv.org/) to get image data as stream
- Forward the data using [aiortc](https://github.com/aiortc/aiortc) to send via WebRTC
- Receive the data and forward it as [ROS](https://ros.org) node
- 

## Installation

Install docker (docker-ce).
run `docker compose build`

## Application
- WebRTC adjusts bitrate automatically depending on the network, ROS does not have such a feature, so the idea is to use WebRTC for images over a slower network and re-publishing on a local machine to use all the handy ROS and computer vision tools. This repo provides docker container to with a basic test setup.
- Sending ROS-images to a Browser is possible (e.g. compressing the images as websocket) but not a very elegant solution, however WebRTC as standard already exists, so it makes lots of sense to use it instead. See TODO: svelte-robot-control as example web application on controlling a robot using a [Virtual Joystick](https://github.com/brean/svelte-gamepad-virtual-joystick)

## Usage