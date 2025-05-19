# webrtc-bridge
This is an example implementation of a ROS 2 camera streaming node using 
- [OpenCV 2](https://opencv.org/) to get image data as stream or from a webcam of a browser or rosnode
- Provide the data using [aiortc](https://github.com/aiortc/aiortc) as WebRTC video stream.
- The clients exchange their information either via WebSocket or ROS.
- Receive the data and forward it as [ROS](https://ros.org) node, or directly to the browser. 

## Communication Overview
Here are some examples how you could use this bridge to send, receive and process image data

### Example #1: Direct WebRTC connection (no ROS needed)
You use this system completely with WebSockets only using ROS to control the robot:
```mermaid
sequenceDiagram
    participant b as Browser
    participant srv as WebSocket Server
    participant snd as Sender with Webcam
    snd-->>srv: connects
    Note left of srv: Server generates unique_ids
    srv->>snd: {type: "registered", sender_id}
    b-->>srv: connects
    srv->>b: {type: "registered", receive_id}
    loop for all snd
        srv->>b: {"type": "sender", sender_id}
    end
    Note right of b: User selects a sender_id
    b->>srv: {"type": "request_offer", sender_id}
    srv->>snd: {"type": "request_offer", receiver_id}
    snd->>srv: {"type": "offer", "offer": {...}, receiver_id}
    srv->>b: {"type": "offer", "offer": {...}}
    b->>srv: {"type": "answer": answer: {...}, sender_id}
    srv->>snd: {"type": "answer": answer: {...}}
    snd-->>b: send video stream via aiortc (not WebSocket)
```

TODO: how to start

### Example #2: Forward image data to ROS for processing (e.g. obstacle detection)
The second example Sending and forwarding Image Data to a ROS node. The result of the image processing will be shown to a huma that connects with a browser:

```mermaid
sequenceDiagram
    participant r as receiver<br />ROS node
    participant s as sender<br />ROS node
    participant b as WebSocket client<br /> ROS node
    participant srv as WebSocket Server
    participant snd as Sender with Webcam
    snd-->>srv: connects
    Note left of srv: Server generates id
    srv->>snd: {type: "registered", sender_id}
    s-->>srv: connects
    srv->>s: {type: "registered", sender_id}
    b-->>srv: connects
    srv->>b: {type: "registered", receive_id}
    loop for all sender (webcam and ros node)
        srv->>b: {"type": "sender", sender_id}
    end
    Note left of b: send predefined<br />sender_id
    
    b->>srv: {"type": "request_offer", sender_id}
    srv->>snd: {"type": "request_offer", receiver_id}
    snd->>srv: {"type": "offer", "offer": {...}, receiver_id}
    srv->>b: {"type": "offer", "offer": {...}}
    b->>srv: {"type": "answer": answer: {...}, sender_id}
    srv->>snd: {"type": "answer": answer: {...}}
    snd-->>b: send video stream via aiortc (not WebSocket)
    b->>r: ROS2 Image data<br />(on /image topic)
    r->>s: publish processed Image<br />(on /detected topic)
    Note left of s: sender provides a<br>virtual camera
    s->>srv: {"type": "offer", "offer": {...}, receiver_id}
    Note left of srv: A user can now recive the offer from <br/>the websocket client node
```

TODO: how to start

### Example #3: Use the bridge to connect to a ROS 2 system with multiple cameras and forward to a User for manual control

Send compressed image data over a slow connection (e.g. WiFi) from a ROS-using robot with a camera to another ROS system running some image processing and a UI on a Browser, the robot uses local obstacle avoidance and also forwards the image streams for different users to control the robot.

TODO: sequence diagram

TODO: how to start

## Installation

Install docker (docker-ce).
run `docker compose build`

## Application
- WebRTC adjusts bitrate automatically depending on the network, ROS does not have such a feature, so the idea is to use WebRTC for images over a slower network and re-publishing on a local machine to use all the handy ROS and computer vision tools. This repo provides docker container to with a basic test setup.
- Sending ROS-images to a Browser is possible (e.g. compressing the images as websocket) but not a very elegant solution, however WebRTC as standard already exists, so it makes lots of sense to use it instead. See TODO: svelte-robot-control as example web application on controlling a robot using a [Virtual Joystick](https://github.com/brean/svelte-gamepad-virtual-joystick)

## Usage