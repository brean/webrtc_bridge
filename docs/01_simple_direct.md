# Overview

This is a simple example with one dedicated machine that has a webcam but does not run ROS (for example a raspberry pi with Raspberry Pi OS or a notebook with a webcam), a server and a machine that runs ROS where you want to process the received images as ROS-Messages.

For more complex examples, that are maybe closer to your application look at the other examples.

# Installation
## Docker
TODO!

## Individual installation
### Webcam-PC
For this example you need to install OpenCV and webrtc on the machine that sends the video stream you would install either just aiortc as webrtc-client with opencv and av to simply forward your camera images:
```bash
sudo apt-get install -y python3-aiortc python3-opencv python3-av
```

### WebRTC-Signaling Server
And on the server you need aiortc and fastapi
```bash
sudo apt-get install -y python3-aiortc python3-fastapi
```

### ROS-System PC
We also have a client for that receives the images from the server and forwards them to ROS so that needs at least aiortc, fastapi (as websocket-client) and the ROS 2 CV-bridge:
```bash
source /opt/ros/YOUR_ROS_ENV/setup.bash
sudo apt-get install -y python3-aiortc python3-fastapi ros-$ROS_DISTRO-cv-bridge
```

## Sequence Diagram
```mermaid
sequenceDiagram
    participant b as ROS Node to forward<br> as sensor_msgs/Image
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