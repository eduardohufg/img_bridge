# Image bridge to transfer images over Web Sockets

### Requirements

This package require ROS2 and fastAPI to launch a web server to send data. 

1. *Ros Humble*: You have to have ROS2 with an Ubuntu machine, for humble distribution you require 22.04 version.
    [Download ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)

#### Install FastAPI and other requirements

Open a terminal and run the following coommands 

```bash
pip install fastapi "uvicorn[standard]" websocket-client opencv-python numpy
```

