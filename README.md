# Image bridge to transfer images over Web Sockets

### Requirements

This package require ROS2 and fastAPI to launch a web server to send data. 

1. *Ros Humble*: You have to have ROS2 with an Ubuntu machine, for humble distribution you require 22.04 version.
    [Download ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)

2. *Python 3.10 or above*: You can Download python in the following link:
   [Download Python](https://www.python.org/downloads/)

3. *Git*: Make sure you have it installed. You can get it from the following link:
   [Download Git](https://git-scm.com/downloads)

#### Install FastAPI and other requirements

Open a terminal and run the following coommands 

```bash
pip install fastapi "uvicorn[standard]" websocket-client opencv-python numpy
```

## Run the package

Then open a new terminal and clone the repository in your source workspace

example:

```bash
cd ~/ros2_ws/src
```

then

```bash
git clone https://github.com/eduardohufg/img_bridge
```