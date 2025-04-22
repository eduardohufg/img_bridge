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

Example:

```bash
cd ~/ros2_ws/src
```

Then

```bash
git clone https://github.com/eduardohufg/img_bridge
```

Return to your workspace, compile and source the workspace

```bash
cd ..
colcon build
source install/setup.bash
```

You can run the gridge with the following command

```bash
ros2 run img_brigde img_bridge
```

##Example to use

For use the bridge, if you want to use it in a ROS package, create a directory named lib in your root package

```bash
mkdir lib
cd lib
```
Then create a empty python file to execute it like a library

```bash
touch __init__.py
```
Then create a file which will contain the library

```bash
touch ws_connection.py
```

And copy de following code in that file:

```bash
import websocket
import threading
import cv2
import numpy as np
import base64

class WsConnection:
    def __init__(self, ws_url):
        self.ws_url = ws_url
        self.frame = None
        self.lock = threading.Lock()

        self.ws_app = websocket.WebSocketApp(
            self.ws_url,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        self.thread = threading.Thread(target=self.ws_app.run_forever, daemon=True)
        self.thread.start()

    def on_open(self, ws):
        print("Conected to server")

    def on_message(self, ws, message):
        img_bytes = base64.b64decode(message)
        np_arr = np.frombuffer(img_bytes, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is not None:
            with self.lock:
                self.frame = frame

    def on_error(self, ws, error):
        print("Error:", error)

    def on_close(self, ws, close_status_code, close_msg):
        print("Conction closed")

    def get_frame(self):
        
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def close(self):

        self.ws_app.close()
        self.thread.join()
```