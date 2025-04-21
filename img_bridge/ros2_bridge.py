import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from rclpy.executors import SingleThreadedExecutor
from threading import Thread
from typing import List, Callable
import json  # Para parsear los datos del WebSocket
import math
import numpy as np
import cv2
import base64

class ROS2Bridge(Node):
    def __init__(self):
        super().__init__('fastapi_ros2_bridge')
        
        self._image_callbacks: List[Callable[[str], None]] = []

    def image_callback(self, msg: Image):
      
        try:
          
            height = msg.height
            width = msg.width
            channels = 3
            img_array = np.array(msg.data, dtype=np.uint8).reshape((height, width, channels))
            
            _, buffer = cv2.imencode(".jpg", img_array)
         
            frame_base64 = base64.b64encode(buffer).decode("utf-8")
            
            for cb in self._image_callbacks:
                cb(frame_base64)
        
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def register_image_callback(self, callback: Callable[[str], None]):
        self._image_callbacks.append(callback)

ros2_node: ROS2Bridge = None 
executor = None

def start_ros2():
    global ros2_node, executor
    rclpy.init()
    ros2_node = ROS2Bridge()  # Inicializa ros2_node aquí
    executor = SingleThreadedExecutor()
    executor.add_node(ros2_node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()

def stop_ros2():
    global ros2_node, executor
    executor.shutdown()
    ros2_node.destroy_node()
    rclpy.shutdown()

def get_ros2_node() -> ROS2Bridge:
    return ros2_node 
