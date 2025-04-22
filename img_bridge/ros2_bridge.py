import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from threading import Thread
from typing import List, Callable


class ROS2Bridge(Node):
    def __init__(self):
        super().__init__('fastapi_ros2_bridge')
        
        self._image_callbacks: List[Callable[[str], None]] = []


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
