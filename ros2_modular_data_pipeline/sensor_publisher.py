import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Create a publisher that publishes Float64 messages on the topic "/sensor_value"
        self.publisher_ = self.create_publisher(Float64, 'sensor_value', 10)

        # Timer: publish every 0.5 seconds
        self.timer = self.create_timer(0.5, self.publish_value)

    def publish_value(self):
        msg = Float64()
        msg.data = random.uniform(0.0, 10.0)  # fake sensor reading
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published sensor value: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
