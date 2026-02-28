import rclpy
from rclpy.node import Node

class SimpleNode(Node):
	def __init__(self):
		super().__init__('simple_node')
		self.timer = self.create_timer(1.0, self.timer_callback)

	def timer_callback(self):
		self.get_logger().info("Hello, I am running!")

def main(args=None):
	rclpy.init(args=args)
	node = SimpleNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

