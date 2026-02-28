import os
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Empty


class ProcessorNode(Node):
    def __init__(self):
        super().__init__('processor_node')

        # Parameter: configurable multiplier
        self.declare_parameter('scale_factor', 2.0)
        self.scale_factor = float(self.get_parameter('scale_factor').value)

        # Parameter: where to save logs (directory)
        self.declare_parameter('log_dir', os.path.expanduser('~/ros2_logs'))
        self.log_dir = str(self.get_parameter('log_dir').value)

        # Internal state
        self.count = 0

        # Ensure log directory exists
        os.makedirs(self.log_dir, exist_ok=True)

        # Create a unique CSV file per run
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.log_dir, f'processor_log_{stamp}.csv')

        # Open CSV file and write header
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp_sec', 'count', 'sensor_value', 'processed_value', 'scale_factor'])
        self.csv_file.flush()

        # Subscriber
        self.subscription = self.create_subscription(
            Float64,
            'sensor_value',
            self.callback_sensor,
            10
        )

        # Publisher
        self.publisher_ = self.create_publisher(Float64, 'processed_value', 10)

        # Service: reset
        self.reset_srv = self.create_service(Empty, 'reset_processor', self.reset_callback)

        self.get_logger().info(f"ProcessorNode started. scale_factor={self.scale_factor}")
        self.get_logger().info(f"Logging CSV to: {self.csv_path}")

    def callback_sensor(self, msg: Float64):
        self.count += 1

        processed = msg.data * self.scale_factor

        out = Float64()
        out.data = processed
        self.publisher_.publish(out)

        # Time now (ROS clock)
        now = self.get_clock().now().nanoseconds / 1e9  # seconds

        # Write one CSV row
        self.csv_writer.writerow([f"{now:.9f}", self.count, f"{msg.data:.6f}", f"{out.data:.6f}", f"{self.scale_factor:.6f}"])
        self.csv_file.flush()

        self.get_logger().info(f"[{self.count}] {msg.data:.3f} -> {out.data:.3f}")

    def reset_callback(self, request, response):
        self.count = 0
        self.get_logger().info("Processor reset: count set to 0")
        return response

    def destroy_node(self):
        # Close CSV safely when node shuts down
        try:
            if hasattr(self, 'csv_file') and not self.csv_file.closed:
                self.csv_file.flush()
                self.csv_file.close()
        except Exception as e:
            self.get_logger().error(f"Failed to close CSV file: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
