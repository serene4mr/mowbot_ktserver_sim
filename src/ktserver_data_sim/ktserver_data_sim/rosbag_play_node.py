import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

class RosBagPlayNode(Node):
    def __init__(self):
        super().__init__('rosbag_play_node')
        # Declare the parameter with a default value
        self.declare_parameter('bag_path', '')
        bag_path = self.get_parameter('bag_path').get_parameter_value().string_value

        if not bag_path:
            self.get_logger().error('No bag_path parameter provided. Please set it with --ros-args -p bag_path:=/path/to/bag')
            rclpy.shutdown()
            return

        self.get_logger().info(f'RosBagPlayNode started with bag_path: {bag_path}')

        # Open the rosbag
        self.reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(
            uri=bag_path,
            storage_id='sqlite3'  # Use 'sqlite3' if your bag is in sqlite3 format
        )
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)

        # Prepare pubsfor all topics
        self.pubs= {}
        self.topics = self.reader.get_all_topics_and_types()
        for topic in self.topics:
            msg_type = get_message(topic.type)
            self.pubs[topic.name] = self.create_publisher(msg_type, topic.name, 10)

        # Read all messages into memory with their timestamps
        self.messages = []
        while self.reader.has_next():
            topic, data, timestamp = self.reader.read_next()
            msg_type = get_message(next(t.type for t in self.topics if t.name == topic))
            msg = deserialize_message(data, msg_type)
            self.messages.append((topic, msg, timestamp))

        # Start playback
        self.start_time = self.get_clock().now().nanoseconds
        self.current_index = 0
        self.timer = self.create_timer(0.001, self.playback_callback)

    def playback_callback(self):
        if self.current_index >= len(self.messages):
            self.get_logger().info('Bag playback finished.')
            self.timer.cancel()
            return

        now = self.get_clock().now().nanoseconds
        topic, msg, msg_time = self.messages[self.current_index]
        elapsed = now - self.start_time

        if elapsed >= msg_time:
            self.pubs[topic].publish(msg)
            self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = RosBagPlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
