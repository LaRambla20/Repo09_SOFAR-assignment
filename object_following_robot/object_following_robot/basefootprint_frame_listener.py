import math
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import TransformStamped

class FrameListener(Node):

    def __init__(self):
        super().__init__('basefootprint_frame_listener')

        # Declare and acquire `target_frame` parameter
        self.declare_parameter('target_frame', 'map')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a robot pose publisher
        self.publisher = self.create_publisher(TransformStamped, '/pose_wrt_map', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Store frame names in variables that will be used to compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'base_footprint'

        # Try to get the transformation from <map> to <base_footprint>
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                from_frame_rel,
                to_frame_rel,
                now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # Publish the transformation on the deidcated topic
        msg = TransformStamped()
        msg = trans

        self.publisher.publish(msg)

  


def main():
    rclpy.init()
    node = FrameListener()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()