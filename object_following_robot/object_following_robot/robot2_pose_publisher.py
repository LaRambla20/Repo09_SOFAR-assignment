import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

class PosePublisher(Node):

    def __init__(self):
        super().__init__('pose_publisher')

        # Declare and acquire `target_frame` parameter
        self.declare_parameter('target_frame', 'map')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a robot pose publisher
        self.publisher_ = self.create_publisher(PoseStamped, '/robot1/goal_pose', 10)

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

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = trans.transform.translation.x
        goal_msg.pose.position.y = trans.transform.translation.y
        goal_msg.pose.position.z = trans.transform.translation.z
        goal_msg.pose.orientation.w = trans.transform.rotation.w
        goal_msg.pose.orientation.x = trans.transform.rotation.x
        goal_msg.pose.orientation.y = trans.transform.rotation.y
        goal_msg.pose.orientation.z = trans.transform.rotation.z
        self.publisher_.publish(goal_msg)
        self.get_logger().info('\033[91m' + 'Publishing:'  + '\033[0m')
        self.get_logger().info('--- position - x: "%f"' % goal_msg.pose.position.x)
        self.get_logger().info('--- position - y: "%f"' % goal_msg.pose.position.y)
        self.get_logger().info('--- position - z: "%f"' % goal_msg.pose.position.z)
        self.get_logger().info('--- orientation - w: "%f"' % goal_msg.pose.orientation.w)
        self.get_logger().info('--- orientation - x: "%f"' % goal_msg.pose.orientation.x)
        self.get_logger().info('--- orientation - y: "%f"' % goal_msg.pose.orientation.y)
        self.get_logger().info('--- orientation - z: "%f"' % goal_msg.pose.orientation.z)


  


def main():
    rclpy.init()
    transform_pub = PosePublisher()
    
    rclpy.spin(transform_pub)
    transform_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()